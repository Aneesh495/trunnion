/*
 * TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
 * High-Performance ANSI Forth Virtual Machine & Hardware Simulation Runtime
 *
 * Copyright (c) 2026 Aneesh Krishna Parthasarathy
 * Licensed under the ISC License.
 */

#include "trunnion_vm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

/* Internal Opcode Constants for Threaded Code Execution (Negative to avoid dictionary collision) */
#define OP_EXIT        (-1)
#define OP_LIT         (-2)
#define OP_BRANCH      (-3)
#define OP_0BRANCH     (-4)
#define OP_DO          (-5)
#define OP_LOOP        (-6)
#define OP_PLOOP       (-7)
#define OP_STR_LIT     (-8)
#define OP_FLIT        (-9)

/* Hardware Register Offsets */
#define HW_REG_MAGIC        0x00
#define HW_REG_STATUS       0x01
#define HW_REG_CLOCK_LO     0x02
#define HW_REG_CLOCK_HI     0x03
#define HW_REG_CPA_AZ_POS   0x04
#define HW_REG_CPA_EL_POS   0x05
#define HW_REG_CPA_AZ_CMD   0x06
#define HW_REG_CPA_EL_CMD   0x07
#define HW_REG_FSM_TIP_CMD  0x08
#define HW_REG_FSM_TILT_CMD 0x09
#define HW_REG_FSM_TIP_FB   0x0A
#define HW_REG_FSM_TILT_FB  0x0B
#define HW_REG_QPD_A        0x0C
#define HW_REG_QPD_B        0x0D
#define HW_REG_QPD_C        0x0E
#define HW_REG_QPD_D        0x0F
#define HW_REG_LASER_STATUS 0x10
#define HW_REG_TEMP_LASER   0x11
#define HW_REG_RX_POWER     0x12
#define HW_REG_MODEM_STATUS 0x13
#define HW_REG_WATCHDOG     0x14

/* Forward declarations */
static void register_core_primitives(trunnion_vm_t *vm);
static void compile_cell(trunnion_vm_t *vm, cell_t val);
static int find_word(trunnion_vm_t *vm, const char *name);
static void execute_xt(trunnion_vm_t *vm, cell_t xt);
static char *next_token(const char **cursor, char *buf, size_t max_len);

trunnion_vm_t *trunnion_vm_create(void) {
    trunnion_vm_t *vm = (trunnion_vm_t *)calloc(1, sizeof(trunnion_vm_t));
    if (!vm) return NULL;

    vm->memory = (uint8_t *)calloc(1, TRUNNION_VM_MEMORY_SIZE);
    if (!vm->memory) {
        free(vm);
        return NULL;
    }

    vm->sp = 0;
    vm->rp = 0;
    vm->fp = 0;
    vm->here = 1024; /* Reserve bottom 1KB for system variables and boot structures */
    vm->num_words = 0;
    vm->latest_word = -1;
    vm->base = 10;
    vm->compiling = false;
    vm->running = true;
    vm->cs_sp = 0;

    /* Initialize Simulated Hardware Registers */
    vm->hw_regs[HW_REG_MAGIC] = 0x5452554E; /* 'TRUN' */
    vm->hw_regs[HW_REG_STATUS] = 0x0001;    /* Ready flag */
    vm->hw_regs[HW_REG_TEMP_LASER] = 293150; /* 293.15 K (20 deg C) in millikelvins */
    vm->hw_regs[HW_REG_RX_POWER] = 1500;    /* 1500 nW baseline background light */
    vm->hw_regs[HW_REG_QPD_A] = 2048;
    vm->hw_regs[HW_REG_QPD_B] = 2048;
    vm->hw_regs[HW_REG_QPD_C] = 2048;
    vm->hw_regs[HW_REG_QPD_D] = 2048;

    register_core_primitives(vm);
    return vm;
}

void trunnion_vm_destroy(trunnion_vm_t *vm) {
    if (!vm) return;
    if (vm->memory) free(vm->memory);
    free(vm);
}

void trunnion_vm_reset(trunnion_vm_t *vm) {
    if (!vm) return;
    vm->sp = 0;
    vm->rp = 0;
    vm->fp = 0;
    vm->cs_sp = 0;
    vm->case_sp = 0;
    vm->case_exit_sp = 0;
    vm->compiling = false;
    vm->running = true;
    vm->cond_skip_depth = 0;
}

void trunnion_vm_push(trunnion_vm_t *vm, cell_t val) {
    if (vm->sp < TRUNNION_VM_DATA_STACK_SIZE) {
        vm->data_stack[vm->sp++] = val;
    } else {
        fprintf(stderr, "[ERROR] Data stack overflow!\n");
    }
}

cell_t trunnion_vm_pop(trunnion_vm_t *vm) {
    if (vm->sp > 0) {
        return vm->data_stack[--vm->sp];
    }
    fprintf(stderr, "[ERROR] Data stack underflow at %s:%d!\n", vm->current_file, vm->current_line);
    return 0;
}

cell_t trunnion_vm_peek(trunnion_vm_t *vm) {
    if (vm->sp > 0) {
        return vm->data_stack[vm->sp - 1];
    }
    return 0;
}

void trunnion_vm_rpush(trunnion_vm_t *vm, cell_t val) {
    if (vm->rp < TRUNNION_VM_RETURN_STACK_SIZE) {
        vm->return_stack[vm->rp++] = val;
    } else {
        fprintf(stderr, "[ERROR] Return stack overflow!\n");
    }
}

cell_t trunnion_vm_rpop(trunnion_vm_t *vm) {
    if (vm->rp > 0) {
        return vm->return_stack[--vm->rp];
    }
    fprintf(stderr, "[ERROR] Return stack underflow!\n");
    return 0;
}

void trunnion_vm_fpush(trunnion_vm_t *vm, double val) {
    if (vm->fp < TRUNNION_VM_FLOAT_STACK_SIZE) {
        vm->float_stack[vm->fp++] = val;
    } else {
        fprintf(stderr, "[ERROR] Float stack overflow!\n");
    }
}

double trunnion_vm_fpop(trunnion_vm_t *vm) {
    if (vm->fp > 0) {
        return vm->float_stack[--vm->fp];
    }
    fprintf(stderr, "[ERROR] Float stack underflow!\n");
    return 0.0;
}

uint32_t trunnion_hw_read32(trunnion_vm_t *vm, uint32_t reg_offset) {
    if (reg_offset < TRUNNION_VM_HW_REGS_SIZE) {
        return vm->hw_regs[reg_offset];
    }
    return 0;
}

void trunnion_hw_write32(trunnion_vm_t *vm, uint32_t reg_offset, uint32_t value) {
    if (reg_offset < TRUNNION_VM_HW_REGS_SIZE) {
        vm->hw_regs[reg_offset] = value;
    }
}

void trunnion_hw_step(trunnion_vm_t *vm, uint64_t delta_us) {
    vm->simulated_time_us += delta_us;
    vm->tick_counter++;
    vm->hw_regs[HW_REG_CLOCK_LO] = (uint32_t)(vm->simulated_time_us & 0xFFFFFFFF);
    vm->hw_regs[HW_REG_CLOCK_HI] = (uint32_t)((vm->simulated_time_us >> 32) & 0xFFFFFFFF);
}

/* Dictionary Management */
static int add_word(trunnion_vm_t *vm, const char *name, uint8_t flags, primitive_fn prim, cell_t xt) {
    if (vm->num_words >= TRUNNION_VM_MAX_WORDS) {
        fprintf(stderr, "[ERROR] Dictionary full!\n");
        return -1;
    }
    int idx = vm->num_words++;
    word_entry_t *w = &vm->dictionary[idx];
    strncpy(w->name, name, TRUNNION_VM_WORD_NAME_MAX - 1);
    w->name[TRUNNION_VM_WORD_NAME_MAX - 1] = '\0';
    w->flags = flags;
    w->prim = prim;
    w->xt = xt;
    w->does_ip = 0;
    vm->latest_word = idx;
    return idx;
}

static int find_word(trunnion_vm_t *vm, const char *name) {
    for (int i = vm->num_words - 1; i >= 0; i--) {
        if (!(vm->dictionary[i].flags & FLAG_HIDDEN)) {
            if (strcasecmp(vm->dictionary[i].name, name) == 0) {
                return i;
            }
        }
    }
    return -1;
}

static void compile_cell(trunnion_vm_t *vm, cell_t val) {
    if (vm->here + (cell_t)sizeof(cell_t) <= TRUNNION_VM_MEMORY_SIZE) {
        *(cell_t *)(vm->memory + vm->here) = val;
        vm->here += sizeof(cell_t);
    } else {
        fprintf(stderr, "[ERROR] Memory capacity exceeded during compilation!\n");
    }
}

/* Threaded Execution Engine */
static void execute_xt(trunnion_vm_t *vm, cell_t xt) {
    if (xt >= 0 && xt < vm->num_words) {
        word_entry_t *w = &vm->dictionary[xt];
        if (w->flags & FLAG_PRIMITIVE) {
            if (w->prim) w->prim(vm);
            return;
        }

        /* Check if it is a CREATE...DOES> word */
        if (w->does_ip > 0) {
            trunnion_vm_push(vm, w->xt); /* Push parameter field address */
            /* Execute does_ip threaded sequence */
            cell_t saved_rp = vm->rp;
            trunnion_vm_rpush(vm, 0); /* Return address sentinel */
            cell_t pc = w->does_ip;

            while (vm->rp > saved_rp && vm->running) {
                cell_t token = *(cell_t *)(vm->memory + pc);
                pc += sizeof(cell_t);

                switch (token) {
                    case OP_EXIT:
                        pc = trunnion_vm_rpop(vm);
                        break;
                    case OP_LIT: {
                        cell_t lit = *(cell_t *)(vm->memory + pc);
                        pc += sizeof(cell_t);
                        trunnion_vm_push(vm, lit);
                        break;
                    }
                    case OP_FLIT: {
                        double flit = *(double *)(vm->memory + pc);
                        pc += sizeof(double);
                        trunnion_vm_fpush(vm, flit);
                        break;
                    }
                    case OP_BRANCH: {
                        cell_t offset = *(cell_t *)(vm->memory + pc);
                        pc += offset;
                        break;
                    }
                    case OP_0BRANCH: {
                        cell_t offset = *(cell_t *)(vm->memory + pc);
                        cell_t cond = trunnion_vm_pop(vm);
                        if (cond == 0) {
                            pc += offset;
                        } else {
                            pc += sizeof(cell_t);
                        }
                        break;
                    }
                    case OP_DO: {
                        cell_t start = trunnion_vm_pop(vm);
                        cell_t limit = trunnion_vm_pop(vm);
                        trunnion_vm_rpush(vm, limit);
                        trunnion_vm_rpush(vm, start);
                        break;
                    }
                    case OP_LOOP: {
                        cell_t offset = *(cell_t *)(vm->memory + pc);
                        cell_t idx = trunnion_vm_rpop(vm) + 1;
                        cell_t lim = trunnion_vm_rpop(vm);
                        if (idx < lim) {
                            trunnion_vm_rpush(vm, lim);
                            trunnion_vm_rpush(vm, idx);
                            pc += offset;
                        } else {
                            pc += sizeof(cell_t);
                        }
                        break;
                    }
                    case OP_PLOOP: {
                        cell_t offset = *(cell_t *)(vm->memory + pc);
                        cell_t delta = trunnion_vm_pop(vm);
                        cell_t idx = trunnion_vm_rpop(vm) + delta;
                        cell_t lim = trunnion_vm_rpop(vm);
                        if ((delta > 0 && idx < lim) || (delta < 0 && idx > lim)) {
                            trunnion_vm_rpush(vm, lim);
                            trunnion_vm_rpush(vm, idx);
                            pc += offset;
                        } else {
                            pc += sizeof(cell_t);
                        }
                        break;
                    }
                    case OP_STR_LIT: {
                        cell_t len = *(cell_t *)(vm->memory + pc);
                        pc += sizeof(cell_t);
                        cell_t str_addr = (cell_t)(uintptr_t)(vm->memory + pc);
                        pc = (pc + len + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                        trunnion_vm_push(vm, str_addr);
                        trunnion_vm_push(vm, len);
                        break;
                    }
                    default:
                        if (token >= 0 && token < vm->num_words) {
                            word_entry_t *sub = &vm->dictionary[token];
                            if (sub->flags & FLAG_PRIMITIVE) {
                                sub->prim(vm);
                            } else {
                                trunnion_vm_rpush(vm, pc);
                                pc = sub->xt;
                            }
                        }
                        break;
                }
            }
            return;
        }

        /* Standard Colon Definition Execution */
        cell_t saved_rp = vm->rp;
        trunnion_vm_rpush(vm, 0); /* Sentinel return address */
        cell_t pc = w->xt;

        while (vm->rp > saved_rp && vm->running) {
            cell_t token = *(cell_t *)(vm->memory + pc);
            pc += sizeof(cell_t);

            switch (token) {
                case OP_EXIT:
                    pc = trunnion_vm_rpop(vm);
                    break;
                case OP_LIT: {
                    cell_t lit = *(cell_t *)(vm->memory + pc);
                    pc += sizeof(cell_t);
                    trunnion_vm_push(vm, lit);
                    break;
                }
                case OP_FLIT: {
                    double flit = *(double *)(vm->memory + pc);
                    pc += sizeof(double);
                    trunnion_vm_fpush(vm, flit);
                    break;
                }
                case OP_BRANCH: {
                    cell_t offset = *(cell_t *)(vm->memory + pc);
                    pc += offset;
                    break;
                }
                case OP_0BRANCH: {
                    cell_t offset = *(cell_t *)(vm->memory + pc);
                    cell_t cond = trunnion_vm_pop(vm);
                    if (cond == 0) {
                        pc += offset;
                    } else {
                        pc += sizeof(cell_t);
                    }
                    break;
                }
                case OP_DO: {
                    cell_t start = trunnion_vm_pop(vm);
                    cell_t limit = trunnion_vm_pop(vm);
                    trunnion_vm_rpush(vm, limit);
                    trunnion_vm_rpush(vm, start);
                    break;
                }
                case OP_LOOP: {
                    cell_t offset = *(cell_t *)(vm->memory + pc);
                    cell_t idx = trunnion_vm_rpop(vm) + 1;
                    cell_t lim = trunnion_vm_rpop(vm);
                    if (idx < lim) {
                        trunnion_vm_rpush(vm, lim);
                        trunnion_vm_rpush(vm, idx);
                        pc += offset;
                    } else {
                        pc += sizeof(cell_t);
                    }
                    break;
                }
                case OP_PLOOP: {
                    cell_t offset = *(cell_t *)(vm->memory + pc);
                    cell_t delta = trunnion_vm_pop(vm);
                    cell_t idx = trunnion_vm_rpop(vm) + delta;
                    cell_t lim = trunnion_vm_rpop(vm);
                    if ((delta > 0 && idx < lim) || (delta < 0 && idx > lim)) {
                        trunnion_vm_rpush(vm, lim);
                        trunnion_vm_rpush(vm, idx);
                        pc += offset;
                    } else {
                        pc += sizeof(cell_t);
                    }
                    break;
                }
                case OP_STR_LIT: {
                    cell_t len = *(cell_t *)(vm->memory + pc);
                    pc += sizeof(cell_t);
                    cell_t str_addr = (cell_t)(uintptr_t)(vm->memory + pc);
                    pc = (pc + len + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                    trunnion_vm_push(vm, str_addr);
                    trunnion_vm_push(vm, len);
                    break;
                }
                default:
                    if (token >= 0 && token < vm->num_words) {
                        word_entry_t *sub = &vm->dictionary[token];
                        if (sub->flags & FLAG_PRIMITIVE) {
                            sub->prim(vm);
                        } else {
                            trunnion_vm_rpush(vm, pc);
                            pc = sub->xt;
                        }
                    }
                    break;
            }
        }
    }
}

/* Lexer and Tokenizer */
static char *next_token(const char **cursor, char *buf, size_t max_len) {
    const char *p = *cursor;
    while (*p && isspace((unsigned char)*p)) p++;

    if (!*p) {
        *cursor = p;
        return NULL;
    }

    size_t i = 0;
    while (*p && !isspace((unsigned char)*p) && i < max_len - 1) {
        buf[i++] = *p++;
    }
    buf[i] = '\0';
    *cursor = p;
    return buf;
}

static bool parse_number(const char *str, int base, cell_t *out_val) {
    char *endptr;
    long long val;
    if (str[0] == '$' || str[0] == '#') {
        val = strtoll(str + 1, &endptr, 16);
    } else if (str[0] == '%' && str[1] != '\0') {
        val = strtoll(str + 1, &endptr, 2);
    } else {
        val = strtoll(str, &endptr, base);
    }
    if (*endptr == '\0' && endptr != str) {
        *out_val = (cell_t)val;
        return true;
    }
    return false;
}

static bool parse_float(const char *str, double *out_val) {
    char *endptr;
    double val = strtod(str, &endptr);
    if (*endptr == '\0' && endptr != str && (strchr(str, '.') != NULL || strchr(str, 'e') != NULL || strchr(str, 'E') != NULL)) {
        *out_val = val;
        return true;
    }
    return false;
}

/* Core Word Primitives */

/* Stack Manipulation */
static void prim_dup(trunnion_vm_t *vm) { cell_t a = trunnion_vm_peek(vm); trunnion_vm_push(vm, a); }
static void prim_drop(trunnion_vm_t *vm) { trunnion_vm_pop(vm); }
static void prim_swap(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); cell_t b = trunnion_vm_pop(vm); trunnion_vm_push(vm, a); trunnion_vm_push(vm, b); }
static void prim_over(trunnion_vm_t *vm) {
    if (vm->sp >= 2) {
        cell_t val = vm->data_stack[vm->sp - 2];
        trunnion_vm_push(vm, val);
    }
}
static void prim_rot(trunnion_vm_t *vm) {
    cell_t c = trunnion_vm_pop(vm);
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    trunnion_vm_push(vm, b);
    trunnion_vm_push(vm, c);
    trunnion_vm_push(vm, a);
}
static void prim_nrot(trunnion_vm_t *vm) {
    cell_t c = trunnion_vm_pop(vm);
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    trunnion_vm_push(vm, c);
    trunnion_vm_push(vm, a);
    trunnion_vm_push(vm, b);
}
static void prim_nip(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_pop(vm); trunnion_vm_push(vm, a); }
static void prim_tuck(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); cell_t b = trunnion_vm_pop(vm); trunnion_vm_push(vm, a); trunnion_vm_push(vm, b); trunnion_vm_push(vm, a); }
static void prim_2dup(trunnion_vm_t *vm) {
    if (vm->sp >= 2) {
        cell_t a = vm->data_stack[vm->sp - 2];
        cell_t b = vm->data_stack[vm->sp - 1];
        trunnion_vm_push(vm, a);
        trunnion_vm_push(vm, b);
    }
}
static void prim_2drop(trunnion_vm_t *vm) { trunnion_vm_pop(vm); trunnion_vm_pop(vm); }
static void prim_2swap(trunnion_vm_t *vm) {
    cell_t d = trunnion_vm_pop(vm);
    cell_t c = trunnion_vm_pop(vm);
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    trunnion_vm_push(vm, c);
    trunnion_vm_push(vm, d);
    trunnion_vm_push(vm, a);
    trunnion_vm_push(vm, b);
}
static void prim_2over(trunnion_vm_t *vm) {
    if (vm->sp >= 4) {
        cell_t a = vm->data_stack[vm->sp - 4];
        cell_t b = vm->data_stack[vm->sp - 3];
        trunnion_vm_push(vm, a);
        trunnion_vm_push(vm, b);
    }
}
static void prim_qdup(trunnion_vm_t *vm) { cell_t a = trunnion_vm_peek(vm); if (a != 0) trunnion_vm_push(vm, a); }
static void prim_depth(trunnion_vm_t *vm) { trunnion_vm_push(vm, vm->sp); }
static void prim_pick(trunnion_vm_t *vm) {
    cell_t n = trunnion_vm_pop(vm);
    if (vm->sp > n && n >= 0) {
        trunnion_vm_push(vm, vm->data_stack[vm->sp - 1 - n]);
    }
}

/* Return Stack Operations */
static void prim_to_r(trunnion_vm_t *vm) { trunnion_vm_rpush(vm, trunnion_vm_pop(vm)); }
static void prim_r_from(trunnion_vm_t *vm) { trunnion_vm_push(vm, trunnion_vm_rpop(vm)); }
static void prim_r_fetch(trunnion_vm_t *vm) {
    if (vm->rp > 0) trunnion_vm_push(vm, vm->return_stack[vm->rp - 1]);
}
static void prim_2to_r(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    trunnion_vm_rpush(vm, a);
    trunnion_vm_rpush(vm, b);
}
static void prim_2r_from(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_rpop(vm);
    cell_t a = trunnion_vm_rpop(vm);
    trunnion_vm_push(vm, a);
    trunnion_vm_push(vm, b);
}
static void prim_2r_fetch(trunnion_vm_t *vm) {
    if (vm->rp >= 2) {
        trunnion_vm_push(vm, vm->return_stack[vm->rp - 2]);
        trunnion_vm_push(vm, vm->return_stack[vm->rp - 1]);
    }
}
static void prim_i(trunnion_vm_t *vm) {
    if (vm->rp > 0) trunnion_vm_push(vm, vm->return_stack[vm->rp - 1]);
}
static void prim_j(trunnion_vm_t *vm) {
    if (vm->rp >= 3) trunnion_vm_push(vm, vm->return_stack[vm->rp - 3]);
}
static void prim_k(trunnion_vm_t *vm) {
    if (vm->rp >= 5) trunnion_vm_push(vm, vm->return_stack[vm->rp - 5]);
}
static void prim_unloop(trunnion_vm_t *vm) {
    if (vm->rp >= 2) {
        trunnion_vm_rpop(vm);
        trunnion_vm_rpop(vm);
    }
}

/* Arithmetic & Bitwise */
static void prim_add(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a + b); }
static void prim_sub(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a - b); }
static void prim_mul(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a * b); }
static void prim_div(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm);
    if (b != 0) trunnion_vm_push(vm, a / b);
    else trunnion_vm_push(vm, 0);
}
static void prim_mod(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm);
    if (b != 0) trunnion_vm_push(vm, a % b);
    else trunnion_vm_push(vm, 0);
}
static void prim_divmod(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm);
    if (b != 0) {
        trunnion_vm_push(vm, a % b);
        trunnion_vm_push(vm, a / b);
    } else {
        trunnion_vm_push(vm, 0);
        trunnion_vm_push(vm, 0);
    }
}
static void prim_star_slash(trunnion_vm_t *vm) {
    cell_t c = trunnion_vm_pop(vm);
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    if (c != 0) {
        __int128 prod = (__int128)a * (__int128)b;
        trunnion_vm_push(vm, (cell_t)(prod / c));
    } else {
        trunnion_vm_push(vm, 0);
    }
}
static void prim_sm_rem(trunnion_vm_t *vm) {
    cell_t n1 = trunnion_vm_pop(vm);
    cell_t d1_hi = trunnion_vm_pop(vm);
    cell_t d1_lo = trunnion_vm_pop(vm);
    if (n1 == 0) {
        trunnion_vm_push(vm, 0);
        trunnion_vm_push(vm, 0);
        return;
    }
    __int128 d1 = ((__int128)d1_hi << 64) | (uint64_t)d1_lo;
    __int128 quot = d1 / n1;
    __int128 rem = d1 % n1;
    trunnion_vm_push(vm, (cell_t)rem);
    trunnion_vm_push(vm, (cell_t)quot);
}
static void prim_um_mod(trunnion_vm_t *vm) {
    uint64_t u1 = (uint64_t)trunnion_vm_pop(vm);
    uint64_t ud1_hi = (uint64_t)trunnion_vm_pop(vm);
    uint64_t ud1_lo = (uint64_t)trunnion_vm_pop(vm);
    if (u1 == 0) {
        trunnion_vm_push(vm, 0);
        trunnion_vm_push(vm, 0);
        return;
    }
    unsigned __int128 ud1 = ((unsigned __int128)ud1_hi << 64) | ud1_lo;
    unsigned __int128 quot = ud1 / u1;
    unsigned __int128 rem = ud1 % u1;
    trunnion_vm_push(vm, (cell_t)rem);
    trunnion_vm_push(vm, (cell_t)quot);
}
static void prim_star_slash_mod(trunnion_vm_t *vm) {
    cell_t c = trunnion_vm_pop(vm);
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    if (c != 0) {
        __int128 prod = (__int128)a * (__int128)b;
        trunnion_vm_push(vm, (cell_t)(prod % c));
        trunnion_vm_push(vm, (cell_t)(prod / c));
    } else {
        trunnion_vm_push(vm, 0);
        trunnion_vm_push(vm, 0);
    }
}
static void prim_1plus(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a + 1); }
static void prim_1minus(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a - 1); }
static void prim_2plus(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a + 2); }
static void prim_2minus(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a - 2); }
static void prim_2mul(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a << 1); }
static void prim_2div(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a >> 1); }
static void prim_negate(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, -a); }
static void prim_abs(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a < 0 ? -a : a); }
static void prim_min(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a < b ? a : b); }
static void prim_max(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a > b ? a : b); }

static void prim_and(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a & b); }
static void prim_or(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a | b); }
static void prim_xor(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a ^ b); }
static void prim_invert(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, ~a); }
static void prim_lshift(trunnion_vm_t *vm) { cell_t u = trunnion_vm_pop(vm); cell_t x = trunnion_vm_pop(vm); trunnion_vm_push(vm, (cell_t)((ucell_t)x << u)); }
static void prim_rshift(trunnion_vm_t *vm) { cell_t u = trunnion_vm_pop(vm); cell_t x = trunnion_vm_pop(vm); trunnion_vm_push(vm, (cell_t)((ucell_t)x >> u)); }
static void prim_arshift(trunnion_vm_t *vm) { cell_t u = trunnion_vm_pop(vm); cell_t x = trunnion_vm_pop(vm); trunnion_vm_push(vm, x >> u); }

/* Comparison */
static void prim_eq(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a == b ? -1 : 0); }
static void prim_neq(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a != b ? -1 : 0); }
static void prim_lt(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a < b ? -1 : 0); }
static void prim_gt(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a > b ? -1 : 0); }
static void prim_lte(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a <= b ? -1 : 0); }
static void prim_gte(trunnion_vm_t *vm) { cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a >= b ? -1 : 0); }
static void prim_0eq(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a == 0 ? -1 : 0); }
static void prim_0neq(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a != 0 ? -1 : 0); }
static void prim_0lt(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a < 0 ? -1 : 0); }
static void prim_0gt(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a > 0 ? -1 : 0); }
static void prim_0lte(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a <= 0 ? -1 : 0); }
static void prim_0gte(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a >= 0 ? -1 : 0); }
static void prim_ult(trunnion_vm_t *vm) { ucell_t b = (ucell_t)trunnion_vm_pop(vm); ucell_t a = (ucell_t)trunnion_vm_pop(vm); trunnion_vm_push(vm, a < b ? -1 : 0); }
static void prim_ugt(trunnion_vm_t *vm) { ucell_t b = (ucell_t)trunnion_vm_pop(vm); ucell_t a = (ucell_t)trunnion_vm_pop(vm); trunnion_vm_push(vm, a > b ? -1 : 0); }

/* Memory Operations */
static void prim_fetch(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + (cell_t)sizeof(cell_t) <= TRUNNION_VM_MEMORY_SIZE) {
        trunnion_vm_push(vm, *(cell_t *)(vm->memory + addr));
    } else {
        trunnion_vm_push(vm, 0);
    }
}
static void prim_store(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    cell_t val = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + (cell_t)sizeof(cell_t) <= TRUNNION_VM_MEMORY_SIZE) {
        *(cell_t *)(vm->memory + addr) = val;
        if (addr == 0) {
            vm->base = (int)val;
        }
    }
}
static void prim_cfetch(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    if (addr >= 0 && addr < TRUNNION_VM_MEMORY_SIZE) {
        trunnion_vm_push(vm, (uint8_t)vm->memory[addr]);
    } else {
        trunnion_vm_push(vm, 0);
    }
}
static void prim_cstore(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    cell_t val = trunnion_vm_pop(vm);
    if (addr >= 0 && addr < TRUNNION_VM_MEMORY_SIZE) {
        vm->memory[addr] = (uint8_t)val;
    }
}
static void prim_wfetch(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + 2 <= TRUNNION_VM_MEMORY_SIZE) {
        trunnion_vm_push(vm, *(uint16_t *)(vm->memory + addr));
    } else {
        trunnion_vm_push(vm, 0);
    }
}
static void prim_wstore(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    cell_t val = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + 2 <= TRUNNION_VM_MEMORY_SIZE) {
        *(uint16_t *)(vm->memory + addr) = (uint16_t)val;
    }
}
static void prim_2fetch(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + 2 * (cell_t)sizeof(cell_t) <= TRUNNION_VM_MEMORY_SIZE) {
        cell_t d_lo = *(cell_t *)(vm->memory + addr);
        cell_t d_hi = *(cell_t *)(vm->memory + addr + sizeof(cell_t));
        trunnion_vm_push(vm, d_lo);
        trunnion_vm_push(vm, d_hi);
    } else {
        trunnion_vm_push(vm, 0);
        trunnion_vm_push(vm, 0);
    }
}
static void prim_2store(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    cell_t d_hi = trunnion_vm_pop(vm);
    cell_t d_lo = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + 2 * (cell_t)sizeof(cell_t) <= TRUNNION_VM_MEMORY_SIZE) {
        *(cell_t *)(vm->memory + addr) = d_lo;
        *(cell_t *)(vm->memory + addr + sizeof(cell_t)) = d_hi;
    }
}
static void prim_plus_store(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    cell_t val = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + (cell_t)sizeof(cell_t) <= TRUNNION_VM_MEMORY_SIZE) {
        *(cell_t *)(vm->memory + addr) += val;
    }
}
static void prim_move(trunnion_vm_t *vm) {
    cell_t u = trunnion_vm_pop(vm);
    cell_t dest = trunnion_vm_pop(vm);
    cell_t src = trunnion_vm_pop(vm);
    if (u > 0 && src >= 0 && dest >= 0 && src + u <= TRUNNION_VM_MEMORY_SIZE && dest + u <= TRUNNION_VM_MEMORY_SIZE) {
        memmove(vm->memory + dest, vm->memory + src, u);
    }
}
static void prim_fill(trunnion_vm_t *vm) {
    cell_t ch = trunnion_vm_pop(vm);
    cell_t u = trunnion_vm_pop(vm);
    cell_t addr = trunnion_vm_pop(vm);
    if (u > 0 && addr >= 0 && addr + u <= TRUNNION_VM_MEMORY_SIZE) {
        memset(vm->memory + addr, (int)ch, u);
    }
}
static void prim_erase(trunnion_vm_t *vm) {
    cell_t u = trunnion_vm_pop(vm);
    cell_t addr = trunnion_vm_pop(vm);
    if (u > 0 && addr >= 0 && addr + u <= TRUNNION_VM_MEMORY_SIZE) {
        memset(vm->memory + addr, 0, u);
    }
}
static void prim_cmove(trunnion_vm_t *vm) {
    cell_t u = trunnion_vm_pop(vm);
    cell_t dst = trunnion_vm_pop(vm);
    cell_t src = trunnion_vm_pop(vm);
    if (u > 0) {
        uint8_t *src_ptr = (src >= 0 && src < TRUNNION_VM_MEMORY_SIZE) ? vm->memory + src : (uint8_t *)(uintptr_t)src;
        uint8_t *dst_ptr = (dst >= 0 && dst < TRUNNION_VM_MEMORY_SIZE) ? vm->memory + dst : (uint8_t *)(uintptr_t)dst;
        memmove(dst_ptr, src_ptr, (size_t)u);
    }
}

/* Dictionary Allocation */
static void prim_here(trunnion_vm_t *vm) { trunnion_vm_push(vm, vm->here); }
static void prim_allot(trunnion_vm_t *vm) {
    cell_t n = trunnion_vm_pop(vm);
    vm->here = (vm->here + n + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
}
static void prim_comma(trunnion_vm_t *vm) { compile_cell(vm, trunnion_vm_pop(vm)); }
static void prim_ccomma(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm);
    if (vm->here < TRUNNION_VM_MEMORY_SIZE) {
        vm->memory[vm->here++] = (uint8_t)b;
    }
}
static void prim_align(trunnion_vm_t *vm) {
    vm->here = (vm->here + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
}
static void prim_aligned(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    trunnion_vm_push(vm, (addr + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1));
}
static void prim_cell_plus(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_push(vm, a + sizeof(cell_t)); }
static void prim_cells(trunnion_vm_t *vm) { cell_t n = trunnion_vm_pop(vm); trunnion_vm_push(vm, n * sizeof(cell_t)); }

/* Double Cell Arithmetic */
static void prim_dplus(trunnion_vm_t *vm) {
    cell_t d2_hi = trunnion_vm_pop(vm); cell_t d2_lo = trunnion_vm_pop(vm);
    cell_t d1_hi = trunnion_vm_pop(vm); cell_t d1_lo = trunnion_vm_pop(vm);
    __int128 a = ((__int128)d1_hi << 64) | (ucell_t)d1_lo;
    __int128 b = ((__int128)d2_hi << 64) | (ucell_t)d2_lo;
    __int128 res = a + b;
    trunnion_vm_push(vm, (cell_t)res);
    trunnion_vm_push(vm, (cell_t)(res >> 64));
}
static void prim_q32_mul(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    __int128 prod = (__int128)a * (__int128)b;
    trunnion_vm_push(vm, (cell_t)(prod >> 32));
}
static void prim_q32_div(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm);
    cell_t a = trunnion_vm_pop(vm);
    if (b != 0) {
        __int128 num = (__int128)a << 32;
        trunnion_vm_push(vm, (cell_t)(num / b));
    } else {
        trunnion_vm_push(vm, 0);
    }
}
static void prim_dminus(trunnion_vm_t *vm) {
    cell_t d2_hi = trunnion_vm_pop(vm); cell_t d2_lo = trunnion_vm_pop(vm);
    cell_t d1_hi = trunnion_vm_pop(vm); cell_t d1_lo = trunnion_vm_pop(vm);
    __int128 a = ((__int128)d1_hi << 64) | (ucell_t)d1_lo;
    __int128 b = ((__int128)d2_hi << 64) | (ucell_t)d2_lo;
    __int128 res = a - b;
    trunnion_vm_push(vm, (cell_t)res);
    trunnion_vm_push(vm, (cell_t)(res >> 64));
}
static void prim_dnegate(trunnion_vm_t *vm) {
    cell_t hi = trunnion_vm_pop(vm); cell_t lo = trunnion_vm_pop(vm);
    __int128 a = ((__int128)hi << 64) | (ucell_t)lo;
    __int128 res = -a;
    trunnion_vm_push(vm, (cell_t)res);
    trunnion_vm_push(vm, (cell_t)(res >> 64));
}
static void prim_mstar(trunnion_vm_t *vm) {
    cell_t b = trunnion_vm_pop(vm); cell_t a = trunnion_vm_pop(vm);
    __int128 res = (__int128)a * (__int128)b;
    trunnion_vm_push(vm, (cell_t)res);
    trunnion_vm_push(vm, (cell_t)(res >> 64));
}
static void prim_umstar(trunnion_vm_t *vm) {
    ucell_t b = (ucell_t)trunnion_vm_pop(vm); ucell_t a = (ucell_t)trunnion_vm_pop(vm);
    unsigned __int128 res = (unsigned __int128)a * (unsigned __int128)b;
    trunnion_vm_push(vm, (cell_t)res);
    trunnion_vm_push(vm, (cell_t)(res >> 64));
}

/* Floating Point Primitives */
static void prim_fadd(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, a + b); }
static void prim_fsub(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, a - b); }
static void prim_fmul(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, a * b); }
static void prim_fdiv(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, b != 0.0 ? a / b : 0.0); }
static void prim_fneg(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, -a); }
static void prim_fabs(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, fabs(a)); }
static void prim_feq(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_push(vm, a == b ? -1 : 0); }
static void prim_flt(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_push(vm, a < b ? -1 : 0); }
static void prim_fgt(trunnion_vm_t *vm) { double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm); trunnion_vm_push(vm, a > b ? -1 : 0); }
static void prim_fdup(trunnion_vm_t *vm) { if (vm->fp > 0) { double val = vm->float_stack[vm->fp - 1]; trunnion_vm_fpush(vm, val); } }
static void prim_fdrop(trunnion_vm_t *vm) { trunnion_vm_fpop(vm); }
static void prim_fswap(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); double b = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, a); trunnion_vm_fpush(vm, b); }
static void prim_fover(trunnion_vm_t *vm) { if (vm->fp >= 2) trunnion_vm_fpush(vm, vm->float_stack[vm->fp - 2]); }
static void prim_frot(trunnion_vm_t *vm) {
    double c = trunnion_vm_fpop(vm); double b = trunnion_vm_fpop(vm); double a = trunnion_vm_fpop(vm);
    trunnion_vm_fpush(vm, b); trunnion_vm_fpush(vm, c); trunnion_vm_fpush(vm, a);
}
static void prim_fsin(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, sin(a)); }
static void prim_fcos(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, cos(a)); }
static void prim_ftan(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, tan(a)); }
static void prim_fatan2(trunnion_vm_t *vm) { double x = trunnion_vm_fpop(vm); double y = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, atan2(y, x)); }
static void prim_fsqrt(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, a >= 0.0 ? sqrt(a) : 0.0); }
static void prim_fexp(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, exp(a)); }
static void prim_fln(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, a > 0.0 ? log(a) : 0.0); }
static void prim_fpow(trunnion_vm_t *vm) { double y = trunnion_vm_fpop(vm); double x = trunnion_vm_fpop(vm); trunnion_vm_fpush(vm, pow(x, y)); }
static void prim_stof(trunnion_vm_t *vm) { cell_t a = trunnion_vm_pop(vm); trunnion_vm_fpush(vm, (double)a); }
static void prim_ftos(trunnion_vm_t *vm) { double a = trunnion_vm_fpop(vm); trunnion_vm_push(vm, (cell_t)a); }
static void prim_ffetch(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    if (addr >= 0 && addr + (cell_t)sizeof(double) <= TRUNNION_VM_MEMORY_SIZE) {
        trunnion_vm_fpush(vm, *(double *)(vm->memory + addr));
    } else {
        trunnion_vm_fpush(vm, 0.0);
    }
}
static void prim_fstore(trunnion_vm_t *vm) {
    cell_t addr = trunnion_vm_pop(vm);
    double val = trunnion_vm_fpop(vm);
    if (addr >= 0 && addr + (cell_t)sizeof(double) <= TRUNNION_VM_MEMORY_SIZE) {
        *(double *)(vm->memory + addr) = val;
    }
}

/* Output & Formatting */
static void prim_dot(trunnion_vm_t *vm) {
    cell_t val = trunnion_vm_pop(vm);
    if (vm->base == 16) {
        printf("%llx ", (unsigned long long)val);
    } else {
        printf("%lld ", (long long)val);
    }
    fflush(stdout);
}
static void prim_udot(trunnion_vm_t *vm) {
    ucell_t val = (ucell_t)trunnion_vm_pop(vm);
    if (vm->base == 16) {
        printf("%llx ", (unsigned long long)val);
    } else {
        printf("%llu ", (unsigned long long)val);
    }
    fflush(stdout);
}
static void prim_fdot(trunnion_vm_t *vm) {
    double val = trunnion_vm_fpop(vm);
    printf("%.8g ", val);
    fflush(stdout);
}
static void prim_dots(trunnion_vm_t *vm) {
    printf("<%d> ", vm->sp);
    for (int i = 0; i < vm->sp; i++) {
        if (vm->base == 16) printf("%llx ", (unsigned long long)vm->data_stack[i]);
        else printf("%lld ", (long long)vm->data_stack[i]);
    }
    printf("\n");
}
static void prim_cr(trunnion_vm_t *vm) { (void)vm; printf("\n"); fflush(stdout); }
static void prim_space(trunnion_vm_t *vm) { (void)vm; putchar(' '); fflush(stdout); }
static void prim_spaces(trunnion_vm_t *vm) {
    cell_t n = trunnion_vm_pop(vm);
    for (cell_t i = 0; i < n; i++) putchar(' ');
    fflush(stdout);
}
static void prim_emit(trunnion_vm_t *vm) {
    cell_t ch = trunnion_vm_pop(vm);
    putchar((int)ch);
    fflush(stdout);
}
static void prim_type(trunnion_vm_t *vm) {
    cell_t u = trunnion_vm_pop(vm);
    cell_t addr = trunnion_vm_pop(vm);
    if (u > 0 && addr != 0) {
        const char *p = (const char *)(uintptr_t)addr;
        for (cell_t i = 0; i < u; i++) putchar(p[i]);
        fflush(stdout);
    }
}
static void prim_hex(trunnion_vm_t *vm) { vm->base = 16; }
static void prim_decimal(trunnion_vm_t *vm) { vm->base = 10; }
static void prim_binary(trunnion_vm_t *vm) { vm->base = 2; }
static void prim_base_addr(trunnion_vm_t *vm) {
    /* Push pointer to base inside system memory */
    cell_t base_addr = 0; /* Offset 0 in linear memory holds base */
    *(cell_t *)(vm->memory + base_addr) = vm->base;
    trunnion_vm_push(vm, base_addr);
}
static void prim_base_fetch(trunnion_vm_t *vm) { trunnion_vm_push(vm, vm->base); }
static void prim_base_store(trunnion_vm_t *vm) { vm->base = (int)trunnion_vm_pop(vm); }
static void prim_bye(trunnion_vm_t *vm) { vm->running = false; }

/* Timing & Microseconds */
static void prim_ms(trunnion_vm_t *vm) {
    cell_t n = trunnion_vm_pop(vm);
    struct timespec ts;
    ts.tv_sec = n / 1000;
    ts.tv_nsec = (n % 1000) * 1000000;
    nanosleep(&ts, NULL);
}
static void prim_utime(trunnion_vm_t *vm) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
    trunnion_vm_push(vm, (cell_t)(us & 0xFFFFFFFF));
    trunnion_vm_push(vm, (cell_t)(us >> 32));
}

/* Simulated Hardware Bus Primitives */
static void prim_hw_fetch(trunnion_vm_t *vm) {
    cell_t reg = trunnion_vm_pop(vm);
    uint32_t val = trunnion_hw_read32(vm, (uint32_t)reg);
    trunnion_vm_push(vm, (cell_t)val);
}
static void prim_hw_store(trunnion_vm_t *vm) {
    cell_t reg = trunnion_vm_pop(vm);
    cell_t val = trunnion_vm_pop(vm);
    trunnion_hw_write32(vm, (uint32_t)reg, (uint32_t)val);
}
static void prim_hw_step(trunnion_vm_t *vm) {
    cell_t dt_us = trunnion_vm_pop(vm);
    trunnion_hw_step(vm, (uint64_t)dt_us);
}
static void prim_hw_clock(trunnion_vm_t *vm) {
    trunnion_vm_push(vm, (cell_t)(vm->simulated_time_us & 0xFFFFFFFF));
    trunnion_vm_push(vm, (cell_t)(vm->simulated_time_us >> 32));
}
static void prim_hw_log(trunnion_vm_t *vm) {
    cell_t u = trunnion_vm_pop(vm);
    cell_t addr = trunnion_vm_pop(vm);
    if (u > 0 && addr != 0) {
        printf("[PAYLOAD LOG @ %llu us] ", (unsigned long long)vm->simulated_time_us);
        const char *p = (const char *)(uintptr_t)addr;
        for (cell_t i = 0; i < u; i++) putchar(p[i]);
        printf("\n");
        fflush(stdout);
    }
}

/* Defining Words and Compilation */
static void prim_semicolon(trunnion_vm_t *vm) {
    compile_cell(vm, OP_EXIT);
    vm->compiling = false;
}
static void prim_immediate(trunnion_vm_t *vm) {
    if (vm->latest_word >= 0 && vm->latest_word < vm->num_words) {
        vm->dictionary[vm->latest_word].flags |= FLAG_IMMEDIATE;
    }
}

/* Control Structure Compilation Primitives (Immediate) */
static void prim_if(trunnion_vm_t *vm) {
    compile_cell(vm, OP_0BRANCH);
    vm->cs_stack[vm->cs_sp++] = vm->here;
    compile_cell(vm, 0); /* Placeholder */
}
static void prim_else(trunnion_vm_t *vm) {
    compile_cell(vm, OP_BRANCH);
    cell_t new_placeholder = vm->here;
    compile_cell(vm, 0);
    cell_t orig = vm->cs_stack[--vm->cs_sp];
    *(cell_t *)(vm->memory + orig) = vm->here - orig;
    vm->cs_stack[vm->cs_sp++] = new_placeholder;
}
static void prim_then(trunnion_vm_t *vm) {
    cell_t orig = vm->cs_stack[--vm->cs_sp];
    *(cell_t *)(vm->memory + orig) = vm->here - orig;
}
static void prim_begin(trunnion_vm_t *vm) {
    vm->cs_stack[vm->cs_sp++] = vm->here;
}
static void prim_until(trunnion_vm_t *vm) {
    cell_t dest = vm->cs_stack[--vm->cs_sp];
    compile_cell(vm, OP_0BRANCH);
    compile_cell(vm, dest - vm->here);
}
static void prim_again(trunnion_vm_t *vm) {
    cell_t dest = vm->cs_stack[--vm->cs_sp];
    compile_cell(vm, OP_BRANCH);
    compile_cell(vm, dest - vm->here);
}
static void prim_while(trunnion_vm_t *vm) {
    compile_cell(vm, OP_0BRANCH);
    vm->cs_stack[vm->cs_sp++] = vm->here;
    compile_cell(vm, 0);
}
static void prim_repeat(trunnion_vm_t *vm) {
    cell_t orig = vm->cs_stack[--vm->cs_sp];
    cell_t dest = vm->cs_stack[--vm->cs_sp];
    compile_cell(vm, OP_BRANCH);
    compile_cell(vm, dest - vm->here);
    *(cell_t *)(vm->memory + orig) = vm->here - orig;
}
static void prim_do(trunnion_vm_t *vm) {
    compile_cell(vm, OP_DO);
    vm->cs_stack[vm->cs_sp++] = vm->here;
}
static void prim_loop(trunnion_vm_t *vm) {
    cell_t dest = vm->cs_stack[--vm->cs_sp];
    compile_cell(vm, OP_LOOP);
    compile_cell(vm, dest - vm->here);
}
static void prim_plus_loop(trunnion_vm_t *vm) {
    cell_t dest = vm->cs_stack[--vm->cs_sp];
    compile_cell(vm, OP_PLOOP);
    compile_cell(vm, dest - vm->here);
}
static void prim_recurse(trunnion_vm_t *vm) {
    if (vm->latest_word >= 0) {
        compile_cell(vm, vm->latest_word);
    }
}
static void prim_exit(trunnion_vm_t *vm) {
    if (vm->compiling) {
        compile_cell(vm, OP_EXIT);
    } else {
        if (vm->rp > 0) vm->ip = trunnion_vm_rpop(vm);
    }
}

static void prim_execute(trunnion_vm_t *vm) {
    cell_t xt = trunnion_vm_pop(vm);
    execute_xt(vm, xt);
}

static void prim_case(trunnion_vm_t *vm) {
    if (vm->case_sp < 32) {
        vm->case_stack[vm->case_sp++] = vm->case_exit_sp;
    }
}

static void prim_of(trunnion_vm_t *vm) {
    compile_cell(vm, find_word(vm, "OVER"));
    compile_cell(vm, find_word(vm, "="));
    compile_cell(vm, OP_0BRANCH);
    vm->cs_stack[vm->cs_sp++] = vm->here;
    compile_cell(vm, 0);
    compile_cell(vm, find_word(vm, "DROP"));
}

static void prim_endof(trunnion_vm_t *vm) {
    compile_cell(vm, OP_BRANCH);
    if (vm->case_exit_sp < 128) {
        vm->case_exits[vm->case_exit_sp++] = vm->here;
    }
    compile_cell(vm, 0);
    cell_t false_orig = vm->cs_stack[--vm->cs_sp];
    *(cell_t *)(vm->memory + false_orig) = vm->here - false_orig;
}

static void prim_endcase(trunnion_vm_t *vm) {
    compile_cell(vm, find_word(vm, "DROP"));
    if (vm->case_sp > 0) {
        int start_exit = (int)vm->case_stack[--vm->case_sp];
        while (vm->case_exit_sp > start_exit) {
            cell_t exit_orig = vm->case_exits[--vm->case_exit_sp];
            *(cell_t *)(vm->memory + exit_orig) = vm->here - exit_orig;
        }
    }
}

/* Register Core Primitives */
static void register_core_primitives(trunnion_vm_t *vm) {
    /* Stack */
    add_word(vm, "DUP", FLAG_PRIMITIVE, prim_dup, 0);
    add_word(vm, "DROP", FLAG_PRIMITIVE, prim_drop, 0);
    add_word(vm, "SWAP", FLAG_PRIMITIVE, prim_swap, 0);
    add_word(vm, "OVER", FLAG_PRIMITIVE, prim_over, 0);
    add_word(vm, "ROT", FLAG_PRIMITIVE, prim_rot, 0);
    add_word(vm, "-ROT", FLAG_PRIMITIVE, prim_nrot, 0);
    add_word(vm, "NIP", FLAG_PRIMITIVE, prim_nip, 0);
    add_word(vm, "TUCK", FLAG_PRIMITIVE, prim_tuck, 0);
    add_word(vm, "2DUP", FLAG_PRIMITIVE, prim_2dup, 0);
    add_word(vm, "2DROP", FLAG_PRIMITIVE, prim_2drop, 0);
    add_word(vm, "2SWAP", FLAG_PRIMITIVE, prim_2swap, 0);
    add_word(vm, "2OVER", FLAG_PRIMITIVE, prim_2over, 0);
    add_word(vm, "?DUP", FLAG_PRIMITIVE, prim_qdup, 0);
    add_word(vm, "DEPTH", FLAG_PRIMITIVE, prim_depth, 0);
    add_word(vm, "PICK", FLAG_PRIMITIVE, prim_pick, 0);

    /* Return Stack */
    add_word(vm, ">R", FLAG_PRIMITIVE, prim_to_r, 0);
    add_word(vm, "R>", FLAG_PRIMITIVE, prim_r_from, 0);
    add_word(vm, "R@", FLAG_PRIMITIVE, prim_r_fetch, 0);
    add_word(vm, "2>R", FLAG_PRIMITIVE, prim_2to_r, 0);
    add_word(vm, "2R>", FLAG_PRIMITIVE, prim_2r_from, 0);
    add_word(vm, "2R@", FLAG_PRIMITIVE, prim_2r_fetch, 0);
    add_word(vm, "I", FLAG_PRIMITIVE, prim_i, 0);
    add_word(vm, "J", FLAG_PRIMITIVE, prim_j, 0);
    add_word(vm, "K", FLAG_PRIMITIVE, prim_k, 0);
    add_word(vm, "UNLOOP", FLAG_PRIMITIVE, prim_unloop, 0);

    /* Arithmetic / Bitwise */
    add_word(vm, "+", FLAG_PRIMITIVE, prim_add, 0);
    add_word(vm, "-", FLAG_PRIMITIVE, prim_sub, 0);
    add_word(vm, "*", FLAG_PRIMITIVE, prim_mul, 0);
    add_word(vm, "/", FLAG_PRIMITIVE, prim_div, 0);
    add_word(vm, "MOD", FLAG_PRIMITIVE, prim_mod, 0);
    add_word(vm, "/MOD", FLAG_PRIMITIVE, prim_divmod, 0);
    add_word(vm, "*/", FLAG_PRIMITIVE, prim_star_slash, 0);
    add_word(vm, "*/MOD", FLAG_PRIMITIVE, prim_star_slash_mod, 0);
    add_word(vm, "M*", FLAG_PRIMITIVE, prim_mstar, 0);
    add_word(vm, "SM/REM", FLAG_PRIMITIVE, prim_sm_rem, 0);
    add_word(vm, "UM/MOD", FLAG_PRIMITIVE, prim_um_mod, 0);
    add_word(vm, "1+", FLAG_PRIMITIVE, prim_1plus, 0);
    add_word(vm, "1-", FLAG_PRIMITIVE, prim_1minus, 0);
    add_word(vm, "2+", FLAG_PRIMITIVE, prim_2plus, 0);
    add_word(vm, "2-", FLAG_PRIMITIVE, prim_2minus, 0);
    add_word(vm, "2*", FLAG_PRIMITIVE, prim_2mul, 0);
    add_word(vm, "2/", FLAG_PRIMITIVE, prim_2div, 0);
    add_word(vm, "NEGATE", FLAG_PRIMITIVE, prim_negate, 0);
    add_word(vm, "ABS", FLAG_PRIMITIVE, prim_abs, 0);
    add_word(vm, "MIN", FLAG_PRIMITIVE, prim_min, 0);
    add_word(vm, "MAX", FLAG_PRIMITIVE, prim_max, 0);
    add_word(vm, "AND", FLAG_PRIMITIVE, prim_and, 0);
    add_word(vm, "OR", FLAG_PRIMITIVE, prim_or, 0);
    add_word(vm, "XOR", FLAG_PRIMITIVE, prim_xor, 0);
    add_word(vm, "INVERT", FLAG_PRIMITIVE, prim_invert, 0);
    add_word(vm, "LSHIFT", FLAG_PRIMITIVE, prim_lshift, 0);
    add_word(vm, "RSHIFT", FLAG_PRIMITIVE, prim_rshift, 0);
    add_word(vm, "ARSHIFT", FLAG_PRIMITIVE, prim_arshift, 0);

    /* Comparisons */
    add_word(vm, "=", FLAG_PRIMITIVE, prim_eq, 0);
    add_word(vm, "<>", FLAG_PRIMITIVE, prim_neq, 0);
    add_word(vm, "<", FLAG_PRIMITIVE, prim_lt, 0);
    add_word(vm, ">", FLAG_PRIMITIVE, prim_gt, 0);
    add_word(vm, "<=", FLAG_PRIMITIVE, prim_lte, 0);
    add_word(vm, ">=", FLAG_PRIMITIVE, prim_gte, 0);
    add_word(vm, "0=", FLAG_PRIMITIVE, prim_0eq, 0);
    add_word(vm, "0<>", FLAG_PRIMITIVE, prim_0neq, 0);
    add_word(vm, "0<", FLAG_PRIMITIVE, prim_0lt, 0);
    add_word(vm, "0>", FLAG_PRIMITIVE, prim_0gt, 0);
    add_word(vm, "0<=", FLAG_PRIMITIVE, prim_0lte, 0);
    add_word(vm, "0>=", FLAG_PRIMITIVE, prim_0gte, 0);
    add_word(vm, "U<", FLAG_PRIMITIVE, prim_ult, 0);
    add_word(vm, "U>", FLAG_PRIMITIVE, prim_ugt, 0);

    /* Memory */
    add_word(vm, "@", FLAG_PRIMITIVE, prim_fetch, 0);
    add_word(vm, "!", FLAG_PRIMITIVE, prim_store, 0);
    add_word(vm, "C@", FLAG_PRIMITIVE, prim_cfetch, 0);
    add_word(vm, "C!", FLAG_PRIMITIVE, prim_cstore, 0);
    add_word(vm, "W@", FLAG_PRIMITIVE, prim_wfetch, 0);
    add_word(vm, "W!", FLAG_PRIMITIVE, prim_wstore, 0);
    add_word(vm, "2@", FLAG_PRIMITIVE, prim_2fetch, 0);
    add_word(vm, "2!", FLAG_PRIMITIVE, prim_2store, 0);
    add_word(vm, "+!", FLAG_PRIMITIVE, prim_plus_store, 0);
    add_word(vm, "MOVE", FLAG_PRIMITIVE, prim_move, 0);
    add_word(vm, "CMOVE", FLAG_PRIMITIVE, prim_cmove, 0);
    add_word(vm, "FILL", FLAG_PRIMITIVE, prim_fill, 0);
    add_word(vm, "ERASE", FLAG_PRIMITIVE, prim_erase, 0);

    /* Allocation */
    add_word(vm, "HERE", FLAG_PRIMITIVE, prim_here, 0);
    add_word(vm, "ALLOT", FLAG_PRIMITIVE, prim_allot, 0);
    add_word(vm, ",", FLAG_PRIMITIVE, prim_comma, 0);
    add_word(vm, "C,", FLAG_PRIMITIVE, prim_ccomma, 0);
    add_word(vm, "ALIGN", FLAG_PRIMITIVE, prim_align, 0);
    add_word(vm, "ALIGNED", FLAG_PRIMITIVE, prim_aligned, 0);
    add_word(vm, "CELL+", FLAG_PRIMITIVE, prim_cell_plus, 0);
    add_word(vm, "CELLS", FLAG_PRIMITIVE, prim_cells, 0);

    /* Double */
    add_word(vm, "D+", FLAG_PRIMITIVE, prim_dplus, 0);
    add_word(vm, "D-", FLAG_PRIMITIVE, prim_dminus, 0);
    add_word(vm, "DNEGATE", FLAG_PRIMITIVE, prim_dnegate, 0);
    add_word(vm, "M*", FLAG_PRIMITIVE, prim_mstar, 0);
    add_word(vm, "UM*", FLAG_PRIMITIVE, prim_umstar, 0);
    add_word(vm, "Q32*", FLAG_PRIMITIVE, prim_q32_mul, 0);
    add_word(vm, "Q32/", FLAG_PRIMITIVE, prim_q32_div, 0);

    /* Float */
    add_word(vm, "F+", FLAG_PRIMITIVE, prim_fadd, 0);
    add_word(vm, "F-", FLAG_PRIMITIVE, prim_fsub, 0);
    add_word(vm, "F*", FLAG_PRIMITIVE, prim_fmul, 0);
    add_word(vm, "F/", FLAG_PRIMITIVE, prim_fdiv, 0);
    add_word(vm, "FNEGATE", FLAG_PRIMITIVE, prim_fneg, 0);
    add_word(vm, "FABS", FLAG_PRIMITIVE, prim_fabs, 0);
    add_word(vm, "F=", FLAG_PRIMITIVE, prim_feq, 0);
    add_word(vm, "F<", FLAG_PRIMITIVE, prim_flt, 0);
    add_word(vm, "F>", FLAG_PRIMITIVE, prim_fgt, 0);
    add_word(vm, "FDUP", FLAG_PRIMITIVE, prim_fdup, 0);
    add_word(vm, "FDROP", FLAG_PRIMITIVE, prim_fdrop, 0);
    add_word(vm, "FSWAP", FLAG_PRIMITIVE, prim_fswap, 0);
    add_word(vm, "FOVER", FLAG_PRIMITIVE, prim_fover, 0);
    add_word(vm, "FROT", FLAG_PRIMITIVE, prim_frot, 0);
    add_word(vm, "FSIN", FLAG_PRIMITIVE, prim_fsin, 0);
    add_word(vm, "FCOS", FLAG_PRIMITIVE, prim_fcos, 0);
    add_word(vm, "FTAN", FLAG_PRIMITIVE, prim_ftan, 0);
    add_word(vm, "FATAN2", FLAG_PRIMITIVE, prim_fatan2, 0);
    add_word(vm, "FSQRT", FLAG_PRIMITIVE, prim_fsqrt, 0);
    add_word(vm, "FEXP", FLAG_PRIMITIVE, prim_fexp, 0);
    add_word(vm, "FLN", FLAG_PRIMITIVE, prim_fln, 0);
    add_word(vm, "F**", FLAG_PRIMITIVE, prim_fpow, 0);
    add_word(vm, "S>F", FLAG_PRIMITIVE, prim_stof, 0);
    add_word(vm, "F>S", FLAG_PRIMITIVE, prim_ftos, 0);
    add_word(vm, "F@", FLAG_PRIMITIVE, prim_ffetch, 0);
    add_word(vm, "F!", FLAG_PRIMITIVE, prim_fstore, 0);
    add_word(vm, "F.", FLAG_PRIMITIVE, prim_fdot, 0);

    /* Output */
    add_word(vm, ".", FLAG_PRIMITIVE, prim_dot, 0);
    add_word(vm, "U.", FLAG_PRIMITIVE, prim_udot, 0);
    add_word(vm, ".S", FLAG_PRIMITIVE, prim_dots, 0);
    add_word(vm, "CR", FLAG_PRIMITIVE, prim_cr, 0);
    add_word(vm, "SPACE", FLAG_PRIMITIVE, prim_space, 0);
    add_word(vm, "SPACES", FLAG_PRIMITIVE, prim_spaces, 0);
    add_word(vm, "EMIT", FLAG_PRIMITIVE, prim_emit, 0);
    add_word(vm, "TYPE", FLAG_PRIMITIVE, prim_type, 0);
    add_word(vm, "HEX", FLAG_PRIMITIVE, prim_hex, 0);
    add_word(vm, "DECIMAL", FLAG_PRIMITIVE, prim_decimal, 0);
    add_word(vm, "BINARY", FLAG_PRIMITIVE, prim_binary, 0);
    add_word(vm, "BASE", FLAG_PRIMITIVE, prim_base_addr, 0);
    add_word(vm, "BASE@", FLAG_PRIMITIVE, prim_base_fetch, 0);
    add_word(vm, "BASE!", FLAG_PRIMITIVE, prim_base_store, 0);
    add_word(vm, "BYE", FLAG_PRIMITIVE, prim_bye, 0);

    /* Time */
    add_word(vm, "MS", FLAG_PRIMITIVE, prim_ms, 0);
    add_word(vm, "UTIME", FLAG_PRIMITIVE, prim_utime, 0);

    /* Hardware Bus Primitives */
    add_word(vm, "HW@", FLAG_PRIMITIVE, prim_hw_fetch, 0);
    add_word(vm, "HW!", FLAG_PRIMITIVE, prim_hw_store, 0);
    add_word(vm, "HW-STEP", FLAG_PRIMITIVE, prim_hw_step, 0);
    add_word(vm, "HW-CLOCK", FLAG_PRIMITIVE, prim_hw_clock, 0);
    add_word(vm, "HW-LOG", FLAG_PRIMITIVE, prim_hw_log, 0);

    /* Immediate Control Flow */
    add_word(vm, ";", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_semicolon, 0);
    add_word(vm, "IMMEDIATE", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_immediate, 0);
    add_word(vm, "IF", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_if, 0);
    add_word(vm, "ELSE", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_else, 0);
    add_word(vm, "THEN", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_then, 0);
    add_word(vm, "BEGIN", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_begin, 0);
    add_word(vm, "UNTIL", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_until, 0);
    add_word(vm, "AGAIN", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_again, 0);
    add_word(vm, "WHILE", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_while, 0);
    add_word(vm, "REPEAT", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_repeat, 0);
    add_word(vm, "DO", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_do, 0);
    add_word(vm, "?DO", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_do, 0);
    add_word(vm, "LOOP", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_loop, 0);
    add_word(vm, "+LOOP", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_plus_loop, 0);
    add_word(vm, "RECURSE", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_recurse, 0);
    add_word(vm, "EXIT", FLAG_PRIMITIVE, prim_exit, 0);
    add_word(vm, "EXECUTE", FLAG_PRIMITIVE, prim_execute, 0);
    add_word(vm, "CASE", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_case, 0);
    add_word(vm, "OF", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_of, 0);
    add_word(vm, "ENDOF", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_endof, 0);
    add_word(vm, "ENDCASE", FLAG_PRIMITIVE | FLAG_IMMEDIATE, prim_endcase, 0);
}

/* Evaluator */
bool trunnion_vm_eval_string(trunnion_vm_t *vm, const char *str) {
    const char *cursor = str;
    char token[TRUNNION_VM_WORD_NAME_MAX];

    while (next_token(&cursor, token, sizeof(token))) {
        /* Line comments */
        if (strcmp(token, "\\") == 0) {
            while (*cursor && *cursor != '\n') cursor++;
            continue;
        }

        /* Parenthesized comments */
        if (strcmp(token, "(") == 0) {
            while (*cursor && *cursor != ')') cursor++;
            if (*cursor == ')') cursor++;
            continue;
        }

        /* Block comments */
        if (strcmp(token, "(*") == 0) {
            while (*cursor && !(*cursor == '*' && *(cursor + 1) == ')')) cursor++;
            if (*cursor) cursor += 2;
            continue;
        }

        /* Conditional compilation skip mode */
        if (vm->cond_skip_depth > 0) {
            if (strcmp(token, "[IF]") == 0) {
                vm->cond_skip_depth++;
            } else if (strcmp(token, "[THEN]") == 0) {
                vm->cond_skip_depth--;
            } else if (strcmp(token, "[ELSE]") == 0 && vm->cond_skip_depth == 1) {
                vm->cond_skip_depth = 0;
            }
            continue;
        }

        /* [DEFINED] name */
        if (strcmp(token, "[DEFINED]") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                trunnion_vm_push(vm, find_word(vm, token) >= 0 ? -1 : 0);
            }
            continue;
        }

        /* [UNDEFINED] name */
        if (strcmp(token, "[UNDEFINED]") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                trunnion_vm_push(vm, find_word(vm, token) < 0 ? -1 : 0);
            }
            continue;
        }

        /* [IF] */
        if (strcmp(token, "[IF]") == 0) {
            cell_t flag = trunnion_vm_pop(vm);
            if (!flag) {
                vm->cond_skip_depth = 1;
            }
            continue;
        }

        /* [ELSE] */
        if (strcmp(token, "[ELSE]") == 0) {
            vm->cond_skip_depth = 1;
            continue;
        }

        /* [THEN] */
        if (strcmp(token, "[THEN]") == 0) {
            continue;
        }

        /* Colon definition */
        if (strcmp(token, ":") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                /* Align here */
                vm->here = (vm->here + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                add_word(vm, token, 0, NULL, vm->here);
                vm->compiling = true;
            }
            continue;
        }

        /* CREATE word */
        if (strcmp(token, "CREATE") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                vm->here = (vm->here + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                cell_t data_addr = vm->here + 3 * sizeof(cell_t);
                cell_t xt = vm->here;
                compile_cell(vm, OP_LIT);
                compile_cell(vm, data_addr);
                compile_cell(vm, OP_EXIT);
                add_word(vm, token, 0, NULL, xt);
            }
            continue;
        }

        /* Variable definition */
        if (strcmp(token, "VARIABLE") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                vm->here = (vm->here + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                cell_t var_addr = vm->here;
                *(cell_t *)(vm->memory + var_addr) = 0;
                vm->here += sizeof(cell_t);

                /* Create a word that compiles or pushes var_addr */
                cell_t xt = vm->here;
                compile_cell(vm, OP_LIT);
                compile_cell(vm, var_addr);
                compile_cell(vm, OP_EXIT);
                add_word(vm, token, 0, NULL, xt);
            }
            continue;
        }

        /* Constant definition */
        if (strcmp(token, "CONSTANT") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                cell_t val = trunnion_vm_pop(vm);
                vm->here = (vm->here + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                cell_t xt = vm->here;
                compile_cell(vm, OP_LIT);
                compile_cell(vm, val);
                compile_cell(vm, OP_EXIT);
                add_word(vm, token, 0, NULL, xt);
            }
            continue;
        }

        /* Value definition */
        if (strcmp(token, "VALUE") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                cell_t val = trunnion_vm_pop(vm);
                vm->here = (vm->here + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                cell_t val_addr = vm->here;
                *(cell_t *)(vm->memory + val_addr) = val;
                vm->here += sizeof(cell_t);

                /* Word pushes the value stored at val_addr */
                cell_t xt = vm->here;
                compile_cell(vm, OP_LIT);
                compile_cell(vm, val_addr);
                compile_cell(vm, find_word(vm, "@"));
                compile_cell(vm, OP_EXIT);
                int idx = add_word(vm, token, 0, NULL, xt);
                (void)idx;
            }
            continue;
        }

        /* TO word */
        if (strcmp(token, "TO") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                int idx = find_word(vm, token);
                if (idx >= 0) {
                    /* Address is the literal compiled in the first word instruction */
                    cell_t xt = vm->dictionary[idx].xt;
                    cell_t val_addr = *(cell_t *)(vm->memory + xt + sizeof(cell_t));
                    if (vm->compiling) {
                        compile_cell(vm, OP_LIT);
                        compile_cell(vm, val_addr);
                        compile_cell(vm, find_word(vm, "!"));
                    } else {
                        cell_t val = trunnion_vm_pop(vm);
                        *(cell_t *)(vm->memory + val_addr) = val;
                    }
                }
            }
            continue;
        }

        /* CHAR / [CHAR] character literal */
        if (strcmp(token, "[CHAR]") == 0 || strcmp(token, "CHAR") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                cell_t ch = (uint8_t)token[0];
                if (vm->compiling) {
                    compile_cell(vm, OP_LIT);
                    compile_cell(vm, ch);
                } else {
                    trunnion_vm_push(vm, ch);
                }
            }
            continue;
        }

        /* Tick ' and ['] */
        if (strcmp(token, "'") == 0 || strcmp(token, "[']") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                int idx = find_word(vm, token);
                if (idx >= 0) {
                    if (vm->compiling) {
                        compile_cell(vm, OP_LIT);
                        compile_cell(vm, (cell_t)idx);
                    } else {
                        trunnion_vm_push(vm, (cell_t)idx);
                    }
                } else {
                    fprintf(stderr, "[ERROR] Tick could not find word '%s'\n", token);
                }
            }
            continue;
        }

        /* S" String Literal */
        if (strcmp(token, "S\"") == 0) {
            /* Collect up to terminating double quote */
            while (*cursor && isspace((unsigned char)*cursor)) cursor++;
            const char *str_start = cursor;
            while (*cursor && *cursor != '"') cursor++;
            size_t str_len = cursor - str_start;
            if (*cursor == '"') cursor++;

            if (vm->compiling) {
                compile_cell(vm, OP_STR_LIT);
                compile_cell(vm, (cell_t)str_len);
                cell_t str_dest = vm->here;
                memcpy(vm->memory + str_dest, str_start, str_len);
                vm->here = (vm->here + str_len + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
            } else {
                /* Copy string into PAD */
                memcpy(vm->pad, str_start, str_len);
                vm->pad[str_len] = '\0';
                trunnion_vm_push(vm, (cell_t)(uintptr_t)vm->pad);
                trunnion_vm_push(vm, (cell_t)str_len);
            }
            continue;
        }

        /* ." Print String Literal */
        if (strcmp(token, ".\"") == 0) {
            while (*cursor && isspace((unsigned char)*cursor)) cursor++;
            const char *str_start = cursor;
            while (*cursor && *cursor != '"') cursor++;
            size_t str_len = cursor - str_start;
            if (*cursor == '"') cursor++;

            if (vm->compiling) {
                compile_cell(vm, OP_STR_LIT);
                compile_cell(vm, (cell_t)str_len);
                cell_t str_dest = vm->here;
                memcpy(vm->memory + str_dest, str_start, str_len);
                vm->here = (vm->here + str_len + (sizeof(cell_t) - 1)) & ~(sizeof(cell_t) - 1);
                compile_cell(vm, find_word(vm, "TYPE"));
            } else {
                for (size_t i = 0; i < str_len; i++) putchar(str_start[i]);
                fflush(stdout);
            }
            continue;
        }

        /* .( Print String Literal in Immediate / Interpretation Mode */
        if (strcmp(token, ".(") == 0) {
            while (*cursor && isspace((unsigned char)*cursor)) cursor++;
            const char *str_start = cursor;
            while (*cursor && *cursor != ')') cursor++;
            size_t str_len = cursor - str_start;
            if (*cursor == ')') cursor++;
            for (size_t i = 0; i < str_len; i++) putchar(str_start[i]);
            fflush(stdout);
            continue;
        }

        /* INCLUDE word */
        if (strcmp(token, "INCLUDE") == 0) {
            if (next_token(&cursor, token, sizeof(token))) {
                if (!trunnion_vm_load_file(vm, token)) {
                    fprintf(stderr, "[ERROR] Could not include file '%s'\n", token);
                }
            }
            continue;
        }

        /* Find token in dictionary */
        int idx = find_word(vm, token);
        if (idx >= 0) {
            word_entry_t *w = &vm->dictionary[idx];
            if (vm->compiling && !(w->flags & FLAG_IMMEDIATE)) {
                compile_cell(vm, idx);
            } else {
                execute_xt(vm, idx);
            }
            continue;
        }

        /* Try parsing as integer */
        cell_t int_val;
        if (parse_number(token, vm->base, &int_val)) {
            if (vm->compiling) {
                compile_cell(vm, OP_LIT);
                compile_cell(vm, int_val);
            } else {
                trunnion_vm_push(vm, int_val);
            }
            continue;
        }

        /* Try parsing as float */
        double flt_val;
        if (parse_float(token, &flt_val)) {
            if (vm->compiling) {
                compile_cell(vm, OP_FLIT);
                if (vm->here + (cell_t)sizeof(double) <= TRUNNION_VM_MEMORY_SIZE) {
                    *(double *)(vm->memory + vm->here) = flt_val;
                    vm->here += sizeof(double);
                }
            } else {
                trunnion_vm_fpush(vm, flt_val);
            }
            continue;
        }

        fprintf(stderr, "[ERROR] Unrecognized word: '%s' at %s:%d\n", token, vm->current_file, vm->current_line);
    }

    return true;
}

bool trunnion_vm_load_file(trunnion_vm_t *vm, const char *filepath) {
    FILE *fp = fopen(filepath, "r");
    if (!fp) return false;

    char prev_file[256];
    int prev_line = vm->current_line;
    strncpy(prev_file, vm->current_file, sizeof(prev_file) - 1);
    prev_file[sizeof(prev_file) - 1] = '\0';

    strncpy(vm->current_file, filepath, sizeof(vm->current_file) - 1);
    vm->current_file[sizeof(vm->current_file) - 1] = '\0';
    vm->current_line = 0;

    char line_buf[4096];
    while (fgets(line_buf, sizeof(line_buf), fp)) {
        vm->current_line++;
        trunnion_vm_eval_string(vm, line_buf);
    }

    fclose(fp);
    strncpy(vm->current_file, prev_file, sizeof(vm->current_file) - 1);
    vm->current_line = prev_line;
    return true;
}

void trunnion_vm_run_repl(trunnion_vm_t *vm) {
    char line[1024];
    printf("====================================================================\n");
    printf(" TRUNNION: Satellite Laser Crosslink Terminal Flight Executive (Forth)\n");
    printf(" Copyright (c) 2026 Aneesh Krishna Parthasarathy\n");
    printf(" Type 'BYE' to exit or '.S' to inspect data stack.\n");
    printf("====================================================================\n\n");

    while (vm->running) {
        printf(vm->compiling ? "  compiled> " : "  ok> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;
        trunnion_vm_eval_string(vm, line);
    }
}
