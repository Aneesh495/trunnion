/*
 * TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
 * Virtual Machine & Hardware Simulation Runtime Header
 *
 * Copyright (c) 2026 Aneesh Krishna Parthasarathy
 * Licensed under the ISC License.
 */

#ifndef TRUNNION_VM_H
#define TRUNNION_VM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define TRUNNION_VM_DATA_STACK_SIZE   1024
#define TRUNNION_VM_RETURN_STACK_SIZE 1024
#define TRUNNION_VM_FLOAT_STACK_SIZE  128
#define TRUNNION_VM_MEMORY_SIZE       (16 * 1024 * 1024)
#define TRUNNION_VM_WORD_NAME_MAX     64
#define TRUNNION_VM_MAX_WORDS         8192
#define TRUNNION_VM_PAD_SIZE          1024
#define TRUNNION_VM_HW_REGS_SIZE      4096

typedef int64_t  cell_t;
typedef uint64_t ucell_t;
typedef double   vm_float_t;

typedef enum {
    FLAG_NONE      = 0x00,
    FLAG_IMMEDIATE = 0x01,
    FLAG_HIDDEN    = 0x02,
    FLAG_PRIMITIVE = 0x04
} word_flags_t;

struct trunnion_vm;
typedef void (*primitive_fn)(struct trunnion_vm *vm);

typedef struct {
    char name[TRUNNION_VM_WORD_NAME_MAX];
    uint8_t flags;
    primitive_fn prim;
    cell_t xt;
    cell_t does_ip;
} word_entry_t;

typedef struct trunnion_vm {
    /* Data Stack */
    cell_t data_stack[TRUNNION_VM_DATA_STACK_SIZE];
    int sp;

    /* Return Stack */
    cell_t return_stack[TRUNNION_VM_RETURN_STACK_SIZE];
    int rp;

    /* Floating Point Stack */
    double float_stack[TRUNNION_VM_FLOAT_STACK_SIZE];
    int fp;

    /* Linear Memory Space */
    uint8_t *memory;
    cell_t here;
    cell_t latest_word;

    /* Dictionary */
    word_entry_t dictionary[TRUNNION_VM_MAX_WORDS];
    int num_words;

    /* Execution State */
    cell_t ip;
    bool compiling;
    bool running;
    int base;
    char pad[TRUNNION_VM_PAD_SIZE];
    int cond_skip_depth;

    /* Control Flow Stack for Compiling Control Structures */
    cell_t cs_stack[64];
    int cs_sp;

    /* Hardware Simulation Space */
    uint32_t hw_regs[TRUNNION_VM_HW_REGS_SIZE];
    uint64_t simulated_time_us;
    uint64_t tick_counter;
    bool laser_enabled;
    double beam_azimuth_rad;
    double beam_elevation_rad;
    double target_azimuth_rad;
    double target_elevation_rad;

    /* I/O and File State */
    char current_file[256];
    int current_line;
} trunnion_vm_t;

/* Public API */
trunnion_vm_t *trunnion_vm_create(void);
void trunnion_vm_destroy(trunnion_vm_t *vm);
void trunnion_vm_reset(trunnion_vm_t *vm);
bool trunnion_vm_load_file(trunnion_vm_t *vm, const char *filepath);
bool trunnion_vm_eval_string(trunnion_vm_t *vm, const char *str);
void trunnion_vm_run_repl(trunnion_vm_t *vm);

/* Stack primitives */
void trunnion_vm_push(trunnion_vm_t *vm, cell_t val);
cell_t trunnion_vm_pop(trunnion_vm_t *vm);
cell_t trunnion_vm_peek(trunnion_vm_t *vm);

void trunnion_vm_rpush(trunnion_vm_t *vm, cell_t val);
cell_t trunnion_vm_rpop(trunnion_vm_t *vm);

void trunnion_vm_fpush(trunnion_vm_t *vm, double val);
double trunnion_vm_fpop(trunnion_vm_t *vm);

/* Hardware bus primitives */
uint32_t trunnion_hw_read32(trunnion_vm_t *vm, uint32_t reg_offset);
void trunnion_hw_write32(trunnion_vm_t *vm, uint32_t reg_offset, uint32_t value);
void trunnion_hw_step(trunnion_vm_t *vm, uint64_t delta_us);

#endif /* TRUNNION_VM_H */
