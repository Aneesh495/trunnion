/*
 * TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
 * Main Executable Entry Point
 *
 * Copyright (c) 2026 Aneesh Krishna Parthasarathy
 * Licensed under the ISC License.
 */

#include "trunnion_vm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    trunnion_vm_t *vm = trunnion_vm_create();
    if (!vm) {
        fprintf(stderr, "[FATAL] Failed to allocate Trunnion Forth Virtual Machine.\n");
        return 1;
    }

    /* Process command line arguments */
    bool interactive = false;
    int files_loaded = 0;
    int expr_count = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--interactive") == 0) {
            interactive = true;
        } else if (strcmp(argv[i], "-e") == 0 && i + 1 < argc) {
            i++;
            trunnion_vm_eval_string(vm, argv[i]);
            expr_count++;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options] [forth_files...]\n", argv[0]);
            printf("Options:\n");
            printf("  -i, --interactive   Launch interactive Forth REPL after loading files\n");
            printf("  -e <code_string>    Evaluate Forth expression string\n");
            printf("  -h, --help          Show this help message\n");
            trunnion_vm_destroy(vm);
            return 0;
        } else {
            /* Treat as file path */
            if (!trunnion_vm_load_file(vm, argv[i])) {
                fprintf(stderr, "[ERROR] Failed to load Forth script: %s\n", argv[i]);
                trunnion_vm_destroy(vm);
                return 1;
            }
            files_loaded++;
        }
    }

    if (interactive || (files_loaded == 0 && expr_count == 0)) {
        trunnion_vm_run_repl(vm);
    }

    trunnion_vm_destroy(vm);
    return 0;
}
