# TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
# Root Build Automation Makefile
# Copyright (c) 2026 Aneesh Krishna Parthasarathy
# Licensed under the ISC License.

CC ?= clang
CFLAGS ?= -O3 -Wall -Wextra -Wno-unused-parameter -std=c11 -Iinclude
LDFLAGS ?= -lm

BIN_DIR = bin
BUILD_DIR = build
TARGET = $(BIN_DIR)/trunnion

VM_SRCS = src/vm/trunnion_vm.c src/vm/main.c
VM_OBJS = $(BUILD_DIR)/trunnion_vm.o $(BUILD_DIR)/main.o

.PHONY: all clean test check lines repl sim bench

all: $(TARGET)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/trunnion_vm.o: src/vm/trunnion_vm.c include/trunnion_vm.h | $(BUILD_DIR)
	@echo "[Compiling Forth VM core: $<]"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/main.o: src/vm/main.c include/trunnion_vm.h | $(BUILD_DIR)
	@echo "[Compiling Trunnion runtime entry: $<]"
	@$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(VM_OBJS) | $(BIN_DIR)
	@echo "[Linking Trunnion binary: $(TARGET)]"
	@$(CC) $(VM_OBJS) $(LDFLAGS) -o $(TARGET)
	@echo "[Build successful: $(TARGET)]"

check:
	@echo "[Verifying dash compliance...]"
	@python3 tools/check_no_em_dash.py

lines:
	@python3 tools/line_count.py

test: $(TARGET) check
	@echo "[Executing comprehensive Trunnion test runner...]"
	@$(TARGET) trunnion.fs tests/test_all.fs

test-vm: test

repl: $(TARGET)
	@$(TARGET) -i trunnion.fs

sim: $(TARGET)
	@echo "[Starting Hardware-in-the-Loop simulation bridge...]"
	@python3 sim/hil_bridge.py

bench: $(TARGET)
	@echo "[Running real-time jitter and control loop benchmarks...]"
	@$(TARGET) trunnion.fs tests/benchmarks.fs

clean:
	@echo "[Cleaning build artifacts...]"
	@rm -rf $(BIN_DIR) $(BUILD_DIR) *.o *.log
	@echo "[Clean complete.]"
