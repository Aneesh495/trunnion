# Getting Started & Operator Manual

This guide covers building the Trunnion flight software, launching the interactive operator shell, executing automated verification suites, running the multi-physics Hardware-in-the-Loop (HIL) simulation, and monitoring real-time avionics telemetry.

---

## 1. Prerequisites & Toolchain

The Trunnion codebase requires zero third-party C library dependencies. A standard ANSI C11 compiler and GNU Make are sufficient to build the complete flight software virtual machine and simulated hardware runtime.

- **Host Compiler**: GCC (>= 8.0) or Clang / Apple LLVM (>= 11.0).
- **Build System**: GNU Make (>= 3.81).
- **Dual Runtime (Optional)**: GNU Forth (`gforth` >= 0.7.3) for independent Forth-2012 dual-target cross-validation.
- **Simulation & Telemetry Tools (Optional)**: Python 3.8+ with standard library (no external packages required).

---

## 2. Building the Flight Software

Clone the repository and compile the native virtual machine:

```bash
# Clean previous artifacts and build the host runtime
make clean
make all
```

The build produces the high-performance ANSI C11 virtual machine binary at `bin/trunnion`.

---

## 3. Interactive Flight Software Shell

Launch the interactive operator console and load the master bootloader:

```bash
./bin/trunnion -i trunnion.fs
```

Upon boot, the system outputs the Trunnion avionics banner:

```text
======================================================================
 TRUNNION :: SATELLITE LASER CROSS-LINK FLIGHT SOFTWARE
 Master Pointing, Acquisition & Tracking (PAT) Engine
 CCSDS Optical Communications Subsystem
 System Architecture: 64-Bit Forth-2012 Engine
======================================================================
[BOOT] Initializing core fixed-point math and CORDIC engine ... [OK]
[BOOT] Mapping MMIO hardware registers and peripherals ........ [OK]
[BOOT] Loading dual-axis gimbal kinematics and LuGre model .... [OK]
[BOOT] Initializing optical centroiding and nutation tracker .. [OK]
[BOOT] Arming master PAT state machine ........................ [OK]
[BOOT] Starting orbital propagator and line-of-sight solver ... [OK]
[BOOT] Initializing CCSDS M-PPM modem and RS(255,223) codec ... [OK]
[BOOT] Arming avionics watchdog and thermal safety supervisor . [OK]
[BOOT] Spawning cooperative real-time scheduler tasks ......... [OK]
[READY] Flight software initialized. Type HELP for commands.
```

### 3.1 Essential Operator Commands

The operator dictionary in `forth/shell/commands.fs` provides high-level commands for manual inspection, subsystem commanding, and diagnostics:

| Command | Stack Effect | Description |
|:--------|:------------:|:------------|
| `HELP` | `( -- )` | Display operator command dictionary and help text |
| `STATUS` | `( -- )` | Print formatted real-time engineering telemetry table |
| `DUMP-ALL` | `( -- )` | Dump raw MMIO register contents across all peripherals |
| `PAT-STANDBY` | `( -- )` | Transition PAT state machine to STANDBY |
| `PAT-SLEW` | `( -- )` | Transition PAT state machine to SLEW mode |
| `PAT-SPIRAL` | `( -- )` | Initiate spiral acquisition scan |
| `PAT-TRACK` | `( -- )` | Transition to dual-stage closed-loop tracking |
| `LASER-ON` | `( -- )` | Close safety interlock and enable laser emission |
| `LASER-OFF` | `( -- )` | Inhibit laser emission and open safety relay |
| `ESTOP` | `( -- )` | Emergency stop: disable all motors, lasers, and piezos |
| `STEP-FRAME` | `( -- )` | Execute one manual 1 kHz real-time executive frame |
| `RUN-FRAMES` | `( n -- )` | Execute $n$ consecutive 1 kHz executive frames |

### 3.2 Real-Time Engineering Status Display

Execute `STATUS` in the REPL to display live subsystem telemetry:

```text
trunnion> STATUS
======================================================================
 TRUNNION AVIONICS REAL-TIME ENGINEERING STATUS
======================================================================
 PAT State Machine : FINE_TRACK (0x05)
 CPA Gimbal Azimuth: +024.512 deg  |  Elevation: +012.348 deg
 CPA Velocity      : +000.012 deg/s|  Elevation: -000.004 deg/s
 FSM Deflection Tip: +0042.8 urad  |  Tilt     : -0018.3 urad
 Optical Receiver  : Energy=14280 counts | SNR=+28.4 dB
 Laser Subsystem   : Power=250 mW  | Emission=ACTIVE | Interlock=CLOSED
 Thermal Telemetry : Bench=+21.45 C| Laser Diode=+24.98 C
 System Executive  : Ticks=482910  | Watchdog=ARMED (Healthy)
======================================================================
```

---

## 4. Automated Verification & Benchmarks

### 4.1 Running the Verification Suites

Trunnion includes 10 comprehensive test suites spanning 128 top-level assertions and 338 individual functional checks:

```bash
# Execute all test suites against custom C11 Forth Virtual Machine
make test-vm

# Execute all test suites against GNU Forth (dual runtime verification)
make test-gforth

# Run both targets in sequence
make test
```

### 4.2 Running High-Rate Performance Benchmarks

Benchmark raw fixed-point arithmetic, CORDIC rotations, matrix transforms, and real-time executive margin over 1,000,000 iterations:

```bash
make bench
```

---

## 5. Hardware-in-the-Loop (HIL) Simulation

The repository provides a multi-physics orbital dynamics, optical link loss, and spacecraft micro-vibration simulation environment (`sim/`):

```bash
# Launch closed-loop HIL simulation
make sim
```

The simulation runner connects to the flight software via bidirectional IPC socket streams, injecting orbit ephemeris, calculating geometric line-of-sight vectors, simulating optical signal attenuation, adding reaction-wheel angular jitter, and verifying closed-loop acquisition and tracking lock.

---

## 6. Live Telemetry Terminal Dashboard

An ANSI terminal dashboard (`tools/telemetry_dashboard.py`) provides live visualization of gimbal pointing, FSM deflection, optical receiver energy, thermal loops, and packet throughput:

```bash
python3 tools/telemetry_dashboard.py
```

Press `Ctrl+C` or `q` to exit the dashboard.

---

## 7. Codebase Layout

```
trunnion/
  ├── bin/                 # Compiled native host binaries (trunnion)
  ├── docs/                # Comprehensive engineering and mathematical specifications
  │   ├── architecture_diagram.md
  │   ├── avionics_arch.md
  │   ├── ccsds_optcomm.md
  │   ├── cpa_fsm_dynamics.md
  │   ├── getting_started.md
  │   ├── memory_map.md
  │   ├── pat_theory.md
  │   └── verification.md
  ├── forth/               # Flight software source tree (30k+ lines of pure Forth)
  │   ├── core/            # Fixed-point math, 128-bit scaling, CORDIC, vectors, quaternions
  │   ├── hal/             # Peripheral register definitions, MMIO drivers, and interrupts
  │   ├── kinematics/      # Dual-axis gimbal solvers, S-curve trajectories, LuGre friction
  │   ├── optics/          # QPD centroiding, NFOV CoG, nutation demodulation, SNR estimation
  │   ├── pat/             # Spiral search, Lissajous scan, acquisition, point-ahead angle
  │   ├── orbit/           # Keplerian propagator, J2 oblateness, LOS geometry, occultation
  │   ├── comm/            # M-PPM modem, CCSDS sync marker, Reed-Solomon RS(255,223), AOS framing
  │   ├── fdir/            # Watchdog supervisor, thermal PID heater, safety interlocks
  │   ├── kernel/          # Cooperative scheduler, event flags, message queues, 1 kHz executive
  │   └── shell/           # Interactive operator dictionary, diagnostic dumps, bootloader
  ├── sim/                 # Multi-physics simulation models and HIL socket bridge
  │   ├── hil_bridge.py
  │   ├── jitter_generator.py
  │   ├── optical_channel.py
  │   └── orbit_dynamics.py
  ├── src/                 # High-performance ANSI C11 Forth Virtual Machine
  │   └── vm/
  │       ├── trunnion_vm.c
  │       └── trunnion_vm.h
  ├── tests/               # Automated test suites, test harness, and high-rate benchmarks
  │   ├── benchmarks.fs
  │   ├── test_all.fs
  │   ├── test_comm.fs
  │   ├── test_core_math.fs
  │   ├── test_core_utils.fs
  │   ├── test_fdir.fs
  │   ├── test_harness.fs
  │   ├── test_kernel.fs
  │   ├── test_kinematics.fs
  │   ├── test_optics.fs
  │   ├── test_orbit.fs
  │   ├── test_pat.fs
  │   └── test_regression.fs
  ├── tools/               # Developer utilities, metrics auditor, and linting
  │   ├── check_no_em_dash.py
  │   ├── line_count.py
  │   └── telemetry_dashboard.py
  ├── Makefile             # Portable build automation and test target runner
  ├── trunnion.fs          # Master top-level flight software entrypoint
  └── LICENSE              # Apache 2.0 Open Source License
```
