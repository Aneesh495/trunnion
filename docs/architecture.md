# TRUNNION Avionics System Architecture

Trunnion is a high-performance, deterministic flight software and Precision Pointing, Acquisition, and Tracking (PAT) engine for satellite Optical Inter-Satellite Links (OISL). Designed for low-Earth orbit (LEO) constellations and deep-space optical terminals, Trunnion coordinates high-rate electro-optical mechanisms, wavefront sensors, and photon-counting modems within strict microsecond jitter boundaries.

The core flight software is implemented entirely in ANSI Forth (Forth-2012 compliant), underpinned by a dedicated zero-dependency ANSI C11 virtual machine (`trunnion-vm`) for embedded execution, with Python 3 hardware-in-the-loop (HIL) simulation tooling.

---

## 1. System Architectural Hierarchy

The flight software architecture is partitioned into six decoupled layers:

```
+========================================================================+
|                       MISSION OPERATOR & GROUND                        |
|        Interactive Diagnostic Shell / CCSDS Telecommand Uplink         |
+========================================================================+
                                   |
                                   v
+========================================================================+
|                     LAYER 5: FAULT & SYSTEM EXECUTIVE                 |
|   Rate-Monotonic Scheduler (1 kHz / 100 Hz / 10 Hz / 1 Hz Cadence)     |
|   Avionics Watchdog Supervisor | Optical Safety Interlocks | FDIR Log  |
+========================================================================+
                                   |
                                   v
+========================================================================+
|               LAYER 4: PAYLOAD CONTROL & COMMUNICATIONS                |
|   PAT Finite State Machine (Standby, Slew, Search, Acquire, Track)     |
|   CCSDS 141.0-B-1 M-PPM Modem | Reed-Solomon Codec | Interleaver       |
|   AOS Transfer Frames | Space Packet Protocol Engine                   |
+========================================================================+
                                   |
                                   v
+========================================================================+
|                  LAYER 3: DUAL-STAGE ACTUATION CONTROL                 |
|   Coarse Pointing Assembly (CPA): Gimbal Kinematics & LuGre Friction   |
|   Fine Steering Mirror (FSM): Piezo Tip/Tilt & Lead-Lag Biquad Filters |
|   Offload Supervisor: Low-Pass Desaturation to Gimbal Gearing          |
+========================================================================+
                                   |
                                   v
+========================================================================+
|               LAYER 2: SENSOR PROCESSING & ASTRODYNAMICS               |
|   Quadrant Photodiode (QPD) Centroiding | Narrow-FOV Peak Detection    |
|   Conical Nutation Demodulator | SNR Estimator                         |
|   Keplerian Orbit Propagator | J2 Secular Drift | Earth Occultation    |
+========================================================================+
                                   |
                                   v
+========================================================================+
|                LAYER 1: CORE MATHEMATICS & DATA STRUCTURES             |
|   Q32.32 & Q16.16 Fixed-Point Math | 32-Stage CORDIC Vector Engine     |
|   3D Vector & 3x3 Direction Cosine Matrix (DCM) Linear Algebra         |
|   Hamilton Quaternions (SLERP) | Lock-Free Circular Ring Buffers       |
+========================================================================+
                                   |
                                   v
+========================================================================+
|               LAYER 0: HARDWARE ABSTRACTION LAYER (HAL)                |
|   Simulated Hardware Bus (Memory-Mapped Registers, Timers, DMA)        |
|   Portable ANSI C11 Forth Virtual Machine / Host Bare-Metal Runtime    |
+========================================================================+
```

---

## 2. Multi-Rate Executive Cadence Loop

Spacecraft laser cross-links require sub-microradian optical pointing stability in the presence of wideband micro-vibrations from reaction wheel assemblies and solar array drives. To satisfy these stringent dynamic requirements, Trunnion organizes its real-time cooperative scheduler into four harmonic dispatch tiers:

| Tier | Cadence | Period | Subsystem Task Responsibilities |
| :--- | :--- | :--- | :--- |
| **Fast Servo** | 1000 Hz | 1.0 ms | QPD centroid sampling, FSM piezo actuation, lead-lag biquad filtering, offload accumulation, watchdog heartbeat |
| **Gimbal S-Curve** | 100 Hz | 10.0 ms | CPA stepper/BLDC trajectory stepping, LuGre dynamic friction estimation, cogging torque cancellation |
| **PAT Supervisor** | 10 Hz | 100.0 ms | Master PAT finite state machine transitions, spiral/Lissajous scan stepping, relativistic point-ahead computation |
| **Avionics & FDIR**| 1 Hz | 1000.0 ms | Optical bench heater PID loop, laser safety interlock audit, sun blinding checks, housekeeping telemetry generation |

---

## 3. Dual-Runtime Execution Architecture

Trunnion supports dual-runtime execution without modifying a single line of Forth application source code:

1. **Embedded Zero-Dependency Runtime (`bin/trunnion`)**:
   - Written in ISO C11 (`src/vm/trunnion_vm.c`).
   - Implements an indirect-threaded virtual machine with separate data, float, return, and control-flow stacks.
   - Embeds simulated hardware registers with microsecond-accurate bus access (`HW@`, `HW!`, `HW-STEP`).
   - Self-contained binary with zero external package dependencies.

2. **Standard Host Forth Runtime (e.g. Gforth / SwiftForth)**:
   - Evaluates the identical Forth code tree via the portable compatibility preamble (`forth/core/preamble.fs`).
   - Provides portable fallbacks for system words, 128-bit scaling, and memory-mapped bus access.
   - Used for desktop interactive development, REPL debugging, and independent verification.

---

## 4. Deterministic Memory and Concurrency Model

Flight software determinism is guaranteed by design:

- **Zero Dynamic Allocation at Runtime**: All task control blocks (TCBs), message queues, ring buffers, and matrix workspaces are statically pre-allocated at compile time (`ALLOT` / `CREATE`). No heap fragmentation or garbage collection pauses can occur.
- **Lock-Free Circular Buffers**: Inter-task telemetry streams, optical DMA detector frames, and event queues communicate via bounded circular FIFO ring buffers with power-of-two modulo addressing.
- **Strict Return-Stack Discipline**: Forth return-stack operations (`>R`, `R>`) are strictly isolated to single expressions and never maintained across loop boundaries (`DO ... LOOP`), preventing return-frame corruption.
