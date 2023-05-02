# Trunnion

[![CI](https://github.com/aneesh495/trunnion/actions/workflows/ci.yml/badge.svg)](https://github.com/aneesh495/trunnion/actions/workflows/ci.yml)
[![Standards: CCSDS 141.0-B-1](https://img.shields.io/badge/CCSDS-141.0--B--1%20Optical-00529B.svg)](https://public.ccsds.org/)
[![Standards: Forth-2012](https://img.shields.io/badge/Forth-2012%20Standard-E05A47.svg)](https://forth-standard.org/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Verification: 10/10 Suites Passing](https://img.shields.io/badge/Verification-100%25%20Passing%20(338%20checks)-brightgreen.svg)](docs/verification.md)
[![Style: Zero Em-Dash](https://img.shields.io/badge/ASCII-Zero%20Em--Dash-blueviolet.svg)](tools/check_no_em_dash.py)

**Trunnion** is a production-grade, deterministically timed Satellite Laser Cross-Link Flight Software, Precision Pointing, Acquisition, and Tracking (PAT) Engine, and CCSDS Optical Communications Subsystem written in Forth.

Designed for Low Earth Orbit (LEO) mega-constellation inter-satellite links (ISL) and deep-space optical communication (DSOC) terminals, Trunnion coordinates a dual-stage optomechanical pointing assembly, evaluates high-precision Keplerian/J2 orbit geometry, compensates for relativistic point-ahead angles, and implements a CCSDS-compliant optical communications physical and data-link layer.

```
       Zenith (+Z)
             ^
             |       Elevation Axis (\theta)
             |          /
             |         /
             |     +--/---+
             |     |  O   |  Telescope Optical Aperture (1550 nm)
             |     +------+
             |        |
             |        +------- Azimuth Axis (\psi)
             |       /
             |      /
             +-----+----------------> Cross-Track (+Y)
            /
           /
          v
      Along-Track (+X)
```

---

## Technical Highlights

- **Pure 64-Bit Forth Flight Software**: Built without dynamic heap allocation or runtime garbage collection, providing sub-microsecond determinism across all control loops.
- **Hierarchical Dual-Stage Pointing Architecture**:
  - **Coarse Pointing Assembly (CPA)**: Hemispherical dual-axis gimbal ($\pm 180^\circ$ azimuth, $-20^\circ / +95^\circ$ elevation) with 7-segment S-curve jerk-limited trajectory generation and LuGre dynamic friction feedforward.
  - **Fine Steering Mirror (FSM)**: High-bandwidth piezoelectric tip-tilt mirror ($\pm 5\text{ mrad}$) running a 200 Hz biquad lead-lag digital filter to suppress spacecraft micro-vibrations and reaction wheel harmonics.
- **Relativistic Point-Ahead Vector Derivation**: Exact computation of transverse and radial velocity aberration vectors over inter-satellite distances up to 5,000 km:
  $$\mathbf{\theta}_{PA} \approx \frac{2}{c} (\mathbf{v}_{rel} - (\mathbf{v}_{rel} \cdot \mathbf{u}) \mathbf{u})$$
- **Multi-Sensor Optical Centroiding**:
  - Wide-Field-of-View Quadrant Photodiode (QPD) differential centroiding.
  - Narrow-Field-of-View (NFOV) sub-pixel Center-of-Gravity (CoG) estimator.
  - Conical nutation tracking with synchronous phase demodulation.
- **CCSDS 141.0-B-1 & 133.0-B-1 Optical Communications Subsystem**:
  - Photon-efficient $M$-ary Pulse Position Modulation (M-PPM with $M \in \{4, 8, 16, 32, 64\}$) with maximum-likelihood decision slicer.
  - 32-bit Attached Sync Marker (`0x1ACFFC1D`) correlator state machine.
  - Reed-Solomon $RS(255, 223)$ forward error correction codec over $GF(2^8)$ (corrects up to 16 byte errors per frame).
  - Matrix block interleaver ($I \in \{1, 2, 4, 8\}$) to mitigate deep optical fading bursts.
  - CCSDS Space Packet Protocol and AOS Transfer Frame multiplexing.
- **Deterministic 1 kHz Executive & Real-Time Kernel**:
  - Cooperative round-robin task scheduler with microsecond hardware timers, event flags, and message queues.
  - Execution headroom exceeding $99.9\%$ at 1 kHz (<0.8 microseconds per active control frame).
- **Dual Runtime Portability**: Runs natively on both Trunnion's zero-dependency ANSI C11 Virtual Machine (`bin/trunnion`) and standard GNU Forth (`gforth`).
- **Comprehensive Verification Suite**: 10 automated test suites verifying 128 top-level assertions across 338 individual checks with 100% pass rate.
- **Multi-Physics Hardware-in-the-Loop (HIL) Simulator**: Socket IPC simulation environment modeling Keplerian orbital dynamics, J2 oblateness secular drift, Gaussian beam propagation, and high-frequency reaction wheel jitter.

---

## System Architecture

```mermaid
flowchart TD
    subgraph Spacecraft [Spacecraft Dynamics & Ephemeris]
        Orbit["Keplerian Propagator + J2 Drift"] --> LOS["Line-of-Sight Unit Vector"]
        LOS --> PAA["Point-Ahead Angle Solver"]
        PAA --> TargetVec["Target Pointing Vector"]
    end

    subgraph OpticalTrain [Optomechanical Train]
        TargetVec --> CPA_Servo["CPA Gimbal Servo (S-Curve + LuGre)"]
        TargetVec --> FSM_Servo["FSM Piezo Servo (200 Hz Biquad)"]
        CPA_Servo --> Gimbal["CPA Motors (+/-180 deg)"]
        FSM_Servo --> Mirror["FSM Piezo Mirror (+/-5 mrad)"]
        Gimbal --> Beam["Output Optical Beam (1550 nm)"]
        Mirror --> Beam
    end

    subgraph OpticalSensors [Optical Sensing & Centroiding]
        RxLight["Incoming Laser Light"] --> Splitter["Dichroic Splitter"]
        Splitter --> QPD["Quadrant Photodiode (WFOV)"]
        Splitter --> NFOV["CMOS Sensor (NFOV CoG)"]
        QPD --> Centroid["Centroid & SNR Estimator"]
        NFOV --> Centroid
        Centroid --> FSM_Servo
        Centroid --> CPA_Servo
    end

    subgraph CommStack [CCSDS Optical Communications Pipeline]
        Splitter --> Photoreceiver["High-Speed Photoreceiver"]
        Photoreceiver --> Demod["M-PPM Slicer & Sync Marker"]
        Demod --> Deint["Matrix Deinterleaver"]
        Deint --> RSDec["Reed-Solomon RS(255, 223) Decoder"]
        RSDec --> Packet["CCSDS Space Packet Multiplexer"]
    end
```

---

## Mathematical Foundations & Theory of Operation

### 1. Keplerian Two-Body Orbit Propagation & J2 Secular Perturbations

Satellite cross-links require anticipating target position along orbit trajectories. Trunnion propagates orbit state vectors using Kepler's equation:

$$M(t) = M_0 + n (t - t_0)$$

$$E - e \sin(E) = M$$

Solved via high-precision Newton-Raphson iteration in Q32.32 fixed-point arithmetic (`forth/orbit/kepler.fs`).

To prevent long-term ephemeris drift, `forth/orbit/j2_perturbation.fs` evaluates the first-order secular drift induced by the Earth's oblateness parameter $J_2 = 1.08263 \times 10^{-3}$:

$$\dot{\Omega}_{J2} = -\frac{3}{2} n J_2 \left(\frac{R_E}{p}\right)^2 \cos(i)$$

$$\dot{\omega}_{J2} = \frac{3}{4} n J_2 \left(\frac{R_E}{p}\right)^2 (5 \cos^2(i) - 1)$$

where $p = a(1 - e^2)$ is the semi-latus rectum and $R_E = 6378.137\text{ km}$ is Earth's equatorial radius.

### 2. Relativistic Point-Ahead Angle (PAA) Exact Derivation

Because optical beams propagate at the speed of light $c$, a laser cross-link spanning distance $R = 5,000\text{ km}$ requires a transit time of $\Delta t = R / c \approx 16.68\text{ ms}$. Over this interval, an orbiting satellite traveling at $v \approx 7.5\text{ km/s}$ moves $\approx 125\text{ meters}$.

```
 Satellite A (Transmitter)                         Target Satellite B
      *                                                 * (At transmit time t_0)
       \                                               /
        \                                             /
         \                                           /  Satellite B motion:
          \  Transmit Line-of-Sight                 /   \Delta r = v_rel * \Delta t
           \                                       v
            \                                     * (At arrival time t_0 + R/c)
             \                                   /
              \   Point-Ahead Vector            /
               \===============================>
```

The required forward pointing offset $\mathbf{\theta}_{PA}$ relative to the apparent line-of-sight unit vector $\mathbf{u}$ is:

$$\mathbf{\theta}_{PA} = \frac{2}{c} \left[ \mathbf{v}_{rel} - (\mathbf{v}_{rel} \cdot \mathbf{u}) \mathbf{u} \right]$$

For typical low-Earth cross-links, $|\mathbf{\theta}_{PA}| \approx 50\text{ }\mu\text{rad}$. Because the 1550 nm telescope beam divergence is only $\approx 15\text{ }\mu\text{rad}$ full-width at half-maximum (FWHM), neglecting the point-ahead correction causes complete signal loss. Trunnion evaluates $\mathbf{\theta}_{PA}$ at 100 Hz in `forth/pat/point_ahead.fs`.

### 3. LuGre Dynamic Friction Model for Gimbal Bearing Actuation

Low-velocity tracking during orbital flybys is limited by non-linear bearing friction and stick-slip phenomena. Trunnion implements the LuGre dynamic friction model (`forth/kinematics/lugre_friction.fs`) to compute feedforward compensation torque:

$$\frac{dz}{dt} = v - \frac{\sigma_0 |v|}{g(v)} z$$

$$g(v) = F_c + (F_s - F_c) e^{-(v / v_s)^2}$$

$$T_f = \sigma_0 z + \sigma_1 \frac{dz}{dt} + \sigma_2 v$$

Injecting $T_f$ into the coarse gimbal motor drive eliminates limit cycles and deadzone hysteresis.

---

## CCSDS Optical Communications Subsystem

The Trunnion communications pipeline implements the physical and data-link layers defined in **CCSDS 141.0-B-1 (Optical Communications Physical Layer)**:

```
+------------------------------------------------------------------------+
| APPLICATION LAYER: Spacecraft Engineering Telemetry & Ground Telecommands
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| CCSDS SPACE PACKETS (CCSDS 133.0-B-1) & AOS FRAMING (CCSDS 732.0-B-3)  |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| REED-SOLOMON RS(255, 223) FORWARD ERROR CORRECTION (GF(2^8))           |
| (Adds 32 Parity Symbols, Corrects up to 16 Byte Errors per Block)      |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| MATRIX BLOCK INTERLEAVER (Configurable Depth I = 1, 2, 4, 8)           |
| (Spreads Burst Fading across Multiple Codewords)                       |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| 32-BIT ATTACHED SYNC MARKER (ASM = 0x1ACFFC1D) CORRELATOR               |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| M-ARY PULSE POSITION MODULATION (M-PPM, M = 16)                        |
| (1550 nm High-Power Narrow-Linewidth Laser)                            |
+------------------------------------------------------------------------+
```

---

## Verification & Test Results

The flight software undergoes continuous automated regression testing across dual execution runtimes (`bin/trunnion` and `gforth`):

```bash
make test
```

| Test Suite | Implementation File | Assertions | Checks | Status |
|:-----------|:--------------------|:----------:|:------:|:------:|
| **Core Fixed-Point, Vector & Quaternion Math** | `tests/test_core_math.fs` | 24 | 71 | **PASS** |
| **Ring Buffers, Bitops, CRC & Core Utilities** | `tests/test_core_utils.fs` | 11 | 26 | **PASS** |
| **CPA Kinematics, Trajectory & FSM Servo** | `tests/test_kinematics.fs` | 10 | 29 | **PASS** |
| **Optical Centroiding & Nutation Tracking** | `tests/test_optics.fs` | 11 | 27 | **PASS** |
| **PAT State Machine & Point-Ahead Solvers** | `tests/test_pat.fs` | 10 | 28 | **PASS** |
| **Orbital Mechanics & Link Line-of-Sight** | `tests/test_orbit.fs` | 11 | 28 | **PASS** |
| **CCSDS Optical Communications & Modem** | `tests/test_comm.fs` | 15 | 36 | **PASS** |
| **FDIR, Thermal PID & Optical Safety** | `tests/test_fdir.fs` | 11 | 29 | **PASS** |
| **Real-Time Kernel, Scheduler & Telemetry** | `tests/test_kernel.fs` | 11 | 28 | **PASS** |
| **Boundary & Edge-Case Regressions** | `tests/test_regression.fs` | 14 | 36 | **PASS** |
| **TOTAL VERIFICATION MATRIX** | `tests/test_all.fs` | **128** | **338** | **100% PASS** |

---

## High-Rate Performance Benchmarks

Execution throughput evaluated across 1,000,000 iterations per benchmark on host hardware:

```bash
make bench
```

| Benchmark Operation | Sample Size | Execution Time | Measured Throughput | Real-Time Headroom Margin |
|:--------------------|:-----------:|:--------------:|:-------------------:|:-------------------------:|
| **Q32.32 Fixed-Point Math** | 1,000,000 | 12.8 ms | ~78.1 MOPS | >99.9% at 1 kHz |
| **CORDIC 32-Iteration Rotations** | 100,000 | 489.2 ms | ~204.4k rot/s | >99.5% at 1 kHz |
| **3x3 Matrix-Vector Transforms** | 100,000 | 58.7 ms | ~1.70M xforms/s | >99.9% at 1 kHz |
| **Reed-Solomon RS(255, 223) Encoding**| 1,000 | 5.2 ms | ~192.3 KB/s | >95.0% at 100 kbps |
| **1 kHz Real-Time Executive Frame** | 10,000 | 7.9 ms | ~1.26M cycles/s | **>99.9% Real-Time Margin** |

---

## Codebase Architecture & Scale Metrics

Audited via `tools/line_count.py`:

| Subsystem Layer | Files | Blank Lines | Comment Lines | Code Lines | Total Lines | Comment % |
|:----------------|:-----:|:-----------:|:-------------:|:----------:|:-----------:|:---------:|
| **Forth Flight Software** | 76 | 1,529 | 3,121 | 5,856 | 10,506 | 29.7% |
| **ANSI C11 Virtual Machine** | 3 | 135 | 105 | 1,688 | 1,928 | 5.4% |
| **HIL Simulation & Telemetry** | 7 | 201 | 202 | 841 | 1,244 | 16.2% |
| **Technical Documentation** | 9 | 329 | 0 | 944 | 1,273 | - |
| **Build System & CI** | 4 | 20 | 4 | 73 | 97 | 4.1% |
| **TOTAL REPOSITORY SCALE** | **99** | **2,214** | **3,432** | **9,402** | **15,048** | **22.8%** |

---

## Quickstart & Operator Manual

### 1. Build the Flight Software

```bash
# Compile native C11 virtual machine
make clean
make all
```

### 2. Launch Interactive Flight Software Shell

```bash
./bin/trunnion -i trunnion.fs
```

Type `HELP` for command listings or `STATUS` to view live avionics telemetry:

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

### 3. Run Multi-Physics HIL Simulation

```bash
# Execute closed-loop orbit and optical link simulation
make sim
```

### 4. Launch Live Telemetry Terminal Dashboard

```bash
# Real-time ANSI terminal engineering telemetry display
python3 tools/telemetry_dashboard.py
```

---

## Detailed Technical Documentation

Trunnion includes 8 comprehensive mathematical and system specifications in [`docs/`](docs/README.md):

1. **[Avionics & Software Architecture](docs/architecture.md)**: 64-bit Forth runtime, cooperative scheduler, 1 kHz executive cadence, and memory model.
2. **[Architecture Diagrams](docs/architecture_diagram.md)**: Optical bench relay layout, dual-stage control loop, PAT state machine, and CCSDS protocol stack.
3. **[PAT Theory & Derivations](docs/pat_theory.md)**: Spiral search equations, QPD/NFOV optical centroiding, relativistic point-ahead angle derivation.
4. **[Mechanism Dynamics & Servo Control](docs/cpa_fsm_dynamics.md)**: CPA kinematics, 7-segment S-curve profile, LuGre friction, and FSM biquad lead-lag filter.
5. **[CCSDS Optical Communications](docs/ccsds_optcomm.md)**: M-PPM modulation, 32-bit ASM correlator, Reed-Solomon RS(255, 223) codec, and matrix interleaver.
6. **[Memory Map & HAL Specification](docs/memory_map.md)**: Physical peripheral register offsets, binary housekeeping telemetry format.
7. **[Verification & Benchmark Plan](docs/verification.md)**: 10 test suite breakdowns, dual-target verification, and high-rate benchmark methodology.
8. **[Getting Started & Operator Manual](docs/getting_started.md)**: Toolchain requirements, REPL command dictionary, and HIL simulation guide.

---

## Coding Standards & Em-Dash Enforcement

All source files, comments, documentation, and tools strictly prohibit typographical em-dashes and en-dashes (`\u2014` and `\u2013`) to guarantee clean, uncorrupted ASCII rendering across cross-compilers and serial terminals. Continuous verification is enforced via:

```bash
python3 tools/check_no_em_dash.py
```

---

## License

Trunnion is open-source software licensed under the [Apache License, Version 2.0](LICENSE).
