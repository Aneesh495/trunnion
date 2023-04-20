# Flight Software Technical Documentation Index

Welcome to the engineering documentation suite for **Trunnion**, a production-grade Satellite Laser Cross-Link Flight Software, Precision Pointing, Acquisition, and Tracking (PAT) Engine, and CCSDS Optical Communications Subsystem.

---

## Technical Specifications & Theory Documents

| Document | Description | Key Topics |
|:---------|:------------|:-----------|
| **[System Architecture](architecture.md)** | Core avionics flight software design | 64-bit Forth-2012 runtime, memory layout, cooperative multi-tasking scheduler, 1 kHz executive cadence |
| **[System Architecture Diagrams](architecture_diagram.md)** | Detailed subsystem block diagrams | Optical bench train, dual-stage control loop, PAT state machine, CCSDS protocol stack, real-time frame timeline |
| **[PAT Theory & Derivations](pat_theory.md)** | Pointing, acquisition, and tracking algorithms | Archimedean spiral search, optical centroiding (QPD/NFOV), relativistic point-ahead angle derivation, dual-stage control |
| **[Mechanism Dynamics & Servo Control](cpa_fsm_dynamics.md)** | Optomechanical dynamics and actuation | CPA gimbal kinematics, 7-segment S-curve trajectory, LuGre dynamic friction model, FSM biquad lead-lag filter, offload supervisor |
| **[CCSDS Optical Communications](ccsds_optcomm.md)** | Space communications physical and data link layers | M-ary Pulse Position Modulation (M-PPM), CCSDS 32-bit ASM correlator, Reed-Solomon RS(255, 223) codec, matrix block interleaving, AOS framing |
| **[Memory Map & Hardware Abstraction](memory_map.md)** | Peripheral register address space & telemetry layout | MMIO registers (CPA, FSM, QPD, Laser, Comm, Timer), 32-byte binary housekeeping telemetry format, bitfield allocations |
| **[Verification Plan & Test Matrix](verification.md)** | Comprehensive flight software assurance | 10 automated test suites (128 assertions, 338 checks), 100% pass rate, dual-runtime validation (C11 VM and Gforth), high-rate benchmarks |
| **[Getting Started & Operator Manual](getting_started.md)** | Developer quickstart & command reference | Building from source, interactive REPL console, operator dictionary, HIL simulation setup, telemetry terminal dashboard |

---

## Navigation & Cross-References

For an end-to-end overview of the physical mission, mathematical foundations, and benchmark results, refer to the top-level repository [README.md](../README.md).
