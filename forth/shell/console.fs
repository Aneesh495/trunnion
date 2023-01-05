\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/shell/console.fs
\ Description: Interactive Console Banner, Operator Help Menu & Command Prompt
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Flight Software Welcome Banner
\ ------------------------------------------------------------------------------

\ CONSOLE-BANNER ( -- )
\ Output ASCII art banner and build version string.
: CONSOLE-BANNER ( -- )
    CR
    ."  _______ _____  _     _ __   _ __   _ _____  _____  __   _" CR
    ."     |    |_____] |     | | \  | | \  |   |   |     | | \  |" CR
    ."     |    |     \ |_____| |  \_| |  \_| __|__ |_____| |  \_|" CR
    CR
    ."  ================================================================" CR
    ."   TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine" CR
    ."   Version 1.0.0-RELEASE (Target: LEO-to-LEO / LEO-to-GEO OISL)" CR
    ."   Copyright (c) 2026 Aneesh Krishna Parthasarathy. ISC License." CR
    ."  ================================================================" CR
    CR
;

\ ------------------------------------------------------------------------------
\ Section 2: Operator Help System
\ ------------------------------------------------------------------------------

\ HELP ( -- )
\ Display comprehensive operator reference for interactive shell commands.
: HELP ( -- )
    CR
    ." [TRUNNION FLIGHT SOFTWARE COMMAND REFERENCE]" CR
    ." ----------------------------------------------------------------" CR
    ."  GIMBAL & STEERING:" CR
    ."    az el CMD-SLEW          Initiate coordinated S-curve CPA gimbal slew" CR
    ."    CMD-HOME                Slew CPA gimbal to stow position (0, 0)" CR
    ."    tip tilt CMD-FSM        Actuate Fast Steering Mirror piezo (uRad)" CR
    ."    CMD-FSM-ZERO            Center Fast Steering Mirror to boresight" CR
    CR
    ."  LASER TRANSMITTER:" CR
    ."    power_mw CMD-LASER-ON   Arm laser, set power, open safety shutter" CR
    ."    CMD-LASER-OFF           Close shutter, ramp down, disarm laser" CR
    CR
    ."  PAT ENGINE (POINTING, ACQUISITION, TRACKING):" CR
    ."    CMD-START-PAT           Transition PAT state machine to ACQUISITION" CR
    ."    CMD-ABORT-PAT           Abort PAT sequence and enter SAFE HOLD" CR
    ."    CMD-STANDBY             Transition terminal to STANDBY mode" CR
    CR
    ."  OPTICAL COMMUNICATIONS:" CR
    ."    order CMD-SET-PPM       Set M-PPM modulation order (4, 16, 64)" CR
    ."    CMD-SEND-PING           Format and transmit test CCSDS ping packet" CR
    CR
    ."  DIAGNOSTICS & TELEMETRY:" CR
    ."    DUMP-ALL                Full multi-page engineering diagnostic dump" CR
    ."    DUMP-REGISTERS          Memory-mapped peripheral bus register table" CR
    ."    DUMP-GIMBAL             CPA angles, trajectory velocity, LuGre friction" CR
    ."    DUMP-FSM                FSM piezo voltages, strain feedback, offload" CR
    ."    DUMP-OPTICS             QPD quad signals, centroid, spot size, SNR" CR
    ."    DUMP-PAT                PAT state, spiral search, point-ahead vector" CR
    ."    DUMP-COMM               Modem status, Reed-Solomon stats, channel BER" CR
    ."    DUMP-FAULTS             Latched flight software and hardware faults" CR
    CR
    ."  EXECUTIVE & SCHEDULER:" CR
    ."    ms CMD-STEP             Step real-time executive for N milliseconds" CR
    ."    CMD-RESET-FAULTS        Clear and reset all latched fault conditions" CR
    ."    CMD-STATUS              Single-line concise system health status" CR
    ."    HELP                    Display this operator reference guide" CR
    ."    BYE                     Exit Trunnion flight software runtime" CR
    ." ----------------------------------------------------------------" CR
;

\ ------------------------------------------------------------------------------
\ Section 3: Interactive Bootstrapping
\ ------------------------------------------------------------------------------

\ CONSOLE-WELCOME ( -- )
\ Boot message presented upon interactive startup.
: CONSOLE-WELCOME ( -- )
    CONSOLE-BANNER
    ." [SYSTEM BOOT] Memory subsystems and hardware bus initialized." CR
    ." [SYSTEM BOOT] Real-time executive ready. All 4 tasks registered." CR
    ." [SYSTEM BOOT] Type 'HELP' for commands or 'DUMP-ALL' for full telemetry." CR
    CR
;
