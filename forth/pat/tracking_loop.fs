\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/tracking_loop.fs
\ Description: Dual-stage Line-Of-Sight (LOS) closed-loop tracking engine.
\              Coordinates Coarse Pointing Assembly (CPA) and Fast Steering
\              Mirror (FSM) actuators with sub-microradian residual error.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Tracking Loop State and Tuning Parameters
\ ------------------------------------------------------------------------------

VARIABLE TRACK-ENABLED
VARIABLE TRACK-ITERATION-COUNT
VARIABLE TRACK-SAMPLE-TIME-US    \ Loop sample period (nominal 1000 us = 1 kHz)

\ Residual tracking error (Q32.32 radians)
VARIABLE TRACK-RESID-AZ
VARIABLE TRACK-RESID-EL
VARIABLE TRACK-RMS-ERROR-URAD    \ Rolling RMS error in microradians

\ Cumulative tracking metrics
VARIABLE TRACK-IN-SPEC-TICKS     \ Consecutive samples where error < 1.0 urad
VARIABLE TRACK-OUT-SPEC-TICKS    \ Consecutive samples where error >= 1.0 urad

\ Spec threshold: 1.0 microradian (nominal diffraction-limited threshold)
1 CONSTANT TRACK-SPEC-THRESHOLD-URAD

\ ------------------------------------------------------------------------------
\ Section 2: Loop Initialization and Configuration
\ ------------------------------------------------------------------------------

\ TRACK-LOOP-INIT ( -- )
\ Initialize tracking loop filters, integrators, and metric counters.
: TRACK-LOOP-INIT ( -- )
    0 TRACK-ENABLED !
    0 TRACK-ITERATION-COUNT !
    1000 TRACK-SAMPLE-TIME-US !
    0 TRACK-RESID-AZ !
    0 TRACK-RESID-EL !
    0 TRACK-RMS-ERROR-URAD !
    0 TRACK-IN-SPEC-TICKS !
    0 TRACK-OUT-SPEC-TICKS !
    FSM-SERVO-RESET
    CPA-SERVO-INIT
;

\ TRACK-LOOP-ENABLE ( -- )
\ Enable closed-loop dual-stage tracking.
: TRACK-LOOP-ENABLE ( -- )
    1 TRACK-ENABLED !
    0 TRACK-IN-SPEC-TICKS !
    0 TRACK-OUT-SPEC-TICKS !
;

\ TRACK-LOOP-DISABLE ( -- )
\ Disable closed-loop tracking and park FSM actuators.
: TRACK-LOOP-DISABLE ( -- )
    0 TRACK-ENABLED !
    0 0 FSM-SET-COMMAND
;

\ TRACK-LOOP-ACTIVE? ( -- flag )
: TRACK-LOOP-ACTIVE? ( -- flag )
    TRACK-ENABLED @ 0<>
;

\ ------------------------------------------------------------------------------
\ Section 3: Dual-Stage LOS Error Processing
\ ------------------------------------------------------------------------------

\ TRACK-STEP-DUAL-STAGE ( az_err_q32 el_err_q32 -- )
\ Execute one iteration of the dual-stage tracking controller:
\ 1. Incorporate relativistic point-ahead angle correction.
\ 2. Convert optical error to microradians.
\ 3. Apply FSM high-bandwidth lead-lag and notch compensator.
\ 4. Evaluate FSM stroke offload to CPA gimbal motors.
\ 5. Step hardware actuators and accumulate performance metrics.
: TRACK-STEP-DUAL-STAGE ( az_err_q32 el_err_q32 -- )
    TRACK-ENABLED @ 0= IF
        2DROP EXIT
    THEN

    1 TRACK-ITERATION-COUNT +!

    \ Apply point-ahead angle compensation
    PAA-AZ-CORRECTION @ Q32+ SWAP
    PAA-EL-CORRECTION @ Q32+ SWAP

    \ Store current residual error
    DUP TRACK-RESID-EL !
    OVER TRACK-RESID-AZ !

    \ Convert error to microradians for fast servo evaluation
    RAD>URAD-Q32 SWAP RAD>URAD-Q32 SWAP

    \ Compute error magnitude: hypot(az_urad, el_urad)
    2DUP DUP * SWAP DUP * +
    INT-SQRT
    DUP TRACK-RMS-ERROR-URAD !

    \ Update in-spec / out-of-spec counters
    TRACK-SPEC-THRESHOLD-URAD < IF
        1 TRACK-IN-SPEC-TICKS +!
        0 TRACK-OUT-SPEC-TICKS !
    ELSE
        0 TRACK-IN-SPEC-TICKS !
        1 TRACK-OUT-SPEC-TICKS +!
    THEN

    \ Execute FSM closed-loop servo step (input: az_urad el_urad)
    FSM-SERVO-STEP

    \ Query FSM deflection for offload management
    FSM-GET-DEFLECTION
    \ Convert urad to Q32.32 for offload threshold evaluation
    URAD>RAD-Q32 SWAP URAD>RAD-Q32 SWAP
    OFFLOAD-STEP

    \ If offload triggered, apply rate to CPA
    OFFLOAD-ACTIVE? IF
        OFFLOAD-AZ-RATE @ OFFLOAD-EL-RATE @ CPA-SET-CMD-RATE
    THEN

    \ Step simulated hardware time by sample period
    TRACK-SAMPLE-TIME-US @ HW-STEP
;

\ ------------------------------------------------------------------------------
\ Section 4: Performance Diagnostics
\ ------------------------------------------------------------------------------

\ TRACK-IN-SPEC? ( -- flag )
\ Return TRUE if tracking error has remained within specification for >= 100 ticks.
: TRACK-IN-SPEC? ( -- flag )
    TRACK-IN-SPEC-TICKS @ 100 >=
;

\ TRACK-PRINT-STATUS ( -- )
\ Display current tracking performance, RMS error, and offload state.
: TRACK-PRINT-STATUS ( -- )
    CR ." [TRUNNION TRACKING LOOP STATUS]" CR
    ."   Enabled:       " TRACK-ENABLED @ IF ." YES" ELSE ." NO" THEN CR
    ."   Iterations:    " TRACK-ITERATION-COUNT @ . CR
    ."   RMS Error:     " TRACK-RMS-ERROR-URAD @ . ." urad" CR
    ."   In-Spec Ticks: " TRACK-IN-SPEC-TICKS @ . CR
    ."   Offload Active:" OFFLOAD-ACTIVE? IF ."  YES" ELSE ."  NO" THEN CR
;
