\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/offload.fs
\ Description: Dual-stage FSM-to-CPA offload controller transferring low-frequency
\              pointing bias to the coarse gimbal to prevent piezo stroke saturation.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Offload Threshold: 5.0 mrad = 5,000,000 nanoradians
\ (50% of the FSM maximum mechanical travel)
5000000 CONSTANT OFFLOAD-THRESHOLD-NRAD

\ Low-pass filter time constant: tau = 0.5 seconds
\ At 100 Hz offload rate: alpha = exp(-dt/tau) ~ 0.9802 = 4209930740 in Q32.32
4209930740 CONSTANT OFFLOAD-ALPHA-Q32

VARIABLE OFFLOAD-ACC-TIP     \ Low-pass filtered tip angle (nanoradians Q32.32)
VARIABLE OFFLOAD-ACC-TILT    \ Low-pass filtered tilt angle (nanoradians Q32.32)
VARIABLE OFFLOAD-AZ-RATE
VARIABLE OFFLOAD-EL-RATE
VARIABLE OFFLOAD-TRIGGERED

\ ------------------------------------------------------------------------------
\ Section 1: Initialization
\ ------------------------------------------------------------------------------

\ OFFLOAD-INIT ( -- )
: OFFLOAD-INIT ( -- )
    0 OFFLOAD-ACC-TIP !
    0 OFFLOAD-ACC-TILT !
    0 OFFLOAD-AZ-RATE !
    0 OFFLOAD-EL-RATE !
    0 OFFLOAD-TRIGGERED !
    FSM-OFFLOAD-RESET
;

\ OFFLOAD-ACTIVE? ( -- flag )
: OFFLOAD-ACTIVE? ( -- flag )
    OFFLOAD-TRIGGERED @
;

\ ------------------------------------------------------------------------------
\ Section 2: Offload Integration and Threshold Detection
\ ------------------------------------------------------------------------------

VARIABLE OFFLOAD-RAW-TIP
VARIABLE OFFLOAD-RAW-TILT
VARIABLE OFFLOAD-DELTA-AZ
VARIABLE OFFLOAD-DELTA-EL

\ OFFLOAD-STEP ( dt_s -- )
\ Update low-pass filtered FSM bias and command CPA coarse adjustment if threshold exceeded.
: OFFLOAD-STEP ( dt -- )
    DROP \ Fixed filter pole uses precomputed alpha
    \ Read current FSM capacitive sensor feedback
    FSM-TIP-FB@ S>Q32 OFFLOAD-RAW-TIP !
    FSM-TILT-FB@ S>Q32 OFFLOAD-RAW-TILT !

    \ Filter tip: acc_tip = alpha * acc_tip + (1 - alpha) * raw_tip
    OFFLOAD-ACC-TIP @ OFFLOAD-ALPHA-Q32 Q32*
    OFFLOAD-RAW-TIP @ Q32-ONE OFFLOAD-ALPHA-Q32 - Q32* +
    OFFLOAD-ACC-TIP !

    \ Filter tilt: acc_tilt = alpha * acc_tilt + (1 - alpha) * raw_tilt
    OFFLOAD-ACC-TILT @ OFFLOAD-ALPHA-Q32 Q32*
    OFFLOAD-RAW-TILT @ Q32-ONE OFFLOAD-ALPHA-Q32 - Q32* +
    OFFLOAD-ACC-TILT !

    \ Check if tip offload threshold exceeded
    OFFLOAD-ACC-TIP @ Q32>S ABS OFFLOAD-THRESHOLD-NRAD > IF
        \ Convert nanoradians to Q32.32 radians
        OFFLOAD-ACC-TIP @ Q32>S S>Q32 1000000000 S>Q32 Q32/
        OFFLOAD-DELTA-AZ !
        \ Command CPA azimuth slew by delta
        CPA-AZ-ANGLE@ OFFLOAD-DELTA-AZ @ +
        0.01 S>Q32 CPA-SERVO-AZ-STEP
        \ Deduct offloaded bias from accumulator
        0 OFFLOAD-ACC-TIP !
    THEN

    \ Check if tilt offload threshold exceeded
    OFFLOAD-ACC-TILT @ Q32>S ABS OFFLOAD-THRESHOLD-NRAD > IF
        OFFLOAD-ACC-TILT @ Q32>S S>Q32 1000000000 S>Q32 Q32/
        OFFLOAD-DELTA-EL !
        CPA-EL-ANGLE@ OFFLOAD-DELTA-EL @ +
        0.01 S>Q32 CPA-SERVO-EL-STEP
        0 OFFLOAD-ACC-TILT !
    THEN
;

\ OFFLOAD-THRESHOLD-CHECK ( -- flag )
\ Return TRUE if FSM is currently operating within linear non-saturated regime.
: OFFLOAD-THRESHOLD-CHECK ( -- flag )
    OFFLOAD-ACC-TIP @ Q32>S ABS OFFLOAD-THRESHOLD-NRAD <=
    OFFLOAD-ACC-TILT @ Q32>S ABS OFFLOAD-THRESHOLD-NRAD <= AND
;
