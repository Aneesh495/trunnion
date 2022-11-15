\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/fsm_servo.fs
\ Description: High-bandwidth (2 kHz) digital lead-lag compensator with active
\              damping for micro-radian optical beam jitter rejection.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ FSM Digital Lead-Lag Compensator Gains (Q32.32):
\ Crossover frequency: 200 Hz, Phase lead peak: 45 deg at 200 Hz
\ Discrete recurrence relation: u[n] = a1*u[n-1] + b0*e[n] + b1*e[n-1]
\
\ Coefficients (f_sample = 2000 Hz):
\   b0 = +2.4142 * 2^32 = 10369165312
\   b1 = -1.9856 * 2^32 = -8528151240
\   a1 = +0.5714 * 2^32 = 2454157840
10369165312 CONSTANT FSM-LEAD-B0
-8528151240 CONSTANT FSM-LEAD-B1
2454157840 CONSTANT FSM-LEAD-A1

VARIABLE FSM-TIP-LAST-E
VARIABLE FSM-TIP-LAST-U
VARIABLE FSM-TILT-LAST-E
VARIABLE FSM-TILT-LAST-U

\ ------------------------------------------------------------------------------
\ Section 1: Initialization
\ ------------------------------------------------------------------------------

\ FSM-SERVO-INIT ( -- )
: FSM-SERVO-INIT ( -- )
    0 FSM-TIP-LAST-E !
    0 FSM-TIP-LAST-U !
    0 FSM-TILT-LAST-E !
    0 FSM-TILT-LAST-U !
    FSM-FILTERS-INIT
;

\ FSM-SERVO-RESET ( -- )
: FSM-SERVO-RESET ( -- )
    0 FSM-TIP-LAST-E !
    0 FSM-TIP-LAST-U !
    0 FSM-TILT-LAST-E !
    0 FSM-TILT-LAST-U !
    FSM-NOTCH-TIP BIQUAD-RESET
    FSM-NOTCH-TILT BIQUAD-RESET
;

\ ------------------------------------------------------------------------------
\ Section 2: 2 kHz Control Loop Step
\ ------------------------------------------------------------------------------

VARIABLE FSM-CTRL-ERR-TIP
VARIABLE FSM-CTRL-ERR-TILT
VARIABLE FSM-OUT-TIP
VARIABLE FSM-OUT-TILT

\ FSM-SERVO-STEP ( err_tip_nrad err_tilt_nrad -- )
\ Execute one discrete 2 kHz control step.
: FSM-SERVO-STEP ( err_tip err_tilt -- )
    FSM-CTRL-ERR-TILT !
    FSM-CTRL-ERR-TIP !

    \ Process Tip axis lead-lag compensator
    FSM-CTRL-ERR-TIP @ S>Q32
    FSM-LEAD-B0 Q32*
    FSM-TIP-LAST-E @ S>Q32 FSM-LEAD-B1 Q32* +
    FSM-TIP-LAST-U @ S>Q32 FSM-LEAD-A1 Q32* +
    \ Pass through 850 Hz structural notch filter
    FSM-NOTCH-TIP-STEP
    Q32>S
    DUP FSM-TIP-LAST-U !
    FSM-OUT-TIP !
    FSM-CTRL-ERR-TIP @ FSM-TIP-LAST-E !

    \ Process Tilt axis lead-lag compensator
    FSM-CTRL-ERR-TILT @ S>Q32
    FSM-LEAD-B0 Q32*
    FSM-TILT-LAST-E @ S>Q32 FSM-LEAD-B1 Q32* +
    FSM-TILT-LAST-U @ S>Q32 FSM-LEAD-A1 Q32* +
    \ Pass through 850 Hz notch filter
    FSM-NOTCH-TILT-STEP
    Q32>S
    DUP FSM-TILT-LAST-U !
    FSM-OUT-TILT !
    FSM-CTRL-ERR-TILT @ FSM-TILT-LAST-E !

    \ Dispatch to push-pull piezo tripod
    FSM-OUT-TIP @ FSM-OUT-TILT @
    FSM-ACTUATE-ANGLES
;
