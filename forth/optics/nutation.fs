\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/optics/nutation.fs
\ Description: Conical beam nutation modulation generator and phase-sensitive
\              synchronous demodulator for single-photodiode optical tracking.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Nutation Tracking Principles:
\ Modulates optical line-of-sight in a circular scan of radius r_nut:
\   theta_x(t) = r_nut * cos(omega_nut * t)
\   theta_y(t) = r_nut * sin(omega_nut * t)
\
\ The received optical power on a single-element APD detector is modulated by
\ the spatial gradient of the Gaussian beam profile.
\ Synchronous in-phase and quadrature demodulation extracts the 2D offset vector:
\   Error_x = (2 / N) * sum(P_k * cos(omega * t_k))
\   Error_y = (2 / N) * sum(P_k * sin(omega * t_k))

\ Nutation Parameters:
\ Radius: 2.0 microradians = 2000 nanoradians
2000 CONSTANT NUTATION-DEFAULT-RADIUS-NRAD
\ 4 discrete samples per nutation circle (omega * dt = 90 deg)
4 CONSTANT NUTATION-SAMPLES-PER-REV
\ Integration window: 16 revolutions (64 samples = 32 ms at 2 kHz)
64 CONSTANT NUTATION-WINDOW-SIZE

VARIABLE NUTATION-RADIUS-NRAD
VARIABLE NUTATION-SUM-I      \ In-phase accumulator (X tracking error)
VARIABLE NUTATION-SUM-Q      \ Quadrature accumulator (Y tracking error)
VARIABLE NUTATION-SAMPLE-CNT \ Sample counter within window
VARIABLE NUTATION-SUM-POWER  \ Total integrated power for AGC normalization

\ ------------------------------------------------------------------------------
\ Section 1: Modulation Trajectory Generator
\ ------------------------------------------------------------------------------

\ NUTATION-INIT ( radius_nrad -- )
: NUTATION-INIT ( radius -- )
    NUTATION-RADIUS-NRAD !
    0 NUTATION-SUM-I !
    0 NUTATION-SUM-Q !
    0 NUTATION-SAMPLE-CNT !
    0 NUTATION-SUM-POWER !
;

\ NUTATION-RESET ( -- )
: NUTATION-RESET ( -- )
    0 NUTATION-SUM-I !
    0 NUTATION-SUM-Q !
    0 NUTATION-SAMPLE-CNT !
    0 NUTATION-SUM-POWER !
;

\ NUTATION-GEN-OFFSET ( sample_idx -- offset_x_nrad offset_y_nrad )
\ Return instantaneous tip/tilt offset to be injected onto FSM mirror.
: NUTATION-GEN-OFFSET ( idx -- off_x off_y )
    NUTATION-SAMPLES-PER-REV MOD
    CASE
        0 OF NUTATION-RADIUS-NRAD @ 0 ENDOF
        1 OF 0 NUTATION-RADIUS-NRAD @ ENDOF
        2 OF NUTATION-RADIUS-NRAD @ NEGATE 0 ENDOF
        3 OF 0 NUTATION-RADIUS-NRAD @ NEGATE ENDOF
        0 0 ROT \ Default fallback
    ENDCASE
;

\ ------------------------------------------------------------------------------
\ Section 2: Phase-Sensitive Demodulation
\ ------------------------------------------------------------------------------

\ NUTATION-ACCUMULATE ( sample_idx power_nw -- )
\ Correlate measured optical power against harmonic in-phase and quadrature carriers.
: NUTATION-ACCUMULATE ( idx power -- )
    SWAP NUTATION-SAMPLES-PER-REV MOD
    OVER NUTATION-SUM-POWER @ + NUTATION-SUM-POWER !
    CASE
        0 OF \ cos = +1, sin = 0
            NUTATION-SUM-I @ + NUTATION-SUM-I !
        ENDOF
        1 OF \ cos = 0, sin = +1
            NUTATION-SUM-Q @ + NUTATION-SUM-Q !
        ENDOF
        2 OF \ cos = -1, sin = 0
            NUTATION-SUM-I @ SWAP - NUTATION-SUM-I !
        ENDOF
        3 OF \ cos = 0, sin = -1
            NUTATION-SUM-Q @ SWAP - NUTATION-SUM-Q !
        ENDOF
        DROP
    ENDCASE
    1 NUTATION-SAMPLE-CNT +!
;

\ NUTATION-EXTRACT-ERRORS ( -- err_x_nrad err_y_nrad valid_flag )
\ Extract normalized pointing error vectors once window integration completes.
: NUTATION-EXTRACT-ERRORS ( -- err_x err_y flag )
    NUTATION-SAMPLE-CNT @ NUTATION-WINDOW-SIZE < IF
        0 0 FALSE EXIT
    THEN
    NUTATION-SUM-POWER @ 100 < IF \ Loss of optical signal
        NUTATION-RESET
        0 0 FALSE EXIT
    THEN

    \ Normalized error: Err_x = (NUTATION-SUM-I / NUTATION-SUM-POWER) * K_nut
    NUTATION-SUM-I @ S>Q32
    NUTATION-SUM-POWER @ S>Q32 Q32/
    NUTATION-RADIUS-NRAD @ S>Q32 Q32*
    2 S>Q32 Q32* Q32>S \ err_x

    NUTATION-SUM-Q @ S>Q32
    NUTATION-SUM-POWER @ S>Q32 Q32/
    NUTATION-RADIUS-NRAD @ S>Q32 Q32*
    2 S>Q32 Q32* Q32>S \ err_y

    NUTATION-RESET
    TRUE
;
