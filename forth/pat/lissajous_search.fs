\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/lissajous_search.fs
\ Description: Anisotropic 2D Lissajous spatial scanning generator for elliptical
\              ephemeris uncertainty regions (in-track vs cross-track).
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Anisotropic Lissajous Pattern:
\ In satellite orbits, along-track ephemeris uncertainty is typically 3x to 5x
\ larger than cross-track and radial orbital position uncertainty.
\ Lissajous scanning parameters:
\   x(t) = A_x * sin(omega_x * t)
\   y(t) = A_y * sin(omega_y * t + delta_phase)
\ Ratio omega_x / omega_y chosen as non-harmonic ratio (e.g. 11/8 or 17/12)
\ to guarantee dense ergodic spatial filling without re-tracing identical lines.

VARIABLE LISS-AMP-X-NRAD
VARIABLE LISS-AMP-Y-NRAD
VARIABLE LISS-OMEGA-X-Q32
VARIABLE LISS-OMEGA-Y-Q32
VARIABLE LISS-PHASE-DELTA-Q32
VARIABLE LISS-TIME-US
VARIABLE LISS-ACTIVE

\ ------------------------------------------------------------------------------
\ Section 1: Initialization
\ ------------------------------------------------------------------------------

\ LISSAJOUS-INIT ( amp_x amp_y freq_x_hz freq_y_hz -- )
: LISSAJOUS-INIT ( ax ay fx fy -- )
    S>Q32 Q32-TWO-PI Q32* LISS-OMEGA-Y-Q32 !
    S>Q32 Q32-TWO-PI Q32* LISS-OMEGA-X-Q32 !
    LISS-AMP-Y-NRAD !
    LISS-AMP-X-NRAD !
    Q32-QUARTER-PI LISS-PHASE-DELTA-Q32 ! \ 45 deg phase offset
    0 LISS-TIME-US !
    FALSE LISS-ACTIVE !
;

\ LISSAJOUS-INIT-DEFAULT ( -- )
\ Initialize Lissajous scan pattern with nominal 10 mrad amplitude and 15/23 Hz frequencies.
: LISSAJOUS-INIT-DEFAULT ( -- )
    10000000 10000000 15 23 LISSAJOUS-INIT
;

\ LISSAJOUS-START ( -- )
: LISSAJOUS-START ( -- )
    0 LISS-TIME-US !
    TRUE LISS-ACTIVE !
;

\ LISSAJOUS-STOP ( -- )
: LISSAJOUS-STOP ( -- )
    FALSE LISS-ACTIVE !
;

\ ------------------------------------------------------------------------------
\ Section 2: Real-Time Pattern Stepping
\ ------------------------------------------------------------------------------

VARIABLE LISS-TIME-SEC-Q32
VARIABLE LISS-THETA-X
VARIABLE LISS-THETA-Y

\ LISSAJOUS-STEP ( dt_us -- off_x_nrad off_y_nrad )
\ Compute next 2D offset vector on Lissajous trajectory.
: LISSAJOUS-STEP ( dt -- off_x off_y )
    LISS-ACTIVE @ 0= IF
        0 0 EXIT
    THEN

    LISS-TIME-US @ +
    LISS-TIME-US !

    LISS-TIME-US @ S>Q32 1000000 S>Q32 Q32/
    LISS-TIME-SEC-Q32 !

    \ theta_x = omega_x * t
    LISS-TIME-SEC-Q32 @ LISS-OMEGA-X-Q32 @ Q32*
    CORDIC-SIN
    LISS-AMP-X-NRAD @ S>Q32 Q32* Q32>S

    \ theta_y = omega_y * t + delta
    LISS-TIME-SEC-Q32 @ LISS-OMEGA-Y-Q32 @ Q32*
    LISS-PHASE-DELTA-Q32 @ +
    CORDIC-SIN
    LISS-AMP-Y-NRAD @ S>Q32 Q32* Q32>S
;
