\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/orbit/j2_perturbation.fs
\ Description: Earth gravitational oblateness (J2) secular drift propagator.
\              Computes nodal regression rate and apsidal precession rate.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: J2 Secular Drift Rates
\ ------------------------------------------------------------------------------

VARIABLE J2-RE-OVER-P-SQ-Q32    \ (R_earth / p)^2 (Q32.32)
VARIABLE J2-FACTOR-Q32          \ 1.5 * J2 * (R_e / p)^2 * n (Q32.32)
VARIABLE J2-COS-I-Q32           \ cos(i)
VARIABLE J2-COS-I-SQ-Q32        \ cos^2(i)

VARIABLE J2-RAAN-RATE-RAD-S-Q32 \ dOmega/dt (rad/s Q32.32)
VARIABLE J2-AOP-RATE-RAD-S-Q32  \ domega/dt (rad/s Q32.32)
VARIABLE J2-MA-RATE-RAD-S-Q32   \ dM/dt correction (rad/s Q32.32)

\ J2-UPDATE-RATES ( -- )
\ Calculate instantaneous secular drift rates based on current orbital elements.
: J2-UPDATE-RATES ( -- )
    \ cos(i)
    ORB-INC-RAD-Q32 @ CORDIC-COS
    DUP J2-COS-I-Q32 !
    DUP Q32* J2-COS-I-SQ-Q32 !

    \ (R_earth / p)^2:
    ORB-EARTH-RADIUS-KM-Q32 ORB-SEMI-LATUS-KM-Q32 @ Q32/
    DUP Q32*
    J2-RE-OVER-P-SQ-Q32 !

    \ Base factor: 1.5 * J2 * (R_e / p)^2 * n
    \ 1.5 in Q32.32 = 6442450944
    6442450944 ORB-J2-Q32 Q32*
    J2-RE-OVER-P-SQ-Q32 @ Q32*
    ORB-MEAN-MOTION-Q32 @ Q32*
    J2-FACTOR-Q32 !

    \ RAAN drift rate: dOmega/dt = -1.5 * J2 * (R_e/p)^2 * n * cos(i)
    J2-FACTOR-Q32 @ J2-COS-I-Q32 @ Q32* NEGATE
    J2-RAAN-RATE-RAD-S-Q32 !

    \ Apsidal precession rate: domega/dt = 0.75 * J2 * (R_e/p)^2 * n * (5*cos^2(i) - 1)
    \ Notice 0.75 is 0.5 * factor (factor has 1.5)
    J2-FACTOR-Q32 @ 2/
    5 S>Q32 J2-COS-I-SQ-Q32 @ Q32* Q32-ONE -
    Q32*
    J2-AOP-RATE-RAD-S-Q32 !

    \ Mean anomaly secular correction: dM_J2/dt = 0.75 * J2 * (R_e/p)^2 * n * sqrt(1 - e^2) * (3*cos^2(i) - 1)
    J2-FACTOR-Q32 @ 2/
    Q32-ONE ORB-ECC-Q32 @ DUP Q32* - CORDIC-SQRT Q32*
    3 S>Q32 J2-COS-I-SQ-Q32 @ Q32* Q32-ONE -
    Q32*
    J2-MA-RATE-RAD-S-Q32 !
;

\ ------------------------------------------------------------------------------
\ Section 2: Perturbed Propagation Step
\ ------------------------------------------------------------------------------

\ J2-APPLY-PERTURBATION ( dt_sec_q32 -- )
\ Advance orbital orientation angles by J2 secular drift over dt_sec seconds.
: J2-APPLY-PERTURBATION ( dt_sec -- )
    J2-UPDATE-RATES

    \ Delta RAAN = dOmega/dt * dt
    DUP J2-RAAN-RATE-RAD-S-Q32 @ Q32*
    ORB-RAAN-RAD-Q32 @ +
    \ Normalize to [0, 2*pi)
    Q32-TWO-PI /MOD DROP
    DUP 0< IF Q32-TWO-PI + THEN
    ORB-RAAN-RAD-Q32 !

    \ Delta AOP = domega/dt * dt
    DUP J2-AOP-RATE-RAD-S-Q32 @ Q32*
    ORB-AOP-RAD-Q32 @ +
    Q32-TWO-PI /MOD DROP
    DUP 0< IF Q32-TWO-PI + THEN
    ORB-AOP-RAD-Q32 !

    \ Delta Mean Anomaly correction
    J2-MA-RATE-RAD-S-Q32 @ Q32*
    ORB-MA-RAD-Q32 @ +
    Q32-TWO-PI /MOD DROP
    DUP 0< IF Q32-TWO-PI + THEN
    ORB-MA-RAD-Q32 !
;

\ J2-PRINT-RATES ( -- )
\ Print the secular nodal regression and apsidal precession rates in degrees/day.
: J2-PRINT-RATES ( -- )
    J2-UPDATE-RATES
    CR ." [J2 ORBITAL SECULAR DRIFT RATES]" CR
    ."   RAAN Rate (dOmega/dt): "
    J2-RAAN-RATE-RAD-S-Q32 @ RAD>DEG-Q32 86400 * Q32>S . ." deg/day" CR
    ."   AOP Rate (domega/dt):  "
    J2-AOP-RATE-RAD-S-Q32 @ RAD>DEG-Q32 86400 * Q32>S . ." deg/day" CR
;
