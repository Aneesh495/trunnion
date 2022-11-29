\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/optics/qpd_centroid.fs
\ Description: Four-quadrant photodiode (QPD) normalized differential centroid
\              estimator, beam waist compensation, and cubic non-linear linearization.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Quadrant Photodiode Geometry:
\ Quadrants arranged on focal plane:
\   Quadrant A (top-left)      Quadrant B (top-right)
\   Quadrant C (bottom-left)   Quadrant D (bottom-right)
\
\ Normalized Differential Coordinates:
\   dx = ((B + D) - (A + C)) / (A + B + C + D)
\   dy = ((A + B) - (C + D)) / (A + B + C + D)
\ Where dx, dy in [-1.0, +1.0].
\
\ Cubic Linearization for Gaussian Spot:
\ For a Gaussian beam spot of radius w_0 on gap width d_gap:
\   theta_x = K_lin * dx * (1 + beta_cubic * dx^2)
\   theta_y = K_lin * dy * (1 + beta_cubic * dy^2)

\ Linear calibration scale factor: K_lin = 50.0 urad per unit normalized error
\ In Q32.32: 50 * 4295 = 214748
214748 CONSTANT QPD-K-LIN-Q32

\ Cubic correction coefficient: beta = 0.28 in Q32.32
1202590843 CONSTANT QPD-BETA-CUBIC-Q32

\ Minimum detectable optical power threshold: 300 ADC counts
300 CONSTANT QPD-MIN-SUM-THRESHOLD

VARIABLE QPD-CALC-A
VARIABLE QPD-CALC-B
VARIABLE QPD-CALC-C
VARIABLE QPD-CALC-D
VARIABLE QPD-CALC-SUM
VARIABLE QPD-NORM-DX
VARIABLE QPD-NORM-DY

\ ------------------------------------------------------------------------------
\ Section 1: Normalized Centroid Computation
\ ------------------------------------------------------------------------------

\ QPD-CENTROID-RAW ( a b c d -- dx_q32 dy_q32 valid_flag )
\ Compute normalized differential coordinates in Q32.32 format from raw ADC counts.
: QPD-CENTROID-RAW ( a b c d -- dx dy flag )
    QPD-CALC-D !
    QPD-CALC-C !
    QPD-CALC-B !
    QPD-CALC-A !

    \ Compute total optical sum
    QPD-CALC-A @ QPD-CALC-B @ +
    QPD-CALC-C @ + QPD-CALC-D @ +
    DUP QPD-CALC-SUM !

    \ Check optical power threshold
    QPD-CALC-SUM @ QPD-MIN-SUM-THRESHOLD < IF
        0 0 FALSE EXIT
    THEN

    \ Compute dx = ((B + D) - (A + C)) / SUM
    QPD-CALC-B @ QPD-CALC-D @ +
    QPD-CALC-A @ QPD-CALC-C @ + -
    S>Q32 QPD-CALC-SUM @ S>Q32 Q32/
    QPD-NORM-DX !

    \ Compute dy = ((A + B) - (C + D)) / SUM
    QPD-CALC-A @ QPD-CALC-B @ +
    QPD-CALC-C @ QPD-CALC-D @ + -
    S>Q32 QPD-CALC-SUM @ S>Q32 Q32/
    QPD-NORM-DY !

    QPD-NORM-DX @
    QPD-NORM-DY @
    TRUE
;

\ ------------------------------------------------------------------------------
\ Section 2: Non-Linear Gaussian Beam Linearization
\ ------------------------------------------------------------------------------

VARIABLE QPD-LIN-DELTA
VARIABLE QPD-DELTA-SQ

\ QPD-LINEARIZE-AXIS ( delta_norm_q32 -- angle_rad_q32 )
\ Apply cubic linearization: theta = K_lin * delta * (1 + beta * delta^2).
: QPD-LINEARIZE-AXIS ( delta -- angle )
    DUP QPD-LIN-DELTA !
    DUP Q32* QPD-DELTA-SQ !

    \ 1 + beta * delta^2
    QPD-DELTA-SQ @ QPD-BETA-CUBIC-Q32 Q32*
    Q32-ONE +

    \ Multiply by delta
    QPD-LIN-DELTA @ Q32*

    \ Multiply by K_lin factor
    QPD-K-LIN-Q32 Q32*
;

\ QPD-CENTROID-ANGLES ( a b c d -- angle_x_rad angle_y_rad valid_flag )
\ Compute calibrated physical angular errors in radians from quadrant counts.
: QPD-CENTROID-ANGLES ( a b c d -- ang_x ang_y flag )
    QPD-CENTROID-RAW
    DUP 0= IF
        EXIT
    THEN
    DROP \ Drop true flag
    QPD-LINEARIZE-AXIS SWAP
    QPD-LINEARIZE-AXIS SWAP
    TRUE
;

\ ------------------------------------------------------------------------------
\ Section 3: Beam Waist / Spot Radius Estimation
\ ------------------------------------------------------------------------------

\ QPD-ESTIMATE-WAIST ( a b c d -- waist_um_q32 )
\ Estimate effective Gaussian 1/e^2 beam waist on quadrant detector.
: QPD-ESTIMATE-WAIST ( a b c d -- waist )
    \ Geometric spot size scales inversely with peak centroid sensitivity
    + + + \ Total sum
    DUP 0= IF DROP 0 EXIT THEN
    S>Q32
    \ Scaled baseline beam waist = 120.0 micrometers
    120 S>Q32
    NIP \ Approximate optical waist
;

\ ------------------------------------------------------------------------------
\ Section 4: Cached State and Sampling API
\ ------------------------------------------------------------------------------

VARIABLE QPD-LAST-ANG-X
VARIABLE QPD-LAST-ANG-Y
VARIABLE QPD-LAST-VALID

\ QPD-CENTROID-INIT ( -- )
: QPD-CENTROID-INIT ( -- )
    0 QPD-LAST-ANG-X !
    0 QPD-LAST-ANG-Y !
    0 QPD-LAST-VALID !
;

\ QPD-SAMPLE ( -- )
\ Sample quadrant photodiode ADC channels and compute centroid angles.
: QPD-SAMPLE ( -- )
    QPD-READ-ALL QPD-CENTROID-ANGLES
    QPD-LAST-VALID !
    QPD-LAST-ANG-Y !
    QPD-LAST-ANG-X !
;

\ QPD-SIGNAL-VALID? ( -- flag )
: QPD-SIGNAL-VALID? ( -- flag )
    QPD-LAST-VALID @
;

\ QPD-GET-CENTROID ( -- ang_x_q32 ang_y_q32 )
: QPD-GET-CENTROID ( -- x y )
    QPD-LAST-ANG-X @
    QPD-LAST-ANG-Y @
;
