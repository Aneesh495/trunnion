\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_optics.fs
\ Description: Test Suite for QPD Centroiding, NFOV CoG, Nutation, and SNR
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Optics, QPD Centroid, NFOV CoG, Nutation Scan, Calibration, and Link Budget" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Quadrant Photodiode Centroiding
\ ------------------------------------------------------------------------------

S" QPD centroid: symmetric illumination produces zero offset" TEST-CASE-START
1000 1000 1000 1000 QPD-CENTROID-RAW
ASSERT-TRUE \ valid flag
0 S>Q32 100 ASSERT-Q32-CLOSE \ dy = 0
0 S>Q32 100 ASSERT-Q32-CLOSE \ dx = 0

S" QPD centroid: right-half illumination produces positive dx" TEST-CASE-START
500 1500 500 1500 QPD-CENTROID-RAW
ASSERT-TRUE
0 S>Q32 100 ASSERT-Q32-CLOSE \ dy = 0
\ dx = ((1500 + 1500) - (500 + 500)) / 4000 = 2000 / 4000 = 0.5 in Q32.32
Q32-ONE 2/ 100 ASSERT-Q32-CLOSE \ dx = +0.5

S" QPD centroid: below detection threshold returns invalid" TEST-CASE-START
50 50 50 50 QPD-CENTROID-RAW
ASSERT-FALSE
DROP DROP

S" QPD cubic linearization: zero input yields zero angle" TEST-CASE-START
0 S>Q32 QPD-LINEARIZE-AXIS 0 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 2: NFOV Focal Plane Center of Gravity
\ ------------------------------------------------------------------------------

CREATE SYNTHETIC-FOCAL-PLANE 256 CELLS ALLOT

: SPOT! ( val x y -- )
    16 * + CELLS SYNTHETIC-FOCAL-PLANE + !
;

: SETUP-SYNTHETIC-SPOT ( -- )
    SYNTHETIC-FOCAL-PLANE 256 CELLS ERASE
    \ Inject symmetric Gaussian beam spot centered at (8, 8)
    100 7 7 SPOT!
    500 8 7 SPOT!
    100 9 7 SPOT!

    500 7 8 SPOT!
    2000 8 8 SPOT!
    500 9 8 SPOT!

    100 7 9 SPOT!
    500 8 9 SPOT!
    100 9 9 SPOT!
;

S" NFOV center of gravity: symmetric 3x3 spot centered at (8, 8)" TEST-CASE-START
SETUP-SYNTHETIC-SPOT
SYNTHETIC-FOCAL-PLANE 16 16 50 NFOV-COG
ASSERT-TRUE \ valid flag
8 S>Q32 100000 ASSERT-Q32-CLOSE \ cy = 8.0
8 S>Q32 100000 ASSERT-Q32-CLOSE \ cx = 8.0

S" NFOV parabolic peak fitting: symmetric peak yields zero sub-pixel offset" TEST-CASE-START
500 1000 500 NFOV-PARABOLIC-FIT
0 S>Q32 100 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 3: Conical Beam Nutation Modulation and Demodulation
\ ------------------------------------------------------------------------------

S" Nutation modulation: 4-quadrant trajectory generation" TEST-CASE-START
2000 NUTATION-INIT
0 NUTATION-GEN-OFFSET
0 ASSERT-EQ 2000 ASSERT-EQ \ ( +R, 0 )
1 NUTATION-GEN-OFFSET
2000 ASSERT-EQ 0 ASSERT-EQ \ ( 0, +R )
2 NUTATION-GEN-OFFSET
0 ASSERT-EQ -2000 ASSERT-EQ \ ( -R, 0 )
3 NUTATION-GEN-OFFSET
-2000 ASSERT-EQ 0 ASSERT-EQ \ ( 0, -R )

: RUN-NUTATION-TEST-CYCLE ( -- )
    2000 NUTATION-INIT
    \ Simulate 16 revolutions (64 samples) with optical signal offset toward +X
    16 0 DO
        0 1200 NUTATION-ACCUMULATE \ sample 0 (cos=+1): higher power
        1 1000 NUTATION-ACCUMULATE \ sample 1 (sin=+1): nominal
        2  800 NUTATION-ACCUMULATE \ sample 2 (cos=-1): lower power
        3 1000 NUTATION-ACCUMULATE \ sample 3 (sin=-1): nominal
    LOOP
;

S" Nutation demodulation: synchronous detection extracts +X error" TEST-CASE-START
RUN-NUTATION-TEST-CYCLE
NUTATION-EXTRACT-ERRORS
ASSERT-TRUE \ valid flag
0 50 ASSERT-CLOSE \ err_y = 0
0> ASSERT-TRUE    \ err_x > 0

\ ------------------------------------------------------------------------------
\ Test Group 4: Optical SNR and Link Budget
\ ------------------------------------------------------------------------------

S" Fixed-point base-10 logarithm: log10(10) = 1.0, log10(100) = 2.0" TEST-CASE-START
10 S>Q32 LOG10-Q32 Q32-ONE 1000 ASSERT-Q32-CLOSE
100 S>Q32 LOG10-Q32 Q32-ONE 2* 1000 ASSERT-Q32-CLOSE

S" Link margin calculation: 100 nW rx vs 10 nW sensitivity = +10.0 dB" TEST-CASE-START
100 10 LINK-MARGIN-DB
Q32-ONE 10 * 1000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 5: Bench Calibration Matrix
\ ------------------------------------------------------------------------------

S" Calibration matrix: identity transform with small cross-coupling" TEST-CASE-START
CALIB-INIT
Q32-ONE 0 S>Q32 0 S>Q32 CALIB-APPLY
\ Returns ( cx cy cz ) with cz on TOS
0 S>Q32 1000 ASSERT-Q32-CLOSE \ cz = 0
2147483 1000 ASSERT-Q32-CLOSE \ cy = +0.0005 rad cross-coupling
Q32-ONE 1000 ASSERT-Q32-CLOSE \ cx = 1.0
