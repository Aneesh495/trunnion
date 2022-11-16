\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/optics/nfov_detector.fs
\ Description: Narrow Field-of-View (NFOV) pixelated CMOS focal plane centroiding
\              with adaptive background thresholding and sub-pixel peak fitting.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Focal Plane Parameters:
\ Tracking ROI Window: 16x16 pixels
\ Pixel pitch: 15.0 micrometers
\ Focal length: 300.0 mm
\ Angular scale: 50.0 microradians per pixel
50 CONSTANT NFOV-URAD-PER-PIXEL

VARIABLE NFOV-IMG-PTR
VARIABLE NFOV-WIDTH
VARIABLE NFOV-HEIGHT
VARIABLE NFOV-THRESH
VARIABLE NFOV-SUM-INTENSITY
VARIABLE NFOV-SUM-WX
VARIABLE NFOV-SUM-WY

\ ------------------------------------------------------------------------------
\ Section 1: Center-of-Gravity (CoG) Centroid Solver
\ ------------------------------------------------------------------------------

\ NFOV-PIXEL@ ( x y -- intensity )
: NFOV-PIXEL@ ( x y -- val )
    NFOV-WIDTH @ * + CELLS NFOV-IMG-PTR @ + @
;

\ NFOV-COG ( img_addr width height threshold -- cx_q32 cy_q32 valid_flag )
\ Compute intensity-weighted center of gravity across pixel sub-array.
: NFOV-COG ( img w h thresh -- cx cy flag )
    NFOV-THRESH !
    NFOV-HEIGHT !
    NFOV-WIDTH !
    NFOV-IMG-PTR !

    0 NFOV-SUM-INTENSITY !
    0 NFOV-SUM-WX !
    0 NFOV-SUM-WY !

    NFOV-HEIGHT @ 0 DO
        NFOV-WIDTH @ 0 DO
            I J NFOV-PIXEL@
            DUP NFOV-THRESH @ > IF
                NFOV-THRESH @ - \ Subtract noise pedestal
                DUP NFOV-SUM-INTENSITY @ + NFOV-SUM-INTENSITY !
                \ Accumulate X moment: i * val
                DUP I S>Q32 Q32* NFOV-SUM-WX @ + NFOV-SUM-WX !
                \ Accumulate Y moment: j * val
                J S>Q32 Q32* NFOV-SUM-WY @ + NFOV-SUM-WY !
            ELSE
                DROP
            THEN
        LOOP
    LOOP

    \ Check sufficient integrated intensity
    NFOV-SUM-INTENSITY @ 50 < IF
        0 0 FALSE EXIT
    THEN

    \ cx = sum_wx / sum_intensity
    NFOV-SUM-WX @ NFOV-SUM-INTENSITY @ S>Q32 Q32/
    \ cy = sum_wy / sum_intensity
    NFOV-SUM-WY @ NFOV-SUM-INTENSITY @ S>Q32 Q32/
    TRUE
;

\ ------------------------------------------------------------------------------
\ Section 2: Sub-Pixel Parabolic Peak Interpolation
\ ------------------------------------------------------------------------------

VARIABLE PEAK-X
VARIABLE PEAK-Y
VARIABLE PEAK-VAL

\ NFOV-FIND-PEAK ( img_addr width height -- px py pval )
\ Find integer pixel coordinates of brightest spot in window.
: NFOV-FIND-PEAK ( img w h -- px py pval )
    NFOV-HEIGHT !
    NFOV-WIDTH !
    NFOV-IMG-PTR !
    0 PEAK-VAL ! 0 PEAK-X ! 0 PEAK-Y !

    NFOV-HEIGHT @ 0 DO
        NFOV-WIDTH @ 0 DO
            I J NFOV-PIXEL@
            DUP PEAK-VAL @ > IF
                PEAK-VAL !
                I PEAK-X !
                J PEAK-Y !
            ELSE
                DROP
            THEN
        LOOP
    LOOP
    PEAK-X @ PEAK-Y @ PEAK-VAL @
;

\ NFOV-PARABOLIC-FIT ( v_left v_center v_right -- delta_subpix_q32 )
\ 1D 3-point parabolic peak estimator: delta = 0.5 * (v_left - v_right) / (v_left - 2*v_center + v_right)
: NFOV-PARABOLIC-FIT ( vl vc vr -- delta )
    ROT 2DUP - S>Q32 >R \ R: (vl - vr)
    -ROT 2* - S>Q32 \ denom = vl - 2*vc + vr
    DUP 0= IF
        DROP R> DROP 0 EXIT
    THEN
    R> SWAP Q32/
    $80000000 Q32* \ Multiply by 0.5
;
