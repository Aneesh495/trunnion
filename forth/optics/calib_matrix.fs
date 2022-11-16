\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/optics/calib_matrix.fs
\ Description: Optical bench thermo-elastic distortion and sensor non-orthogonality
\              calibration matrix model.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Baseline Calibration Matrix C_0 (close to identity with micro-radian misalignments)
\ Off-diagonal cross-coupling terms: ~500 urad mechanical non-orthogonality
CREATE CALIB-MATRIX 72 ALLOT

\ Thermal distortion coefficient: 15.0 nanoradians per Kelvin in Q32.32
64 CONSTANT CALIB-THERMAL-COEFF-Q32

VARIABLE CALIB-CURRENT-TEMP-MK
293150 CONSTANT CALIB-REF-TEMP-MK \ Reference 20.0 deg C

\ ------------------------------------------------------------------------------
\ Section 1: Calibration Matrix Setup
\ ------------------------------------------------------------------------------

\ CALIB-INIT ( -- )
: CALIB-INIT ( -- )
    CALIB-MATRIX M3-IDENTITY
    \ Inject calibrated non-orthogonality:
    \ c01 = -0.0005 rad = -2147483 in Q32.32
    -2147483 CALIB-MATRIX 0 1 M3!
    \ c10 = +0.0005 rad = +2147483 in Q32.32
    2147483 CALIB-MATRIX 1 0 M3!
    CALIB-REF-TEMP-MK CALIB-CURRENT-TEMP-MK !
;

\ CALIB-UPDATE-TEMP ( temp_mK -- )
\ Update thermo-elastic distortion terms as optical bench temperature shifts.
: CALIB-UPDATE-TEMP ( temp -- )
    CALIB-CURRENT-TEMP-MK !
;

\ ------------------------------------------------------------------------------
\ Section 2: Sensor Alignment Correction
\ ------------------------------------------------------------------------------

\ CALIB-APPLY ( rx ry rz -- cx cy cz )
\ Multiply raw sensor vector by calibration matrix to obtain true optical LOS.
: CALIB-APPLY ( rx ry rz -- cx cy cz )
    CALIB-MATRIX -ROT M3*V3
;
