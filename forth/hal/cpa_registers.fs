\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/cpa_registers.fs
\ Description: Coarse Pointing Assembly (CPA) hardware register driver,
\              resolver count conversion, and motor drive interfaces.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ CPA Encoder Resolution: 24-bit optical absolute multi-turn resolver
\ 1 revolution (2*pi rad) = 16,777,216 counts (0x1000000)
\ Conversion factor: 1 tick = (2*pi / 16777216) rad = 0.37453988 urad
\ In Q32.32: TICKS_TO_RAD_Q32 = 26986075409 / 16777216 = 1608
1608 CONSTANT CPA-TICKS-TO-RAD-Q32
16777216 CONSTANT CPA-TICKS-PER-REV

\ Mechanical Travel Limits:
\ Azimuth: Full 360 degree rotation with slip-ring [-pi, +pi]
\ Elevation: Trunnion mount travel [-15 deg, +95 deg]
\ -15 deg in Q32.32 = -353380295
\ +95 deg in Q32.32 = +2239103230
-353380295 CONSTANT CPA-EL-MIN-RAD
2239103230 CONSTANT CPA-EL-MAX-RAD

\ Motor Current Limit: +/- 3000 mA
3000 CONSTANT CPA-MAX-CURRENT-MA

\ ------------------------------------------------------------------------------
\ Section 1: Power and Status Control
\ ------------------------------------------------------------------------------

\ CPA-ENABLE ( -- )
\ Energize CPA motor H-bridges and resolver excitation.
: CPA-ENABLE ( -- )
    REG-CPA-CTRL HW@
    CPA-STAT-ENABLED OR
    REG-CPA-CTRL HW!
;

\ CPA-DISABLE ( -- )
\ Safely disable CPA motor drivers (freewheel/brake).
: CPA-DISABLE ( -- )
    REG-CPA-CTRL HW@
    CPA-STAT-ENABLED INVERT AND
    REG-CPA-CTRL HW!
    \ Clear velocity command DACs
    0 REG-CPA-AZ-CMD-RATE HW!
    0 REG-CPA-EL-CMD-RATE HW!
;

\ CPA-STATUS@ ( -- stat )
: CPA-STATUS@ ( -- stat )
    REG-CPA-STATUS HW@
;

\ CPA-ENABLED? ( -- flag )
: CPA-ENABLED? ( -- flag )
    CPA-STATUS@ CPA-STAT-ENABLED AND 0<>
;

\ ------------------------------------------------------------------------------
\ Section 2: Resolver Telemetry and Coordinate Conversion
\ ------------------------------------------------------------------------------

\ CPA-AZ-TICKS@ ( -- ticks )
: CPA-AZ-TICKS@ ( -- ticks )
    REG-CPA-AZ-POS HW@
;

\ CPA-EL-TICKS@ ( -- ticks )
: CPA-EL-TICKS@ ( -- ticks )
    REG-CPA-EL-POS HW@
;

\ CPA-AZ-ANGLE@ ( -- rad_q32 )
\ Convert current azimuth resolver ticks to Q32.32 radians.
: CPA-AZ-ANGLE@ ( -- rad_q32 )
    CPA-AZ-TICKS@ CPA-TICKS-TO-RAD-Q32 *
    \ Wrap into [-pi, +pi]
    BEGIN
        DUP Q32-PI >
    WHILE
        Q32-TWO-PI -
    REPEAT
    BEGIN
        DUP Q32-PI NEGATE <
    WHILE
        Q32-TWO-PI +
    REPEAT
;

\ CPA-EL-ANGLE@ ( -- rad_q32 )
\ Convert current elevation resolver ticks to Q32.32 radians.
: CPA-EL-ANGLE@ ( -- rad_q32 )
    CPA-EL-TICKS@ CPA-TICKS-TO-RAD-Q32 *
;

\ CPA-RAD>TICKS ( rad_q32 -- ticks )
\ Convert Q32.32 angle in radians to resolver ticks.
: CPA-RAD>TICKS ( rad -- ticks )
    CPA-TICKS-TO-RAD-Q32 /
;

\ ------------------------------------------------------------------------------
\ Section 3: Motor Rate Commands & Actuation
\ ------------------------------------------------------------------------------

\ CPA-AZ-CMD-RATE! ( mA -- )
\ Command azimuth motor winding current clamped to safe limits.
: CPA-AZ-CMD-RATE! ( mA -- )
    CPA-MAX-CURRENT-MA NEGATE CPA-MAX-CURRENT-MA CLAMP
    REG-CPA-AZ-CMD-RATE HW!
;

\ CPA-EL-CMD-RATE! ( mA -- )
\ Command elevation motor winding current clamped to safe limits.
: CPA-EL-CMD-RATE! ( mA -- )
    CPA-MAX-CURRENT-MA NEGATE CPA-MAX-CURRENT-MA CLAMP
    REG-CPA-EL-CMD-RATE HW!
;

\ CPA-LIMIT-CHECK ( az_rad el_rad -- flag )
\ Return TRUE if commanded azimuth and elevation angles are within mechanical limits.
: CPA-LIMIT-CHECK ( az el -- flag )
    DUP CPA-EL-MIN-RAD >= SWAP CPA-EL-MAX-RAD <= AND
    SWAP DUP Q32-PI NEGATE >= SWAP Q32-PI <= AND AND
;
