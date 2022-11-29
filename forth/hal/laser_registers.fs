\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/laser_registers.fs
\ Description: 1550nm DFB seed laser, EDFA optical power amplifier, and
\              mechanical safety shutter hardware drivers.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Laser Power Parameters:
\ Maximum output power: 5000 mW (5.0 Watts optical)
\ Nominal acquisition beacon power: 1000 mW
\ Communication data mode power: 2500 mW
5000 CONSTANT LASER-MAX-POWER-MW
1000 CONSTANT LASER-BEACON-POWER-MW
2500 CONSTANT LASER-COMM-POWER-MW

\ Temperature limits for optical bench (in milli-Kelvin):
\ 273.15 K to 323.15 K (0 to 50 deg C)
273150 CONSTANT LASER-TEMP-MIN-MK
323150 CONSTANT LASER-TEMP-MAX-MK

\ ------------------------------------------------------------------------------
\ Section 1: Interlocks and Arming Sequence
\ ------------------------------------------------------------------------------

\ LASER-STATUS@ ( -- stat )
: LASER-STATUS@ ( -- stat )
    REG-LASER-STATUS HW@
;

\ LASER-INTERLOCK-OK? ( -- flag )
\ Verify all hardware safety interlocks (solar exclusion, thermal, voltage) are closed.
: LASER-INTERLOCK-OK? ( -- flag )
    LASER-STATUS@ LASER-STAT-INTERLOCK-CLOSED AND 0<>
;

\ LASER-ARM ( -- success_flag )
\ Execute automated laser arming sequence if interlocks and thermal loops are healthy.
: LASER-ARM ( -- flag )
    LASER-INTERLOCK-OK? 0= IF
        FALSE EXIT
    THEN
    REG-LASER-TEMP HW@
    DUP LASER-TEMP-MIN-MK < SWAP LASER-TEMP-MAX-MK > OR IF
        FALSE EXIT
    THEN
    \ Turn on seed laser diode
    REG-LASER-CTRL HW@
    LASER-STAT-SEED-READY OR
    REG-LASER-CTRL HW!
    TRUE
;

\ LASER-DISARM ( -- )
\ Instantly shut down pump diodes, close shutter, and disable laser emission.
: LASER-DISARM ( -- )
    0 REG-LASER-PUMP-CUR HW!
    0 REG-LASER-SEED-CUR HW!
    0 REG-LASER-SHUTTER HW!
    REG-LASER-CTRL HW@
    LASER-STAT-EMISSION-ACTIVE INVERT AND
    LASER-STAT-SHUTTER-OPEN INVERT AND
    REG-LASER-CTRL HW!
;

\ ------------------------------------------------------------------------------
\ Section 2: Shutter and Power Actuation
\ ------------------------------------------------------------------------------

\ LASER-SHUTTER-OPEN ( -- )
\ Energize shutter solenoid to unblock optical exit pupil.
: LASER-SHUTTER-OPEN ( -- )
    LASER-INTERLOCK-OK? IF
        1 REG-LASER-SHUTTER HW!
        REG-LASER-STATUS HW@
        LASER-STAT-SHUTTER-OPEN OR
        REG-LASER-STATUS HW!
    THEN
;

\ LASER-SHUTTER-CLOSE ( -- )
\ De-energize shutter solenoid (spring-return beam dump).
: LASER-SHUTTER-CLOSE ( -- )
    0 REG-LASER-SHUTTER HW!
    REG-LASER-STATUS HW@
    LASER-STAT-SHUTTER-OPEN INVERT AND
    REG-LASER-STATUS HW!
;

\ LASER-SET-POWER ( mW -- )
\ Regulate output optical power via EDFA 980nm pump current control.
: LASER-SET-POWER ( mW -- )
    0 LASER-MAX-POWER-MW CLAMP
    REG-LASER-TX-POWER HW!
;

\ LASER-EMITTING? ( -- flag )
\ Return TRUE if optical emission is active and shutter is open.
: LASER-EMITTING? ( -- flag )
    LASER-STATUS@
    LASER-STAT-EMISSION-ACTIVE LASER-STAT-SHUTTER-OPEN OR AND
    0<>
;

\ LASER-TEMP@ ( -- mK )
: LASER-TEMP@ ( -- mK )
    REG-LASER-TEMP HW@
;

\ LASER-SET-SHUTTER ( open_flag -- )
: LASER-SET-SHUTTER ( flag -- )
    IF LASER-SHUTTER-OPEN ELSE LASER-SHUTTER-CLOSE THEN
;

\ LASER-SET-PUMP-CURRENT ( mA -- )
: LASER-SET-PUMP-CURRENT ( mA -- )
    REG-LASER-PUMP-CUR HW!
;
