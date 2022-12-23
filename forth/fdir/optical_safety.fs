\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/fdir/optical_safety.fs
\ Description: Laser emission hardware interlocks, sun-sensor blind trip, optical
\              leakage monitor, and autonomous safety shutter trip.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Safety Interlock State
\ ------------------------------------------------------------------------------

VARIABLE SAFETY-INTERLOCK-ENGAGED
VARIABLE SAFETY-SUN-BLIND-TRIP
VARIABLE SAFETY-LEAKAGE-FAULT
VARIABLE SAFETY-EMERGENCY-STOP

\ Hardware Thresholds
10 CONSTANT SAFETY-MAX-LEAKAGE-MW   \ Max allowable optical leakage with shutter closed (10 mW)

\ ------------------------------------------------------------------------------
\ Section 2: Safety Verification API
\ ------------------------------------------------------------------------------

\ SAFETY-INIT ( -- )
: SAFETY-INIT ( -- )
    0 SAFETY-INTERLOCK-ENGAGED !
    0 SAFETY-SUN-BLIND-TRIP !
    0 SAFETY-LEAKAGE-FAULT !
    0 SAFETY-EMERGENCY-STOP !
;

\ SAFETY-ASSERT-ESTOP ( -- )
\ Instantly disable all optical emission, close shutter, and cut EDFA pump rails.
: SAFETY-ASSERT-ESTOP ( -- )
    1 SAFETY-EMERGENCY-STOP !
    0 LASER-SET-SHUTTER
    0 LASER-SET-POWER
    0 LASER-SET-PUMP-CURRENT
    LASER-DISARM
;

\ SAFETY-CHECK-LEAKAGE ( -- leakage_fault_flag )
\ Verify that optical power drops below 10 mW when shutter is closed.
: SAFETY-CHECK-LEAKAGE ( -- flag )
    REG-LASER-SHUTTER HW@ 0= IF
        \ Shutter commanded closed: measure output power
        REG-LASER-TX-POWER HW@
        SAFETY-MAX-LEAKAGE-MW > IF
            1 SAFETY-LEAKAGE-FAULT !
            SAFETY-ASSERT-ESTOP
            1 EXIT
        THEN
    THEN
    0 SAFETY-LEAKAGE-FAULT !
    0
;

\ SAFETY-CHECK-SUN-BLIND ( -- sun_trip_flag )
\ Check if quadrant detector ambient light exceeds damage threshold.
: SAFETY-CHECK-SUN-BLIND ( -- flag )
    QPD-AMBIENT@ 50000 > IF
        1 SAFETY-SUN-BLIND-TRIP !
        SAFETY-ASSERT-ESTOP
        1 EXIT
    THEN
    0 SAFETY-SUN-BLIND-TRIP !
    0
;

\ SAFETY-PERMIT-EMISSION? ( -- permit_flag )
\ Return TRUE only if all safety interlocks, temperatures, and keep-outs are clear.
: SAFETY-PERMIT-EMISSION? ( -- flag )
    SAFETY-EMERGENCY-STOP @ 0=
    SAFETY-SUN-BLIND-TRIP @ 0= AND
    SAFETY-LEAKAGE-FAULT @ 0= AND
    LASER-INTERLOCK-OK? AND
;
