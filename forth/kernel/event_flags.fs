\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kernel/event_flags.fs
\ Description: 32-bit inter-task event flag synchronization primitives for
\              payload flight software coordination.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Event Flag Storage
\ ------------------------------------------------------------------------------

VARIABLE KERNEL-EVENT-FLAGS

\ Standard Avionics System Event Masks
1 0 LSHIFT CONSTANT EVT-FRAME-SYNC-LOCKED
1 1 LSHIFT CONSTANT EVT-BEACON-DETECTED
1 2 LSHIFT CONSTANT EVT-TRACK-IN-SPEC
1 3 LSHIFT CONSTANT EVT-EPHEMERIS-UPDATED
1 4 LSHIFT CONSTANT EVT-TELECOMMAND-RECEIVED
1 5 LSHIFT CONSTANT EVT-THERMAL-WARNING
1 6 LSHIFT CONSTANT EVT-SAFE-MODE-ENTERED

\ ------------------------------------------------------------------------------
\ Section 2: Event Flag API
\ ------------------------------------------------------------------------------

\ EVENT-INIT ( -- )
: EVENT-INIT ( -- )
    0 KERNEL-EVENT-FLAGS !
;

\ POST-EVENT ( mask -- )
\ Atomically set event flag bits.
: POST-EVENT ( mask -- )
    KERNEL-EVENT-FLAGS @ OR
    KERNEL-EVENT-FLAGS !
;

\ CLEAR-EVENT ( mask -- )
\ Atomically clear event flag bits.
: CLEAR-EVENT ( mask -- )
    INVERT
    KERNEL-EVENT-FLAGS @ AND
    KERNEL-EVENT-FLAGS !
;

\ CHECK-EVENT ( mask -- flag )
\ Return TRUE if all bits in mask are currently asserted.
: CHECK-EVENT ( mask -- flag )
    DUP KERNEL-EVENT-FLAGS @ AND =
;

\ WAIT-EVENT ( mask -- )
\ Poll until specified event condition is met.
: WAIT-EVENT ( mask -- )
    BEGIN
        DUP CHECK-EVENT 0=
    WHILE
        YIELD
    REPEAT
    DROP
;
