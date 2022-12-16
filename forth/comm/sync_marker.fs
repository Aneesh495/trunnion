\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/sync_marker.fs
\ Description: Sliced Attached Synchronization Marker (SASM) 32-bit correlator,
\              Hamming distance sliding window, and 4-state frame synchronizer.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Sync Marker Constants and Synchronization States
\ ------------------------------------------------------------------------------

\ CCSDS Standard 32-bit Attached Sync Marker (ASM)
$1ACFFC1D CONSTANT CCSDS-ASM-32

0 CONSTANT SYNC-STATE-SEARCH
1 CONSTANT SYNC-STATE-CHECK
2 CONSTANT SYNC-STATE-LOCK
3 CONSTANT SYNC-STATE-FLYWHEEL

VARIABLE SYNC-CURRENT-STATE
VARIABLE SYNC-SHIFT-REG-32       \ 32-bit incoming serial shift register
VARIABLE SYNC-FRAME-BIT-COUNT    \ Bit counter within current frame
VARIABLE SYNC-FRAME-LEN-BITS     \ Total frame length (nominal 8920 bits = 1115 bytes)
VARIABLE SYNC-CONSEC-LOCKS       \ Number of consecutive valid markers
VARIABLE SYNC-FLYWHEEL-COUNT     \ Number of consecutive missed markers in flywheel
VARIABLE SYNC-BIT-ERRORS         \ Bit error count in last detected ASM

\ Tolerances
2 CONSTANT SYNC-SEARCH-MAX-ERRORS   \ Strict threshold during initial acquisition
4 CONSTANT SYNC-LOCK-MAX-ERRORS     \ Relaxed threshold in locked state
3 CONSTANT SYNC-MAX-FLYWHEELS       \ Maximum tolerated missed frames before drop

\ ------------------------------------------------------------------------------
\ Section 2: Initialization and Reset
\ ------------------------------------------------------------------------------

\ SYNC-INIT ( frame_len_bits -- )
\ Initialize frame synchronizer to SEARCH state.
: SYNC-INIT ( len -- )
    SYNC-FRAME-LEN-BITS !
    SYNC-STATE-SEARCH SYNC-CURRENT-STATE !
    0 SYNC-SHIFT-REG-32 !
    0 SYNC-FRAME-BIT-COUNT !
    0 SYNC-CONSEC-LOCKS !
    0 SYNC-FLYWHEEL-COUNT !
    0 SYNC-BIT-ERRORS !
;

\ ------------------------------------------------------------------------------
\ Section 3: Sliding Bit Correlator
\ ------------------------------------------------------------------------------

\ SYNC-COMPUTE-DISTANCE ( val32 -- dist )
\ Compute Hamming distance between val32 and CCSDS-ASM-32.
: SYNC-COMPUTE-DISTANCE ( val -- dist )
    CCSDS-ASM-32 HAMMING-DISTANCE
;

\ SYNC-FEED-BIT ( bit -- sync_event )
\ Push single bit into shift register and evaluate correlation against ASM.
\ Returns:
\   0 = No sync event (normal bit ingestion)
\   1 = Frame sync detected (start of frame payload)
\   2 = Frame sync lost (transitioned to SEARCH)
: SYNC-FEED-BIT ( bit -- event )
    \ Shift into 32-bit window
    SYNC-SHIFT-REG-32 @ 1 LSHIFT SWAP 1 AND OR
    $FFFFFFFF AND
    DUP SYNC-SHIFT-REG-32 !

    1 SYNC-FRAME-BIT-COUNT +!

    SYNC-CURRENT-STATE @ CASE
        SYNC-STATE-SEARCH OF
            \ Correlate with ASM
            SYNC-SHIFT-REG-32 @ SYNC-COMPUTE-DISTANCE
            DUP SYNC-BIT-ERRORS !
            SYNC-SEARCH-MAX-ERRORS <= IF
                SYNC-STATE-CHECK SYNC-CURRENT-STATE !
                0 SYNC-FRAME-BIT-COUNT !
                1 SYNC-CONSEC-LOCKS !
                0 EXIT
            THEN
            0 EXIT
        ENDOF

        SYNC-STATE-CHECK OF
            \ Check only when expected frame boundary is reached
            SYNC-FRAME-BIT-COUNT @ SYNC-FRAME-LEN-BITS @ >= IF
                SYNC-SHIFT-REG-32 @ SYNC-COMPUTE-DISTANCE
                DUP SYNC-BIT-ERRORS !
                SYNC-SEARCH-MAX-ERRORS <= IF
                    SYNC-STATE-LOCK SYNC-CURRENT-STATE !
                    0 SYNC-FRAME-BIT-COUNT !
                    1 SYNC-CONSEC-LOCKS +!
                    1 EXIT \ Frame lock confirmed!
                ELSE
                    SYNC-STATE-SEARCH SYNC-CURRENT-STATE !
                    0 SYNC-CONSEC-LOCKS !
                    0 EXIT
                THEN
            THEN
            0 EXIT
        ENDOF

        SYNC-STATE-LOCK OF
            SYNC-FRAME-BIT-COUNT @ SYNC-FRAME-LEN-BITS @ >= IF
                SYNC-SHIFT-REG-32 @ SYNC-COMPUTE-DISTANCE
                DUP SYNC-BIT-ERRORS !
                SYNC-LOCK-MAX-ERRORS <= IF
                    0 SYNC-FRAME-BIT-COUNT !
                    1 SYNC-CONSEC-LOCKS +!
                    0 SYNC-FLYWHEEL-COUNT !
                    1 EXIT \ Frame sync pulse!
                ELSE
                    \ Missed marker: enter flywheel
                    SYNC-STATE-FLYWHEEL SYNC-CURRENT-STATE !
                    1 SYNC-FLYWHEEL-COUNT !
                    0 SYNC-FRAME-BIT-COUNT !
                    1 EXIT \ Flywheel frame pulse
                THEN
            THEN
            0 EXIT
        ENDOF

        SYNC-STATE-FLYWHEEL OF
            SYNC-FRAME-BIT-COUNT @ SYNC-FRAME-LEN-BITS @ >= IF
                SYNC-SHIFT-REG-32 @ SYNC-COMPUTE-DISTANCE
                DUP SYNC-BIT-ERRORS !
                SYNC-LOCK-MAX-ERRORS <= IF
                    SYNC-STATE-LOCK SYNC-CURRENT-STATE !
                    0 SYNC-FRAME-BIT-COUNT !
                    0 SYNC-FLYWHEEL-COUNT !
                    1 EXIT \ Recovered lock!
                ELSE
                    1 SYNC-FLYWHEEL-COUNT +!
                    0 SYNC-FRAME-BIT-COUNT !
                    SYNC-FLYWHEEL-COUNT @ SYNC-MAX-FLYWHEELS > IF
                        SYNC-STATE-SEARCH SYNC-CURRENT-STATE !
                        0 SYNC-CONSEC-LOCKS !
                        2 EXIT \ Sync permanently lost
                    THEN
                    1 EXIT \ Continue flywheel
                THEN
            THEN
            0 EXIT
        ENDOF
    ENDCASE

    0
;

\ SYNC-LOCKED? ( -- flag )
: SYNC-LOCKED? ( -- flag )
    SYNC-CURRENT-STATE @ SYNC-STATE-LOCK =
    SYNC-CURRENT-STATE @ SYNC-STATE-FLYWHEEL = OR
;

\ SYNC-PRINT-STATUS ( -- )
: SYNC-PRINT-STATUS ( -- )
    CR ." [FRAME SYNCHRONIZER STATUS]" CR
    ."   Current State: "
    SYNC-CURRENT-STATE @ CASE
        SYNC-STATE-SEARCH   OF ." SEARCH"   ENDOF
        SYNC-STATE-CHECK    OF ." CHECK"    ENDOF
        SYNC-STATE-LOCK     OF ." LOCK"     ENDOF
        SYNC-STATE-FLYWHEEL OF ." FLYWHEEL" ENDOF
        ." UNKNOWN"
    ENDCASE CR
    ."   Bit Count:     " SYNC-FRAME-BIT-COUNT @ . CR
    ."   Consec Locks:  " SYNC-CONSEC-LOCKS @ . CR
    ."   Last Errors:   " SYNC-BIT-ERRORS @ . CR
;
