\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/acquisition_fsm.fs
\ Description: Optical beacon acquisition detector, M-of-N temporal confirmation
\              filter, and master-slave spatial handshake coordinator.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Temporal Filter Parameters:
\ Confirmation window: N = 5 consecutive frames
\ Required hits: M = 4 detections above power threshold
5 CONSTANT ACQ-WINDOW-SIZE
4 CONSTANT ACQ-REQUIRED-HITS

\ Optical beacon detection threshold: 1200 ADC counts above ambient background
1200 CONSTANT ACQ-BEACON-THRESHOLD-COUNTS

VARIABLE ACQ-HIT-HISTORY
VARIABLE ACQ-CONSECUTIVE-LOCKS
VARIABLE ACQ-STATE-FLAG

\ ------------------------------------------------------------------------------
\ Section 1: Acquisition Filter Engine
\ ------------------------------------------------------------------------------

\ ACQ-RESET ( -- )
: ACQ-RESET ( -- )
    0 ACQ-HIT-HISTORY !
    0 ACQ-CONSECUTIVE-LOCKS !
    FALSE ACQ-STATE-FLAG !
;

\ ACQ-COUNT-HITS ( -- count )
\ Count number of set bits in 5-frame sliding window history.
: ACQ-COUNT-HITS ( -- count )
    ACQ-HIT-HISTORY @ $1F AND POPCOUNT
;

\ ACQ-PROCESS-FRAME ( raw_sum ambient -- acquired_flag )
\ Evaluate incoming optical frame against detection criteria.
: ACQ-PROCESS-FRAME ( sum ambient -- flag )
    - \ Net signal power above background
    ACQ-BEACON-THRESHOLD-COUNTS >= IF
        \ Shift in a 1-bit
        ACQ-HIT-HISTORY @ 1 LSHIFT 1 OR $1F AND
        ACQ-HIT-HISTORY !
    ELSE
        \ Shift in a 0-bit
        ACQ-HIT-HISTORY @ 1 LSHIFT $1F AND
        ACQ-HIT-HISTORY !
    THEN

    \ Check M-of-N condition
    ACQ-COUNT-HITS ACQ-REQUIRED-HITS >= IF
        1 ACQ-CONSECUTIVE-LOCKS +!
        ACQ-CONSECUTIVE-LOCKS @ 3 >= IF
            TRUE ACQ-STATE-FLAG !
            TRUE EXIT
        THEN
    ELSE
        0 ACQ-CONSECUTIVE-LOCKS !
        FALSE ACQ-STATE-FLAG !
    THEN
    FALSE
;

\ ACQ-CONFIRMED? ( -- flag )
: ACQ-CONFIRMED? ( -- flag )
    ACQ-STATE-FLAG @
;

\ ------------------------------------------------------------------------------
\ Section 3: High-Level State Management
\ ------------------------------------------------------------------------------

\ ACQ-FSM-RESET ( -- )
: ACQ-FSM-RESET ( -- )
    ACQ-RESET
;

\ ACQ-FSM-UPDATE ( hit_flag -- )
: ACQ-FSM-UPDATE ( hit -- )
    IF
        ACQ-HIT-HISTORY @ 1 LSHIFT 1 OR $1F AND ACQ-HIT-HISTORY !
        ACQ-COUNT-HITS ACQ-REQUIRED-HITS >= IF
            1 ACQ-CONSECUTIVE-LOCKS +!
            ACQ-CONSECUTIVE-LOCKS @ 3 >= IF
                TRUE ACQ-STATE-FLAG !
            THEN
        THEN
    ELSE
        ACQ-HIT-HISTORY @ 1 LSHIFT $1F AND ACQ-HIT-HISTORY !
        0 ACQ-CONSECUTIVE-LOCKS !
        FALSE ACQ-STATE-FLAG !
    THEN
;

\ ACQ-FSM-ACQUIRED? ( -- flag )
: ACQ-FSM-ACQUIRED? ( -- flag )
    ACQ-CONFIRMED?
;

\ ACQ-FSM-LOST? ( -- flag )
: ACQ-FSM-LOST? ( -- flag )
    ACQ-HIT-HISTORY @ 0=
;
