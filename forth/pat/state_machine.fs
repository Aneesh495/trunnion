\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/state_machine.fs
\ Description: Master Hierarchical Finite State Machine for Pointing, Acquisition,
\              and Tracking (PAT) mission phases.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: PAT State Enumeration and Context
\ ------------------------------------------------------------------------------

0 CONSTANT PAT-STATE-STANDBY
1 CONSTANT PAT-STATE-SLEW
2 CONSTANT PAT-STATE-SEARCH
3 CONSTANT PAT-STATE-ACQUIRE
4 CONSTANT PAT-STATE-TRACK
5 CONSTANT PAT-STATE-COMM-LOCK
6 CONSTANT PAT-STATE-SAFE-HOLD

VARIABLE PAT-CURRENT-STATE
VARIABLE PAT-PREV-STATE
VARIABLE PAT-STATE-TIME-MS       \ Time spent in current state (milliseconds)
VARIABLE PAT-FAULT-CODE

\ Target Ephemeris Coordinates for Slew (Q32.32 radians)
VARIABLE PAT-TARGET-AZ
VARIABLE PAT-TARGET-EL

\ ------------------------------------------------------------------------------
\ Section 2: State Machine Lifecycle and Initialization
\ ------------------------------------------------------------------------------

\ PAT-TRANSITION ( new_state -- )
\ Execute state transition with logging and timer reset.
: PAT-TRANSITION ( new_state -- )
    PAT-CURRENT-STATE @ PAT-PREV-STATE !
    PAT-CURRENT-STATE !
    0 PAT-STATE-TIME-MS !
;

\ PAT-FSM-INIT ( -- )
\ Initialize all PAT subsystems and transition to STANDBY.
: PAT-FSM-INIT ( -- )
    PAT-STATE-STANDBY PAT-CURRENT-STATE !
    PAT-STATE-STANDBY PAT-PREV-STATE !
    0 PAT-STATE-TIME-MS !
    0 PAT-FAULT-CODE !
    0 PAT-TARGET-AZ !
    0 PAT-TARGET-EL !

    CPA-SERVO-INIT
    FSM-SERVO-RESET
    QPD-CENTROID-INIT
    SPIRAL-INIT-DEFAULT
    LISSAJOUS-INIT-DEFAULT
    PAA-INIT
    TRACK-LOOP-INIT
    LOL-INIT
    ACQ-FSM-RESET
;

\ ------------------------------------------------------------------------------
\ Section 3: State Action Handlers
\ ------------------------------------------------------------------------------

\ PAT-ENTER-STANDBY ( -- )
: PAT-ENTER-STANDBY ( -- )
    TRACK-LOOP-DISABLE
    0 LASER-SET-SHUTTER
    0 LASER-SET-PUMP-CURRENT
    CPA-SERVO-DISABLE
    PAT-STATE-STANDBY PAT-TRANSITION
;

\ PAT-ENTER-SLEW ( target_az_q32 target_el_q32 -- )
: PAT-ENTER-SLEW ( target_az_q32 target_el_q32 -- )
    PAT-TARGET-EL !
    PAT-TARGET-AZ !
    TRACK-LOOP-DISABLE
    0 LASER-SET-SHUTTER
    CPA-SERVO-ENABLE
    \ Plan S-curve trajectory to target
    CPA-READ-ENCODERS DROP
    PAT-TARGET-AZ @
    CPA-TRAJ-PLAN-DEFAULT
    PAT-STATE-SLEW PAT-TRANSITION
;

\ PAT-ENTER-SEARCH ( -- )
: PAT-ENTER-SEARCH ( -- )
    TRACK-LOOP-DISABLE
    SPIRAL-START
    \ Open laser shutter at beacon power
    500 LASER-SET-PUMP-CURRENT
    1 LASER-SET-SHUTTER
    PAT-STATE-SEARCH PAT-TRANSITION
;

\ PAT-ENTER-ACQUIRE ( -- )
: PAT-ENTER-ACQUIRE ( -- )
    ACQ-FSM-RESET
    PAT-STATE-ACQUIRE PAT-TRANSITION
;

\ PAT-ENTER-TRACK ( -- )
: PAT-ENTER-TRACK ( -- )
    TRACK-LOOP-ENABLE
    1000 LASER-SET-PUMP-CURRENT
    1 LASER-SET-SHUTTER
    PAT-STATE-TRACK PAT-TRANSITION
;

\ PAT-ENTER-COMM-LOCK ( -- )
: PAT-ENTER-COMM-LOCK ( -- )
    1500 LASER-SET-PUMP-CURRENT
    1 LASER-SET-SHUTTER
    PAT-STATE-COMM-LOCK PAT-TRANSITION
;

\ PAT-ENTER-SAFE-HOLD ( fault_code -- )
: PAT-ENTER-SAFE-HOLD ( fault_code -- )
    PAT-FAULT-CODE !
    TRACK-LOOP-DISABLE
    0 LASER-SET-SHUTTER
    0 LASER-SET-PUMP-CURRENT
    0 0 FSM-SET-COMMAND
    0 0 CPA-SET-CMD-RATE
    CPA-SERVO-DISABLE
    PAT-STATE-SAFE-HOLD PAT-TRANSITION
;

\ ------------------------------------------------------------------------------
\ Section 4: Master Execution Step
\ ------------------------------------------------------------------------------

\ PAT-STEP ( dt_ms -- )
\ Step the master finite state machine by dt_ms milliseconds.
: PAT-STEP ( dt_ms -- )
    DUP PAT-STATE-TIME-MS +!

    PAT-CURRENT-STATE @ CASE
        PAT-STATE-STANDBY OF
            DROP
        ENDOF

        PAT-STATE-SLEW OF
            DROP
            \ Check if slew trajectory is completed
            CPA-TRAJ-COMPLETED? IF
                PAT-ENTER-SEARCH
            THEN
        ENDOF

        PAT-STATE-SEARCH OF
            \ Execute spiral step
            SPIRAL-STEP
            \ Check for beacon detection on QPD
            QPD-SAMPLE
            QPD-SIGNAL-VALID? IF
                PAT-ENTER-ACQUIRE
            THEN
            DROP
        ENDOF

        PAT-STATE-ACQUIRE OF
            DROP
            \ Sample optical centroid and evaluate acquisition filter
            QPD-SAMPLE
            QPD-SIGNAL-VALID? IF
                1 ACQ-FSM-UPDATE
                ACQ-FSM-ACQUIRED? IF
                    PAT-ENTER-TRACK
                THEN
            ELSE
                0 ACQ-FSM-UPDATE
                ACQ-FSM-LOST? IF
                    PAT-ENTER-SEARCH
                THEN
            THEN
        ENDOF

        PAT-STATE-TRACK OF
            DROP
            \ Execute dual-stage closed-loop tracking
            QPD-SAMPLE
            QPD-GET-CENTROID
            TRACK-STEP-DUAL-STAGE

            \ Update Last Known Good coordinates
            CPA-READ-ENCODERS LOL-RECORD-LKG

            \ Check for communication lock condition
            TRACK-IN-SPEC? IF
                PAT-ENTER-COMM-LOCK
            THEN
        ENDOF

        PAT-STATE-COMM-LOCK OF
            DROP
            \ Maintain tight tracking loop
            QPD-SAMPLE
            QPD-GET-CENTROID
            TRACK-STEP-DUAL-STAGE

            \ Monitor for loss of lock
            QPD-SIGNAL-VALID? 1 1 LOL-EVALUATE
            DUP 2 = IF
                \ Loss of lock: revert to search
                DROP PAT-ENTER-SEARCH
            ELSE
                DROP
            THEN
        ENDOF

        PAT-STATE-SAFE-HOLD OF
            DROP
        ENDOF
    ENDCASE
;

\ ------------------------------------------------------------------------------
\ Section 5: Telemetry and Inspection
\ ------------------------------------------------------------------------------

\ PAT-PRINT-STATE ( -- )
\ Print the human-readable name of the current PAT state.
: PAT-PRINT-STATE ( -- )
    PAT-CURRENT-STATE @ CASE
        PAT-STATE-STANDBY   OF ." STANDBY"   ENDOF
        PAT-STATE-SLEW      OF ." SLEW"      ENDOF
        PAT-STATE-SEARCH    OF ." SEARCH"    ENDOF
        PAT-STATE-ACQUIRE   OF ." ACQUIRE"   ENDOF
        PAT-STATE-TRACK     OF ." TRACK"     ENDOF
        PAT-STATE-COMM-LOCK OF ." COMM_LOCK" ENDOF
        PAT-STATE-SAFE-HOLD OF ." SAFE_HOLD" ENDOF
        ." UNKNOWN"
    ENDCASE
;
