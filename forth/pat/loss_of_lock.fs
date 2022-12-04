\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/loss_of_lock.fs
\ Description: Anomaly detection, fading persistence counter, loss-of-lock (LOL)
\              mitigation, safety shutter triggering, and re-acquisition logic.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Loss-of-Lock State and Threshold Constants
\ ------------------------------------------------------------------------------

VARIABLE LOL-STATE              \ 0: Nominal, 1: Fading, 2: Lost Lock, 3: Re-acquiring
VARIABLE LOL-FADE-COUNTER-MS    \ Elapsed fade duration in milliseconds
VARIABLE LOL-TOTAL-EVENTS       \ Cumulative loss of lock event count
VARIABLE LOL-SHUTTER-AUTO-CLOSE \ Boolean: automatically close shutter on LOL

\ Thresholds
50 CONSTANT LOL-FADE-TIMEOUT-MS     \ Maximum tolerated fade duration before LOL
1000 CONSTANT LOL-MAX-REACQ-TIME-MS \ Max time in re-acquisition before fallback to search

\ Last Known Good (LKG) Gimbal Coordinates (Q32.32 radians)
VARIABLE LOL-LKG-AZ
VARIABLE LOL-LKG-EL

\ ------------------------------------------------------------------------------
\ Section 2: Initialization and Reset
\ ------------------------------------------------------------------------------

\ LOL-INIT ( -- )
\ Initialize loss of lock detector and timers.
: LOL-INIT ( -- )
    0 LOL-STATE !
    0 LOL-FADE-COUNTER-MS !
    0 LOL-TOTAL-EVENTS !
    1 LOL-SHUTTER-AUTO-CLOSE !
    0 LOL-LKG-AZ !
    0 LOL-LKG-EL !
;

\ LOL-RECORD-LKG ( az_q32 el_q32 -- )
\ Store last known good pointing angles for rapid re-acquisition.
: LOL-RECORD-LKG ( az_q32 el_q32 -- )
    LOL-LKG-EL !
    LOL-LKG-AZ !
;

\ ------------------------------------------------------------------------------
\ Section 3: Periodic Fault Evaluation
\ ------------------------------------------------------------------------------

\ LOL-EVALUATE ( valid_snr? valid_centroid? dt_ms -- event_code )
\ Evaluate optical telemetry for signal loss or beam walk-off.
\ Returns event code:
\   0 = Nominal tracking maintained
\   1 = Temporary optical fade in progress
\   2 = Loss of lock declared, safety action triggered
\   3 = Link recovered
: LOL-EVALUATE ( valid_snr? valid_centroid? dt_ms -- event_code )
    >R
    AND IF
        \ Both SNR and centroid valid
        R> DROP
        LOL-STATE @ 1 = IF
            \ Recovered from fade
            0 LOL-STATE !
            0 LOL-FADE-COUNTER-MS !
            3 EXIT
        THEN
        0 LOL-STATE !
        0 LOL-FADE-COUNTER-MS !
        0 EXIT
    THEN

    \ Signal invalid: advance fade counter
    R> LOL-FADE-COUNTER-MS +!

    LOL-STATE @ 0= IF
        \ Transition from Nominal to Fading
        1 LOL-STATE !
    THEN

    LOL-FADE-COUNTER-MS @ LOL-FADE-TIMEOUT-MS >= IF
        \ Timeout exceeded: declare loss of lock
        LOL-STATE @ 2 <> IF
            2 LOL-STATE !
            1 LOL-TOTAL-EVENTS +!

            \ Trigger safety beam shutter if configured
            LOL-SHUTTER-AUTO-CLOSE @ IF
                0 LASER-SET-SHUTTER
            THEN
            2 EXIT
        THEN
    THEN

    1
;

\ ------------------------------------------------------------------------------
\ Section 4: Re-Acquisition Trigger
\ ------------------------------------------------------------------------------

\ LOL-TRIGGER-REACQ ( -- az_q32 el_q32 )
\ Set state to Re-acquiring and return last known good pointing coordinates.
: LOL-TRIGGER-REACQ ( -- az_q32 el_q32 )
    3 LOL-STATE !
    0 LOL-FADE-COUNTER-MS !
    LOL-LKG-AZ @ LOL-LKG-EL @
;

\ LOL-STATUS-PRINT ( -- )
\ Print diagnostic status of loss of lock manager.
: LOL-STATUS-PRINT ( -- )
    CR ." [LOSS-OF-LOCK MONITOR STATUS]" CR
    ."   Current State: "
    LOL-STATE @ CASE
        0 OF ." NOMINAL" ENDOF
        1 OF ." FADING" ENDOF
        2 OF ." LOST LOCK" ENDOF
        3 OF ." RE-ACQUIRING" ENDOF
        ." UNKNOWN"
    ENDCASE CR
    ."   Fade Time:     " LOL-FADE-COUNTER-MS @ . ." ms" CR
    ."   Total Events:  " LOL-TOTAL-EVENTS @ . CR
;
