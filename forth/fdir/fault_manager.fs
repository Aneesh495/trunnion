\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/fdir/fault_manager.fs
\ Description: Master Fault Detection, Isolation, and Recovery (FDIR) executive,
\              severity classifier, circular anomaly log, and safe-hold failover.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Fault Codes and Severity Levels
\ ------------------------------------------------------------------------------

0 CONSTANT SEV-INFO
1 CONSTANT SEV-WARN
2 CONSTANT SEV-ERROR
3 CONSTANT SEV-CRITICAL

\ Standard Avionics Fault Codes
$1001 CONSTANT FAULT-WATCHDOG-STALL
$1002 CONSTANT FAULT-LASER-OVERTEMP
$1003 CONSTANT FAULT-MOTOR-OVERTEMP
$1004 CONSTANT FAULT-SUN-BLIND-TRIP
$1005 CONSTANT FAULT-OPTICAL-LEAKAGE
$1006 CONSTANT FAULT-LOSS-OF-LOCK
$1007 CONSTANT FAULT-FRAME-SYNC-LOST
$1008 CONSTANT FAULT-RS-UNCORRECTABLE

\ Circular Fault Log (64 entries, 4 cells per entry: time_lo, time_hi, code, sev)
64 CONSTANT FAULT-LOG-MAX-ENTRIES
4  CONSTANT FAULT-ENTRY-CELLS
CREATE FAULT-LOG-BUFFER FAULT-LOG-MAX-ENTRIES FAULT-ENTRY-CELLS CELLS * ALLOT

VARIABLE FAULT-LOG-HEAD
VARIABLE FAULT-LOG-COUNT
VARIABLE FAULT-ACTIVE-SEVERITY

\ ------------------------------------------------------------------------------
\ Section 2: Fault Logging API
\ ------------------------------------------------------------------------------

\ FAULT-INIT ( -- )
: FAULT-INIT ( -- )
    0 FAULT-LOG-HEAD !
    0 FAULT-LOG-COUNT !
    SEV-INFO FAULT-ACTIVE-SEVERITY !
    FAULT-LOG-BUFFER FAULT-LOG-MAX-ENTRIES FAULT-ENTRY-CELLS CELLS * ERASE
    WD-INIT
    HEATER-PID-INIT
    SAFETY-INIT
;

\ FAULT-POST ( fault_code severity -- )
\ Record anomaly into circular log and trigger autonomous recovery if critical.
: FAULT-POST ( code sev -- )
    SWAP OVER >R >R \ Stash sev and code
    FAULT-ACTIVE-SEVERITY @ R@ MAX FAULT-ACTIVE-SEVERITY !

    \ Compute entry address in circular buffer
    FAULT-LOG-HEAD @ FAULT-ENTRY-CELLS CELLS * FAULT-LOG-BUFFER + >R

    \ Store timestamp
    HW-CLOCK
    R@ 1 CELLS + !
    R@ 0 CELLS + !

    \ Store code and severity
    R> R> R> ROT
    DUP R@ 2 CELLS + !
    R@ 3 CELLS + !
    DROP R> DROP

    \ Advance circular head index
    FAULT-LOG-HEAD @ 1+ FAULT-LOG-MAX-ENTRIES MOD FAULT-LOG-HEAD !
    FAULT-LOG-COUNT @ 1+ FAULT-LOG-MAX-ENTRIES MIN FAULT-LOG-COUNT !

    \ If severity is CRITICAL, initiate emergency safe-hold transition
    FAULT-ACTIVE-SEVERITY @ SEV-CRITICAL >= IF
        SAFETY-ASSERT-ESTOP
        FAULT-LASER-OVERTEMP PAT-ENTER-SAFE-HOLD
    THEN
;

\ FAULT-CLEAR ( -- )
\ Clear active severity status after operator intervention or autonomous recovery.
: FAULT-CLEAR ( -- )
    SEV-INFO FAULT-ACTIVE-SEVERITY !
;

\ FAULT-PRINT-LOG ( -- )
: FAULT-PRINT-LOG ( -- )
    CR ." [FDIR AVIONICS ANOMALY LOG]" CR
    ."   Active Severity: "
    FAULT-ACTIVE-SEVERITY @ CASE
        SEV-INFO     OF ." NOMINAL (INFO)"     ENDOF
        SEV-WARN     OF ." WARNING"            ENDOF
        SEV-ERROR    OF ." DEGRADED (ERROR)"   ENDOF
        SEV-CRITICAL OF ." CRITICAL SAFE-HOLD" ENDOF
        ." UNKNOWN"
    ENDCASE CR
    ."   Logged Events:   " FAULT-LOG-COUNT @ . CR
;
