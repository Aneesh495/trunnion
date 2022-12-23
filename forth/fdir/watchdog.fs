\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/fdir/watchdog.fs
\ Description: Avionics hardware watchdog supervisor, task heartbeat monitor,
\              thread stall detector, and safe-hold reset trigger.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Watchdog State and Heartbeat Bitmasks
\ ------------------------------------------------------------------------------

VARIABLE WD-TIMEOUT-MS          \ Hardware watchdog countdown period (nominal 1000 ms)
VARIABLE WD-ELAPSED-MS          \ Elapsed milliseconds since last supervisor feed
VARIABLE WD-HEARTBEAT-MASK      \ Bitmask of tasks that have reported alive in this cycle
VARIABLE WD-EXPECTED-MASK       \ Bitmask of required alive tasks (e.g. 0x0F for 4 tasks)
VARIABLE WD-STALL-COUNT         \ Cumulative thread stall anomaly count
VARIABLE WD-RESET-COUNT         \ Cumulative software reset count

\ Task Heartbeat Bits
1 0 LSHIFT CONSTANT TASK-BIT-SERVO       \ 1 kHz FSM & CPA fast servo
1 1 LSHIFT CONSTANT TASK-BIT-PAT         \ 10 Hz acquisition & tracking state machine
1 2 LSHIFT CONSTANT TASK-BIT-COMM        \ Optical modem and CCSDS frame handler
1 3 LSHIFT CONSTANT TASK-BIT-TELEMETRY   \ 1 Hz health monitor & housekeeping

\ ------------------------------------------------------------------------------
\ Section 2: Watchdog Supervisor API
\ ------------------------------------------------------------------------------

\ WD-INIT ( -- )
\ Initialize watchdog monitor with required 4-task heartbeat mask.
: WD-INIT ( -- )
    1000 WD-TIMEOUT-MS !
    0 WD-ELAPSED-MS !
    0 WD-HEARTBEAT-MASK !
    TASK-BIT-SERVO TASK-BIT-PAT OR
    TASK-BIT-COMM OR TASK-BIT-TELEMETRY OR
    WD-EXPECTED-MASK !
    0 WD-STALL-COUNT !
    0 WD-RESET-COUNT !
;

\ WD-HEARTBEAT ( task_bit -- )
\ Assert alive status for calling task.
: WD-HEARTBEAT ( task_bit -- )
    WD-HEARTBEAT-MASK @ OR
    WD-HEARTBEAT-MASK !
;

\ WD-SUPERVISE-STEP ( dt_ms -- status_code )
\ Advance watchdog countdown. If all tasks reported, feed hardware register.
\ Returns:
\   0 = Normal (all heartbeats present, hardware watchdog serviced)
\   1 = Pending (waiting for remaining task heartbeats)
\   2 = Stall detected (one or more threads failed to report within timeout)
: WD-SUPERVISE-STEP ( dt -- status )
    DUP WD-ELAPSED-MS +!

    \ Check if all required tasks reported
    WD-HEARTBEAT-MASK @ WD-EXPECTED-MASK @ AND
    WD-EXPECTED-MASK @ = IF
        \ Service physical hardware watchdog
        WD-FEED!
        0 WD-ELAPSED-MS !
        0 WD-HEARTBEAT-MASK ! \ Reset mask for next cycle
        0 EXIT
    THEN

    \ Check for timeout violation
    WD-ELAPSED-MS @ WD-TIMEOUT-MS @ >= IF
        1 WD-STALL-COUNT +!
        0 WD-ELAPSED-MS !
        0 WD-HEARTBEAT-MASK !
        2 EXIT
    THEN

    1
;

\ WD-PRINT-STATUS ( -- )
: WD-PRINT-STATUS ( -- )
    CR ." [AVIONICS WATCHDOG SUPERVISOR]" CR
    ."   Elapsed Time:   " WD-ELAPSED-MS @ . ." ms" CR
    ."   Heartbeat Mask: $" WD-HEARTBEAT-MASK @ BASE @ >R HEX . R> BASE ! CR
    ."   Expected Mask:  $" WD-EXPECTED-MASK @ BASE @ >R HEX . R> BASE ! CR
    ."   Stall Events:   " WD-STALL-COUNT @ . CR
;
