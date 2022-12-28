\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kernel/executive.fs
\ Description: Flight software rate-monotonic real-time executive cadence loop.
\              Coordinates 1 kHz fast servo, 100 Hz gimbal trajectory, 10 Hz
\              PAT acquisition FSM, and 1 Hz health & thermal supervisory tasks.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Cadence Timers and Execution Counters
\ ------------------------------------------------------------------------------

VARIABLE EXEC-CYCLE-COUNT
VARIABLE EXEC-SUB-100HZ-COUNT
VARIABLE EXEC-SUB-10HZ-COUNT
VARIABLE EXEC-SUB-1HZ-COUNT
VARIABLE EXEC-RUNNING

\ Task IDs
VARIABLE TASK-ID-FAST-SERVO
VARIABLE TASK-ID-GIMBAL-TRAJ
VARIABLE TASK-ID-PAT-FSM
VARIABLE TASK-ID-SUPERVISOR

\ ------------------------------------------------------------------------------
\ Section 2: Task Bodies for Multi-Rate Execution
\ ------------------------------------------------------------------------------

\ TASK-BODY-1KHZ ( -- )
\ 1 kHz (1 ms) Cadence: FSM closed-loop servo, lead-lag filter, and offload.
: TASK-BODY-1KHZ ( -- )
    TASK-BIT-SERVO WD-HEARTBEAT
    PAT-CURRENT-STATE @ PAT-STATE-TRACK =
    PAT-CURRENT-STATE @ PAT-STATE-COMM-LOCK = OR IF
        QPD-SAMPLE
        QPD-GET-CENTROID
        TRACK-STEP-DUAL-STAGE
    THEN
;

\ TASK-BODY-100HZ ( -- )
\ 100 Hz (10 ms) Cadence: CPA gimbal trajectory profiler and LuGre friction.
: TASK-BODY-100HZ ( -- )
    PAT-CURRENT-STATE @ PAT-STATE-SLEW = IF
        10000 CPA-TRAJ-STEP DROP
        \ Command new position to servo
        0.01 S>Q32 CPA-SERVO-STEP
    THEN
;

\ TASK-BODY-10HZ ( -- )
\ 10 Hz (100 ms) Cadence: Master PAT finite state machine and orbital geometry.
: TASK-BODY-10HZ ( -- )
    TASK-BIT-PAT WD-HEARTBEAT
    100 PAT-STEP
;

\ TASK-BODY-1HZ ( -- )
\ 1 Hz (1000 ms) Cadence: Thermal supervisor, housekeeping TM, and watchdog.
: TASK-BODY-1HZ ( -- )
    TASK-BIT-TELEMETRY WD-HEARTBEAT
    THERM-SUPERVISE DROP
    SAFETY-CHECK-LEAKAGE DROP
    SAFETY-CHECK-SUN-BLIND DROP
    TM-GENERATE-SPACE-PACKET 2DROP
    1000 WD-SUPERVISE-STEP DROP
;

\ ------------------------------------------------------------------------------
\ Section 3: Executive Initialization and Stepping
\ ------------------------------------------------------------------------------

\ EXEC-INIT ( -- )
\ Initialize all flight software subsystems, task tables, and spawn threads.
: EXEC-INIT ( -- )
    0 EXEC-CYCLE-COUNT !
    0 EXEC-SUB-100HZ-COUNT !
    0 EXEC-SUB-10HZ-COUNT !
    0 EXEC-SUB-1HZ-COUNT !
    1 EXEC-RUNNING !

    TASK-INIT-KERNEL
    EVENT-INIT
    MSG-QUEUE-INIT
    FAULT-INIT
    TM-INIT
    PAT-FSM-INIT
    BER-INIT
    TF-INIT
    RS-CODEC-INIT
    PPM-INIT

    \ Spawn multi-rate tasks: ( xt period_ms -- id )
    ['] TASK-BODY-1KHZ   1    TASK-SPAWN TASK-ID-FAST-SERVO !
    ['] TASK-BODY-100HZ  10   TASK-SPAWN TASK-ID-GIMBAL-TRAJ !
    ['] TASK-BODY-10HZ   100  TASK-SPAWN TASK-ID-PAT-FSM !
    ['] TASK-BODY-1HZ    1000 TASK-SPAWN TASK-ID-SUPERVISOR !
;

\ EXEC-STEP-1MS ( -- )
\ Execute one 1-millisecond real-time flight software cadence slice.
: EXEC-STEP-1MS ( -- )
    EXEC-RUNNING @ 0= IF EXIT THEN

    1 EXEC-CYCLE-COUNT +!
    TASK-STEP
    1000 HW-STEP
;

\ EXEC-RUN ( n_ms -- )
\ Run flight software executive for specified number of milliseconds.
: EXEC-RUN ( n -- )
    DUP 0<= IF DROP EXIT THEN
    0 ?DO
        EXEC-STEP-1MS
    LOOP
;
