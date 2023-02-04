\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_kernel.fs
\ Description: Test Suite for Cooperative Scheduler, Events, Queues & Executive
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Real-Time Kernel, Cooperative Scheduler, Event Flags, and Executive Cadence" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Task Control Block Lifecycle and Scheduler
\ ------------------------------------------------------------------------------

VARIABLE TEST-TASK-RUN-COUNT-A
VARIABLE TEST-TASK-RUN-COUNT-B

: TEST-TASK-BODY-A ( -- )
    1 TEST-TASK-RUN-COUNT-A +!
;

: TEST-TASK-BODY-B ( -- )
    1 TEST-TASK-RUN-COUNT-B +!
;

: STEP-SCHED-10 ( -- )
    10 0 DO
        TASK-STEP
    LOOP
;

: SETUP-KERNEL-TEST-TASKS ( -- )
    \ Spawn Task A: period 1 ms (1 kHz)
    ['] TEST-TASK-BODY-A 1 TASK-SPAWN
    0 ASSERT-EQ \ task ID 0
    KERNEL-NUM-TASKS @ 1 ASSERT-EQ

    \ Spawn Task B: period 5 ms (200 Hz)
    ['] TEST-TASK-BODY-B 5 TASK-SPAWN
    1 ASSERT-EQ \ task ID 1
    KERNEL-NUM-TASKS @ 2 ASSERT-EQ
;

S" Kernel scheduler: task spawning and multi-rate execution" TEST-CASE-START
TASK-INIT-KERNEL
0 TEST-TASK-RUN-COUNT-A !
0 TEST-TASK-RUN-COUNT-B !
KERNEL-NUM-TASKS @ 0 ASSERT-EQ

SETUP-KERNEL-TEST-TASKS

\ Step scheduler 10 times (10 ms)
STEP-SCHED-10

\ Task A (period 1) runs every step: 10 executions
TEST-TASK-RUN-COUNT-A @ 10 ASSERT-EQ
\ Task B (period 5) runs every 5 steps: 2 executions
TEST-TASK-RUN-COUNT-B @ 2 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 2: Inter-Task Event Flags
\ ------------------------------------------------------------------------------

S" Event flags: post, check, and clear atomic bitmasks" TEST-CASE-START
EVENT-INIT
KERNEL-EVENT-FLAGS @ 0 ASSERT-EQ

\ Post event bits
EVT-BEACON-DETECTED EVT-TRACK-IN-SPEC OR POST-EVENT
EVT-BEACON-DETECTED CHECK-EVENT ASSERT-TRUE
EVT-TRACK-IN-SPEC CHECK-EVENT ASSERT-TRUE
EVT-BEACON-DETECTED EVT-TRACK-IN-SPEC OR CHECK-EVENT ASSERT-TRUE
EVT-FRAME-SYNC-LOCKED CHECK-EVENT ASSERT-FALSE

\ Clear beacon event bit
EVT-BEACON-DETECTED CLEAR-EVENT
EVT-BEACON-DETECTED CHECK-EVENT ASSERT-FALSE
EVT-TRACK-IN-SPEC CHECK-EVENT ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 3: Inter-Task Message Queue
\ ------------------------------------------------------------------------------

S" Message queue: FIFO enqueue and dequeue ordering" TEST-CASE-START
MSG-QUEUE-INIT
MSG-PENDING-COUNT 0 ASSERT-EQ

\ Post message 1: sender 10, type 1, payload $AABB
10 1 $AABB MSG-POST 1 ASSERT-EQ
MSG-PENDING-COUNT 1 ASSERT-EQ

\ Post message 2: sender 20, type 2, payload $CCDD
20 2 $CCDD MSG-POST 1 ASSERT-EQ
MSG-PENDING-COUNT 2 ASSERT-EQ

\ Receive message 1
MSG-RECEIVE
1 ASSERT-EQ      \ Valid flag
$AABB ASSERT-EQ  \ Payload
1 ASSERT-EQ      \ Type
10 ASSERT-EQ     \ Sender
MSG-PENDING-COUNT 1 ASSERT-EQ

\ Receive message 2
MSG-RECEIVE
1 ASSERT-EQ      \ Valid flag
$CCDD ASSERT-EQ  \ Payload
2 ASSERT-EQ      \ Type
20 ASSERT-EQ     \ Sender
MSG-PENDING-COUNT 0 ASSERT-EQ

\ Receive on empty queue returns valid flag = 0
MSG-RECEIVE
0 ASSERT-EQ \ flag
DROP DROP DROP

: FILL-MSG-QUEUE-32 ( -- )
    32 0 DO
        1 1 I MSG-POST DROP
    LOOP
;

S" Message queue: capacity limit and overflow detection" TEST-CASE-START
MSG-QUEUE-INIT
\ Fill queue to max capacity (32 items)
FILL-MSG-QUEUE-32
MSG-PENDING-COUNT 32 ASSERT-EQ
MSG-Q-OVERFLOW-COUNT @ 0 ASSERT-EQ

\ 33rd post should fail
1 1 999 MSG-POST 0 ASSERT-EQ
MSG-Q-OVERFLOW-COUNT @ 1 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 4: Housekeeping Binary Telemetry and Space Packet Assembly
\ ------------------------------------------------------------------------------

S" Housekeeping telemetry: packet generation and sizing" TEST-CASE-START
TM-INIT
TM-PACK-HOUSEKEEPING
32 ASSERT-EQ \ 32-byte binary payload
TM-PAYLOAD-BUF ASSERT-EQ

\ Generate complete CCSDS Space Packet
TM-GENERATE-SPACE-PACKET
\ Sizing: 6 bytes primary header + 32 bytes housekeeping + 2 bytes CRC = 40 bytes
40 ASSERT-EQ
TM-PACKET-BUF ASSERT-EQ
TM-PACKET-COUNT @ 1 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 5: Flight Software Multi-Rate Executive Cadence
\ ------------------------------------------------------------------------------

S" Real-time executive: multi-rate execution cadence" TEST-CASE-START
EXEC-INIT
EXEC-RUNNING @ 1 ASSERT-EQ
EXEC-CYCLE-COUNT @ 0 ASSERT-EQ

\ Run executive for 20 ms
20 EXEC-RUN
EXEC-CYCLE-COUNT @ 20 ASSERT-EQ

\ Check that fast servo task (ID 0) ran 20 times
0 TCB-ADDR 5 CELLS + @ 20 ASSERT-EQ
\ Check that gimbal trajectory task (ID 1, period 10 ms) ran 2 times
1 TCB-ADDR 5 CELLS + @ 2 ASSERT-EQ
