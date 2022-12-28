\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kernel/task.fs
\ Description: Cooperative multitasking kernel, Task Control Block (TCB)
\              management, and round-robin scheduler for flight software threads.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Task Control Block Layout
\ ------------------------------------------------------------------------------

0 CONSTANT TASK-STATE-STOPPED
1 CONSTANT TASK-STATE-READY
2 CONSTANT TASK-STATE-RUNNING
3 CONSTANT TASK-STATE-WAITING

\ TCB Record Layout (6 cells = 48 bytes):
\ Offset 0: Task ID
\ Offset 1: State (STOPPED, READY, RUNNING, WAITING)
\ Offset 2: Execution XT (word execution token)
\ Offset 3: Period in milliseconds
\ Offset 4: Elapsed ticks
\ Offset 5: Total execution count
6 CONSTANT TCB-CELLS
TCB-CELLS CELLS CONSTANT TCB-BYTES

8 CONSTANT KERNEL-MAX-TASKS
CREATE KERNEL-TCB-TABLE KERNEL-MAX-TASKS TCB-BYTES * ALLOT

VARIABLE KERNEL-NUM-TASKS
VARIABLE KERNEL-CURR-TASK-ID
VARIABLE KERNEL-SCHED-TICKS

\ ------------------------------------------------------------------------------
\ Section 2: Task Registration and Lifecycle
\ ------------------------------------------------------------------------------

\ TCB-ADDR ( task_idx -- addr )
: TCB-ADDR ( idx -- addr )
    TCB-BYTES * KERNEL-TCB-TABLE +
;

\ TASK-INIT-KERNEL ( -- )
: TASK-INIT-KERNEL ( -- )
    0 KERNEL-NUM-TASKS !
    0 KERNEL-CURR-TASK-ID !
    0 KERNEL-SCHED-TICKS !
    KERNEL-TCB-TABLE KERNEL-MAX-TASKS TCB-BYTES * ERASE
;

\ TASK-SPAWN ( xt period_ms -- task_id )
\ Register a new task with given execution token and rate period.
: TASK-SPAWN ( xt period -- id )
    KERNEL-NUM-TASKS @ KERNEL-MAX-TASKS >= IF
        2DROP -1 EXIT
    THEN

    KERNEL-NUM-TASKS @ DUP >R TCB-ADDR >R
    \ Set period
    R@ 3 CELLS + !
    \ Set execution XT
    R@ 2 CELLS + !
    \ Set state READY
    TASK-STATE-READY R@ 1 CELLS + !
    \ Set ID
    R@ 0 CELLS + !
    \ Clear elapsed ticks and count
    0 R@ 4 CELLS + !
    0 R> 5 CELLS + !

    1 KERNEL-NUM-TASKS +!
    R>
;

\ ------------------------------------------------------------------------------
\ Section 3: Cooperative Dispatcher
\ ------------------------------------------------------------------------------

\ TASK-STEP ( -- )
\ Advance round-robin scheduler by one millisecond tick.
: TASK-STEP ( -- )
    1 KERNEL-SCHED-TICKS +!

    KERNEL-NUM-TASKS @ 0 DO
        I TCB-ADDR >R
        R@ 1 CELLS + @ TASK-STATE-READY = IF
            \ Advance elapsed tick counter
            1 R@ 4 CELLS + +!

            \ Check if task period elapsed
            R@ 4 CELLS + @ R@ 3 CELLS + @ >= IF
                0 R@ 4 CELLS + ! \ Reset elapsed counter
                TASK-STATE-RUNNING R@ 1 CELLS + !
                I KERNEL-CURR-TASK-ID !

                \ Execute task body
                R@ 2 CELLS + @ EXECUTE

                1 R@ 5 CELLS + +!
                TASK-STATE-READY R@ 1 CELLS + !
            THEN
        THEN
        R> DROP
    LOOP
;

\ YIELD ( -- )
\ Cooperative yield placeholder for flight software tasks.
: YIELD ( -- )
    \ In cooperative model, return control to scheduler
;
