\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kernel/msg_queue.fs
\ Description: Circular message queue and mailbox buffer for decoupled inter-task
\              asynchronous communication across flight software modules.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Message Layout and Buffer Storage
\ ------------------------------------------------------------------------------

32 CONSTANT MSG-QUEUE-MAX-ITEMS
3  CONSTANT MSG-ITEM-CELLS \ (sender_id, msg_type, payload_val)

CREATE MSG-QUEUE-BUF MSG-QUEUE-MAX-ITEMS MSG-ITEM-CELLS CELLS * ALLOT

VARIABLE MSG-Q-HEAD
VARIABLE MSG-Q-TAIL
VARIABLE MSG-Q-COUNT
VARIABLE MSG-Q-OVERFLOW-COUNT

\ ------------------------------------------------------------------------------
\ Section 2: Queue Management API
\ ------------------------------------------------------------------------------

\ MSG-QUEUE-INIT ( -- )
: MSG-QUEUE-INIT ( -- )
    0 MSG-Q-HEAD !
    0 MSG-Q-TAIL !
    0 MSG-Q-COUNT !
    0 MSG-Q-OVERFLOW-COUNT !
    MSG-QUEUE-BUF MSG-QUEUE-MAX-ITEMS MSG-ITEM-CELLS CELLS * ERASE
;

\ MSG-POST ( sender_id msg_type payload -- success_flag )
\ Enqueue a message item into circular buffer.
: MSG-POST ( sender type payload -- flag )
    MSG-Q-COUNT @ MSG-QUEUE-MAX-ITEMS >= IF
        3DROP 1 MSG-Q-OVERFLOW-COUNT +!
        0 EXIT
    THEN

    MSG-Q-HEAD @ MSG-ITEM-CELLS CELLS * MSG-QUEUE-BUF + >R
    R@ 2 CELLS + !
    R@ 1 CELLS + !
    R> 0 CELLS + !

    MSG-Q-HEAD @ 1+ MSG-QUEUE-MAX-ITEMS MOD MSG-Q-HEAD !
    1 MSG-Q-COUNT +!
    1
;

\ MSG-RECEIVE ( -- sender_id msg_type payload valid_flag )
\ Dequeue oldest message item from circular buffer.
: MSG-RECEIVE ( -- sender type payload flag )
    MSG-Q-COUNT @ 0= IF
        0 0 0 0 EXIT
    THEN

    MSG-Q-TAIL @ MSG-ITEM-CELLS CELLS * MSG-QUEUE-BUF + >R
    R@ 0 CELLS + @
    R@ 1 CELLS + @
    R@ 2 CELLS + @
    R> DROP

    MSG-Q-TAIL @ 1+ MSG-QUEUE-MAX-ITEMS MOD MSG-Q-TAIL !
    -1 MSG-Q-COUNT +!
    1
;

\ MSG-PENDING-COUNT ( -- count )
: MSG-PENDING-COUNT ( -- count )
    MSG-Q-COUNT @
;
