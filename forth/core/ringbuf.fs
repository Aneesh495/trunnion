\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/ringbuf.fs
\ Description: Circular FIFO ring buffer manager for telemetry streams,
\              optical detector DMA frames, and inter-task message queues.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Ring buffer header structure:
\   Offset +0:  head (write index)
\   Offset +8:  tail (read index)
\   Offset +16: capacity (maximum items)
\   Offset +24: item_size (bytes per item)
\   Offset +32: data buffer payload starts here

32 CONSTANT RINGBUF-HEADER-SIZE

\ Field Accessors
: RB-HEAD@ ( addr -- head ) @ ;
: RB-HEAD! ( head addr -- ) ! ;
: RB-TAIL@ ( addr -- tail ) CELL+ @ ;
: RB-TAIL! ( tail addr -- ) CELL+ ! ;
: RB-CAP@ ( addr -- cap ) 2 CELLS + @ ;
: RB-CAP! ( cap addr -- ) 2 CELLS + ! ;
: RB-SIZE@ ( addr -- size ) 3 CELLS + @ ;
: RB-SIZE! ( size addr -- ) 3 CELLS + ! ;
: RB-DATA ( addr -- data_ptr ) 4 CELLS + ;

\ ------------------------------------------------------------------------------
\ Section 1: Initialization and Status
\ ------------------------------------------------------------------------------

\ RINGBUF-INIT ( addr capacity item_size -- )
\ Initialize ring buffer header at addr with given capacity and element size.
: RINGBUF-INIT ( addr capacity item_size -- )
    ROT >R \ R: addr
    R@ RB-SIZE!
    R@ RB-CAP!
    0 R@ RB-HEAD!
    0 R@ RB-TAIL!
    R@ RB-CAP@ R@ RB-SIZE@ *
    R@ RB-DATA SWAP ERASE
    R> DROP
;

\ RINGBUF-CLEAR ( addr -- )
\ Reset head and tail pointers to empty.
: RINGBUF-CLEAR ( addr -- )
    0 OVER RB-HEAD!
    0 SWAP RB-TAIL!
;

\ RINGBUF-COUNT ( addr -- count )
\ Return current number of occupied items in ring buffer.
: RINGBUF-COUNT ( addr -- count )
    DUP RB-HEAD@
    OVER RB-TAIL@ -
    SWAP RB-CAP@ MOD
;

\ RINGBUF-EMPTY? ( addr -- flag )
\ Return TRUE if ring buffer is empty.
: RINGBUF-EMPTY? ( addr -- flag )
    DUP RB-HEAD@
    SWAP RB-TAIL@ =
;

\ RINGBUF-FULL? ( addr -- flag )
\ Return TRUE if ring buffer is full (count == capacity - 1).
: RINGBUF-FULL? ( addr -- flag )
    DUP RB-HEAD@ 1+
    OVER RB-CAP@ MOD
    SWAP RB-TAIL@ =
;

\ ------------------------------------------------------------------------------
\ Section 2: Push and Pop Data Operations
\ ------------------------------------------------------------------------------

\ RINGBUF-PUSH ( elem_addr addr -- success_flag )
\ Push item at elem_addr into ring buffer. Returns TRUE on success, FALSE if full.
: RINGBUF-PUSH ( elem addr -- flag )
    DUP RINGBUF-FULL? IF
        2DROP FALSE EXIT
    THEN
    >R \ R: addr
    \ Compute slot address: RB-DATA + head * item_size
    R@ RB-DATA
    R@ RB-HEAD@ R@ RB-SIZE@ * +
    \ Copy element into slot
    OVER SWAP R@ RB-SIZE@ MOVE
    DROP \ drop elem
    \ Increment head: head = (head + 1) % cap
    R@ RB-HEAD@ 1+ R@ RB-CAP@ MOD
    R@ RB-HEAD!
    R> DROP
    TRUE
;

\ RINGBUF-POP ( dst_addr addr -- success_flag )
\ Pop next item into dst_addr. Returns TRUE on success, FALSE if empty.
: RINGBUF-POP ( dst addr -- flag )
    DUP RINGBUF-EMPTY? IF
        2DROP FALSE EXIT
    THEN
    >R \ R: addr
    \ Compute slot address: RB-DATA + tail * item_size
    R@ RB-DATA
    R@ RB-TAIL@ R@ RB-SIZE@ * +
    \ Copy element to dst
    OVER R@ RB-SIZE@ MOVE
    DROP \ drop dst
    \ Increment tail: tail = (tail + 1) % cap
    R@ RB-TAIL@ 1+ R@ RB-CAP@ MOD
    R@ RB-TAIL!
    R> DROP
    TRUE
;

\ RINGBUF-PEEK ( dst_addr addr -- success_flag )
\ Copy next item into dst_addr without advancing tail pointer.
: RINGBUF-PEEK ( dst addr -- flag )
    DUP RINGBUF-EMPTY? IF
        2DROP FALSE EXIT
    THEN
    >R
    R@ RB-DATA
    R@ RB-TAIL@ R@ RB-SIZE@ * +
    OVER R@ RB-SIZE@ MOVE
    DROP R> DROP
    TRUE
;

\ ------------------------------------------------------------------------------
\ Section 3: High-Speed Single-Cell Ring Buffer Helpers
\ ------------------------------------------------------------------------------

\ RINGBUF-PUSH-CELL ( val addr -- success_flag )
\ Push single 64-bit cell directly into ring buffer.
: RINGBUF-PUSH-CELL ( val addr -- flag )
    DUP RINGBUF-FULL? IF
        2DROP FALSE EXIT
    THEN
    >R
    R@ RB-DATA R@ RB-HEAD@ CELLS + !
    R@ RB-HEAD@ 1+ R@ RB-CAP@ MOD R@ RB-HEAD!
    R> DROP
    TRUE
;

\ RINGBUF-POP-CELL ( addr -- val success_flag )
\ Pop single 64-bit cell directly from ring buffer.
: RINGBUF-POP-CELL ( addr -- val flag )
    DUP RINGBUF-EMPTY? IF
        DROP 0 FALSE EXIT
    THEN
    >R
    R@ RB-DATA R@ RB-TAIL@ CELLS + @
    R@ RB-TAIL@ 1+ R@ RB-CAP@ MOD R@ RB-TAIL!
    R> DROP
    TRUE
;
