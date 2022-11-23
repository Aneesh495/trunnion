\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/preamble.fs
\ Description: ANSI Forth-2012 environment compatibility, basic constants,
\              stack manipulation aliases, and debugging primitives.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Standard Constants and Flags
\ ------------------------------------------------------------------------------

0 CONSTANT FALSE
-1 CONSTANT TRUE

\ Cell size constants
8 CONSTANT CELL-BYTES
64 CONSTANT BITS-PER-CELL

\ Memory alignment boundary
8 CONSTANT ALIGN-BOUNDARY

\ Maximum and minimum signed cell integers
$7FFFFFFFFFFFFFFF CONSTANT MAX-INT
$8000000000000000 CONSTANT MIN-INT

\ Maximum unsigned cell integer
$FFFFFFFFFFFFFFFF CONSTANT MAX-UINT

\ ------------------------------------------------------------------------------
\ Section 2: Stack Manipulation Helpers
\ ------------------------------------------------------------------------------

\ 3DUP ( a b c -- a b c a b c )
\ Duplicate the top three stack cells.
: 3DUP ( a b c -- a b c a b c )
    2 PICK 2 PICK 2 PICK
;

\ 3DROP ( a b c -- )
\ Discard the top three stack cells.
: 3DROP ( a b c -- )
    DROP DROP DROP
;

\ 4DUP ( a b c d -- a b c d a b c d )
\ Duplicate the top four stack cells.
: 4DUP ( a b c d -- a b c d a b c d )
    2OVER 2OVER
;

\ 4DROP ( a b c d -- )
\ Discard the top four stack cells.
: 4DROP ( a b c d -- )
    2DROP 2DROP
;

\ UNDER+ ( a b c -- a+c b )
\ Add top cell to the cell beneath the second stack element.
: UNDER+ ( a b c -- a+c b )
    ROT + SWAP
;

\ ------------------------------------------------------------------------------
\ Section 3: Bitmask Manipulation Primitives
\ ------------------------------------------------------------------------------

\ BIT ( u -- mask )
\ Generate a single-bit mask with bit u set.
: BIT ( u -- mask )
    1 SWAP LSHIFT
;

\ SET-BIT ( val bit_idx -- val' )
\ Set bit at bit_idx in val.
: SET-BIT ( val bit_idx -- val' )
    BIT OR
;

\ CLEAR-BIT ( val bit_idx -- val' )
\ Clear bit at bit_idx in val.
: CLEAR-BIT ( val bit_idx -- val' )
    BIT INVERT AND
;

\ TEST-BIT ( val bit_idx -- flag )
\ Return TRUE if bit at bit_idx is set, else FALSE.
: TEST-BIT ( val bit_idx -- flag )
    BIT AND 0<>
;

\ TOGGLE-BIT ( val bit_idx -- val' )
\ Toggle bit at bit_idx in val.
: TOGGLE-BIT ( val bit_idx -- val' )
    BIT XOR
;

\ MASK ( n -- mask )
\ Create a bitmask of n contiguous low bits set to 1.
: MASK ( n -- mask )
    1 SWAP LSHIFT 1-
;

\ ARSHIFT ( n1 u -- n2 )
\ Arithmetic right shift by u bit positions preserving sign bit.
[UNDEFINED] ARSHIFT [IF]
: ARSHIFT ( n1 u -- n2 )
    0 ?DO 2/ LOOP
;
[THEN]

\ EXTRACT-FIELD ( val shift width -- field )
\ Extract bitfield of given width after shifting right by shift.
: EXTRACT-FIELD ( val shift width -- field )
    >R RSHIFT R> MASK AND
;

\ INSERT-FIELD ( val field shift width -- val' )
\ Insert field into val at specified shift offset and width.
: INSERT-FIELD ( val field shift width -- val' )
    >R >R OVER R> R> 2DUP MASK SWAP LSHIFT INVERT -ROT
    MASK AND SWAP LSHIFT ROT AND OR
;

\ ------------------------------------------------------------------------------
\ Section 4: Arithmetic and Bounds Check Helpers
\ ------------------------------------------------------------------------------

\ CLAMP ( val min_val max_val -- clamped )
\ Restrict val to the inclusive range [min_val, max_val].
: CLAMP ( val min_val max_val -- clamped )
    ROT MIN MAX
;

\ SGN ( n -- -1 | 0 | 1 )
\ Return signum of signed integer.
: SGN ( n -- sgn )
    DUP 0= IF
        EXIT
    THEN
    0< IF -1 ELSE 1 THEN
;

\ WITHIN? ( val min_val max_val -- flag )
\ Check if min_val <= val <= max_val (inclusive).
: WITHIN? ( val min_val max_val -- flag )
    >R OVER >= SWAP R> <= AND
;

\ DIFF ( a b -- abs_diff )
\ Compute absolute difference between two signed cells.
: DIFF ( a b -- abs_diff )
    - ABS
;

\ ------------------------------------------------------------------------------
\ Section 5: Memory Access Utilities
\ ------------------------------------------------------------------------------

\ BOUNDED-MOVE ( src dst count max_len -- copied_count )
\ Copy memory up to max_len limit to prevent buffer overflow.
: BOUNDED-MOVE ( src dst count max_len -- copied_count )
    MIN DUP >R
    ROT ROT R@ MOVE
    R>
;

\ ZERO-MEMORY ( addr u -- )
\ Clear memory region of u bytes at addr to zero.
: ZERO-MEMORY ( addr u -- )
    ERASE
;

\ ------------------------------------------------------------------------------
\ Section 6: Console Formatting and Diagnostic Utilities
\ ------------------------------------------------------------------------------

\ PRINT-HEX32 ( u -- )
\ Print 32-bit unsigned integer as formatted 8-digit hexadecimal.
: PRINT-HEX32 ( u -- )
    BASE @ >R HEX
    $FFFFFFFF AND
    DUP 0= IF
        ." 00000000"
    ELSE
        ." 0x" U.
    THEN
    R> BASE !
;

\ PRINT-BOOL ( flag -- )
\ Print boolean flag as human-readable string.
: PRINT-BOOL ( flag -- )
    IF ." TRUE" ELSE ." FALSE" THEN
;

\ PRINT-LINE ( -- )
\ Print standard 78-character diagnostic separator line.
: PRINT-LINE ( -- )
    78 0 DO [CHAR] = EMIT LOOP CR
;

\ PRINT-DASH-LINE ( -- )
\ Print standard 78-character dashed diagnostic separator line.
: PRINT-DASH-LINE ( -- )
    78 0 DO [CHAR] - EMIT LOOP CR
;
