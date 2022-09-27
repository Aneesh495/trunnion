\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/double.fs
\ Description: Double-cell (128-bit / dual 64-bit cell) arithmetic, scaling,
\              logical shifts, and comparison words for optical tracking counters.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ In standard Forth double numbers, the least significant cell (d_lo) is
\ lower on the stack, and the most significant cell (d_hi) is on top:
\ Stack representation: ( d_lo d_hi )

\ ------------------------------------------------------------------------------
\ Section 1: Double Stack Manipulation
\ ------------------------------------------------------------------------------

\ D0= ( d -- flag )
\ Return TRUE if double number d equals 0.
: D0= ( d_lo d_hi -- flag )
    OR 0=
;

\ D0< ( d -- flag )
\ Return TRUE if double number d is negative.
: D0< ( d_lo d_hi -- flag )
    NIP 0<
;

\ D= ( d1 d2 -- flag )
\ Return TRUE if double numbers d1 and d2 are identical.
: D= ( d1_lo d1_hi d2_lo d2_hi -- flag )
    ROT = -ROT = AND
;

\ D<> ( d1 d2 -- flag )
\ Return TRUE if double numbers d1 and d2 are not identical.
: D<> ( d1_lo d1_hi d2_lo d2_hi -- flag )
    D= 0=
;

\ D< ( d1 d2 -- flag )
\ Return TRUE if signed double d1 is strictly less than d2.
: D< ( d1_lo d1_hi d2_lo d2_hi -- flag )
    ROT 2DUP = IF
        2DROP U<
    ELSE
        > NIP NIP
    THEN
;

\ D> ( d1 d2 -- flag )
\ Return TRUE if signed double d1 is strictly greater than d2.
: D> ( d1_lo d1_hi d2_lo d2_hi -- flag )
    2SWAP D<
;

\ DU< ( ud1 ud2 -- flag )
\ Return TRUE if unsigned double ud1 is less than ud2.
: DU< ( ud1_lo ud1_hi ud2_lo ud2_hi -- flag )
    ROT 2DUP = IF
        2DROP U<
    ELSE
        U> NIP NIP
    THEN
;

\ ------------------------------------------------------------------------------
\ Section 2: Double Math Primitives
\ ------------------------------------------------------------------------------

\ DABS ( d -- |d| )
\ Compute absolute value of double number.
: DABS ( d_lo d_hi -- |d|_lo |d|_hi )
    DUP 0< IF DNEGATE THEN
;

\ DMIN ( d1 d2 -- min_d )
\ Return minimum of two signed double numbers.
: DMIN ( d1_lo d1_hi d2_lo d2_hi -- min_lo min_hi )
    4DUP D< IF 2DROP ELSE 2SWAP 2DROP THEN
;

\ DMAX ( d1 d2 -- max_d )
\ Return maximum of two signed double numbers.
: DMAX ( d1_lo d1_hi d2_lo d2_hi -- max_lo max_hi )
    4DUP D> IF 2DROP ELSE 2SWAP 2DROP THEN
;

\ S>D ( n -- d )
\ Sign-extend single cell integer into double cell.
: S>D ( n -- d_lo d_hi )
    DUP 0< IF -1 ELSE 0 THEN
;

\ D>S ( d -- n )
\ Truncate double cell integer to single cell.
: D>S ( d_lo d_hi -- n )
    DROP
;

\ ------------------------------------------------------------------------------
\ Section 3: Double Shifts and Scaling
\ ------------------------------------------------------------------------------

\ D2* ( d -- d*2 )
\ Double-cell arithmetic shift left by 1 bit.
: D2* ( d_lo d_hi -- d*2_lo d*2_hi )
    DUP 1 LSHIFT
    OVER 63 RSHIFT OR
    SWAP 1 LSHIFT SWAP
;

\ D2/ ( d -- d/2 )
\ Double-cell arithmetic shift right by 1 bit.
: D2/ ( d_lo d_hi -- d/2_lo d/2_hi )
    DUP 1 RSHIFT
    OVER 1 AND 63 LSHIFT
    ROT 1 RSHIFT OR
    SWAP
;

\ DLSHIFT ( d u -- d' )
\ Double-cell logical left shift by u bits (0 <= u < 64).
: DLSHIFT ( d_lo d_hi u -- d'_lo d'_hi )
    DUP 0= IF DROP EXIT THEN
    DUP >R LSHIFT
    OVER 64 R> - RSHIFT OR
    SWAP R@ LSHIFT SWAP
    R> DROP
;

\ DRSHIFT ( d u -- d' )
\ Double-cell logical right shift by u bits (0 <= u < 64).
: DRSHIFT ( d_lo d_hi u -- d'_lo d'_hi )
    DUP 0= IF DROP EXIT THEN
    DUP >R RSHIFT
    OVER 64 R> - LSHIFT OR
    ROT R@ RSHIFT ROT
    R> DROP
;

\ M+ ( d n -- d+n )
\ Add single signed cell to double cell.
: M+ ( d_lo d_hi n -- d'_lo d'_hi )
    S>D D+
;

\ ------------------------------------------------------------------------------
\ Section 4: Double Formatting and Printing
\ ------------------------------------------------------------------------------

\ D. ( d -- )
\ Print signed double number in current numeric base.
: D. ( d_lo d_hi -- )
    DUP 0< IF
        [CHAR] - EMIT
        DNEGATE
    THEN
    DUP 0= IF
        DROP U.
    ELSE
        SWAP U. SPACE U.
    THEN
;
