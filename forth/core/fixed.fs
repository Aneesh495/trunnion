\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/fixed.fs
\ Description: High-precision signed Q32.32 and Q16.16 fixed-point arithmetic
\              engine for sub-nanoradian pointing and high-rate servo loops.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Fixed-Point Scale Constants
\ ------------------------------------------------------------------------------

\ Q32.32 Scaling constants
32 CONSTANT Q32-SHIFT
$100000000 CONSTANT Q32-ONE
$0FFFFFFFF CONSTANT Q32-FRAC-MASK

\ Q16.16 Scaling constants
16 CONSTANT Q16-SHIFT
$10000 CONSTANT Q16-ONE
$0FFFF CONSTANT Q16-FRAC-MASK

\ Mathematical Constants in Q32.32 (Precomputed for zero runtime latency)
\ PI = 3.141592653589793238 * 2^32 = 13493037704 = 0x3243F6A88
13493037704 CONSTANT Q32-PI
\ TWO-PI = 26986075409 = 0x6487ED511
26986075409 CONSTANT Q32-TWO-PI
\ HALF-PI = 6746518852 = 0x1921FB544
6746518852 CONSTANT Q32-HALF-PI
\ PI/4 = 3373259426 = 0x0C90FDAA2
3373259426 CONSTANT Q32-QUARTER-PI

\ Degrees to Radians conversion factor: (pi / 180) * 2^32 = 74961320
74961320 CONSTANT Q32-DEG-TO-RAD
\ Radians to Degrees conversion factor: (180 / pi) * 2^32 = 246067087799
246067087799 CONSTANT Q32-RAD-TO-DEG

\ Radians to Microradians factor: 1,000,000 in Q32.32 = 4294967296000000
\ Microradians in Q32.32: 1 urad = 4295
4295 CONSTANT Q32-ONE-URAD

\ ------------------------------------------------------------------------------
\ Section 2: Q32.32 Arithmetic Primitives
\ ------------------------------------------------------------------------------

\ S>Q32 ( n -- q32 )
\ Convert signed integer to Q32.32 fixed-point.
: S>Q32 ( n -- q32 )
    Q32-SHIFT LSHIFT
;

\ Q32>S ( q32 -- n )
\ Convert Q32.32 fixed-point to truncated integer.
: Q32>S ( q32 -- n )
    Q32-SHIFT ARSHIFT
;

\ Q32-ROUND ( q32 -- n )
\ Convert Q32.32 fixed-point to rounded integer.
: Q32-ROUND ( q32 -- n )
    DUP 0< IF
        $80000000 - Q32-SHIFT ARSHIFT
    ELSE
        $80000000 + Q32-SHIFT ARSHIFT
    THEN
;

\ Q32+ ( a b -- a+b )
\ Add two Q32.32 fixed-point numbers.
: Q32+ ( a b -- a+b )
    +
;

\ Q32- ( a b -- a-b )
\ Subtract Q32.32 b from a.
: Q32- ( a b -- a-b )
    -
;

\ Q32-NEG ( a -- -a )
\ Negate Q32.32 number.
: Q32-NEG ( a -- -a )
    NEGATE
;

\ Q32-ABS ( a -- |a| )
\ Absolute value of Q32.32 number.
: Q32-ABS ( a -- |a| )
    ABS
;

\ Q32* and Q32/ are implemented as high-performance 128-bit primitives
\ in the Trunnion execution engine. For environments where they are not
\ built-in, equivalent 128-bit intermediate math is supported.
[UNDEFINED] Q32* [IF]
: Q32* ( a b -- prod )
    M* SWAP $FFFFFFFF00000000 AND 32 RSHIFT SWAP 32 LSHIFT OR
;
[THEN]

[UNDEFINED] Q32/ [IF]
: Q32-ASR32 ( n -- n_shifted )
    32 0 DO 2/ LOOP
;
: Q32/ ( a b -- quot )
    DUP 0= IF 2DROP 0 EXIT THEN
    >R DUP 32 LSHIFT SWAP Q32-ASR32 R> SM/REM NIP
;
[THEN]

\ Q32-MIN ( a b -- min_ab )
: Q32-MIN ( a b -- min_ab )
    MIN
;

\ Q32-MAX ( a b -- max_ab )
: Q32-MAX ( a b -- max_ab )
    MAX
;

\ Q32-CLAMP ( val min_val max_val -- clamped )
: Q32-CLAMP ( val min_val max_val -- clamped )
    CLAMP
;

\ ------------------------------------------------------------------------------
\ Section 3: Q16.16 High-Rate Servo Arithmetic
\ ------------------------------------------------------------------------------

\ S>Q16 ( n -- q16 )
\ Convert integer to Q16.16 fixed-point.
: S>Q16 ( n -- q16 )
    Q16-SHIFT LSHIFT
;

\ Q16>S ( q16 -- n )
\ Convert Q16.16 fixed-point to truncated integer.
: Q16>S ( q16 -- n )
    Q16-SHIFT ARSHIFT
;

\ Q16-ROUND ( q16 -- n )
\ Convert Q16.16 fixed-point to rounded integer.
: Q16-ROUND ( q16 -- n )
    DUP 0< IF
        $8000 - Q16-SHIFT ARSHIFT
    ELSE
        $8000 + Q16-SHIFT ARSHIFT
    THEN
;

\ Q16+ ( a b -- a+b )
: Q16+ ( a b -- a+b )
    +
;

\ Q16- ( a b -- a-b )
: Q16- ( a b -- a-b )
    -
;

\ Q16-NEG ( a -- -a )
: Q16-NEG ( a -- -a )
    NEGATE
;

\ Q16-ABS ( a -- |a| )
: Q16-ABS ( a -- |a| )
    ABS
;

\ Q16* ( a b -- a*b )
\ Fast Q16.16 multiplication fitting within 64-bit cell without overflow.
: Q16* ( a b -- a*b )
    * Q16-SHIFT ARSHIFT
;

\ Q16/ ( a b -- a/b )
\ Fast Q16.16 division fitting within 64-bit cell.
: Q16/ ( a b -- a/b )
    DUP 0= IF 2DROP 0 EXIT THEN
    SWAP Q16-SHIFT LSHIFT SWAP /
;

\ Q16>Q32 ( q16 -- q32 )
\ Promote Q16.16 to Q32.32 format.
: Q16>Q32 ( q16 -- q32 )
    16 LSHIFT
;

\ Q32>Q16 ( q32 -- q16 )
\ Demote Q32.32 to Q16.16 format.
: Q32>Q16 ( q32 -- q16 )
    16 RSHIFT
;

\ ------------------------------------------------------------------------------
\ Section 4: Unit Conversion Helpers
\ ------------------------------------------------------------------------------

\ DEG>RAD-Q32 ( deg_q32 -- rad_q32 )
\ Convert degrees to radians in Q32.32 format.
: DEG>RAD-Q32 ( deg_q32 -- rad_q32 )
    Q32-DEG-TO-RAD Q32*
;

\ RAD>DEG-Q32 ( rad_q32 -- deg_q32 )
\ Convert radians to degrees in Q32.32 format.
: RAD>DEG-Q32 ( rad_q32 -- deg_q32 )
    Q32-RAD-TO-DEG Q32*
;

\ RAD>URAD-Q32 ( rad_q32 -- urad_int )
\ Convert radians in Q32.32 to microradians integer.
: RAD>URAD-Q32 ( rad_q32 -- urad_int )
    1000000 * Q32-SHIFT RSHIFT
;

\ URAD>RAD-Q32 ( urad_int -- rad_q32 )
\ Convert microradians integer to radians in Q32.32.
: URAD>RAD-Q32 ( urad_int -- rad_q32 )
    Q32-SHIFT LSHIFT 1000000 /
;

\ ------------------------------------------------------------------------------
\ Section 5: Formatted Output Utilities
\ ------------------------------------------------------------------------------

\ Q32. ( q32 -- )
\ Print Q32.32 number as decimal with 6 fractional digits.
: Q32. ( q32 -- )
    DUP 0< IF
        [CHAR] - EMIT
        NEGATE
    THEN
    DUP Q32-SHIFT RSHIFT .
    [CHAR] . EMIT
    $FFFFFFFF AND
    1000000 * 32 RSHIFT
    DUP 100000 < IF [CHAR] 0 EMIT THEN
    DUP 10000 < IF [CHAR] 0 EMIT THEN
    DUP 1000 < IF [CHAR] 0 EMIT THEN
    DUP 100 < IF [CHAR] 0 EMIT THEN
    DUP 10 < IF [CHAR] 0 EMIT THEN
    U.
;

\ Q16. ( q16 -- )
\ Print Q16.16 number as decimal with 4 fractional digits.
: Q16. ( q16 -- )
    DUP 0< IF
        [CHAR] - EMIT
        NEGATE
    THEN
    DUP Q16-SHIFT RSHIFT .
    [CHAR] . EMIT
    $FFFF AND
    10000 * Q16-SHIFT RSHIFT
    DUP 1000 < IF [CHAR] 0 EMIT THEN
    DUP 100 < IF [CHAR] 0 EMIT THEN
    DUP 10 < IF [CHAR] 0 EMIT THEN
    U.
;
