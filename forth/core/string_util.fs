\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/string_util.fs
\ Description: String manipulation utilities, formatted numeric serialization,
\              ASCII telemetry token parsing, and memory hex dumps.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: String Length and Comparison
\ ------------------------------------------------------------------------------

\ STR-LEN ( c-addr max_len -- len )
\ Compute length of null-terminated string up to max_len.
: STR-LEN ( c-addr max_len -- len )
    0 DO
        DUP I + C@ 0= IF
            DROP I UNLOOP EXIT
        THEN
    LOOP
    DROP
;

\ STR-CMP ( c-addr1 len1 c-addr2 len2 -- diff )
\ Compare two strings lexicographically. Returns 0 if identical, <0 if s1 < s2, >0 if s1 > s2.
: STR-CMP ( c-addr1 len1 c-addr2 len2 -- diff )
    2OVER 2OVER MIN >R \ R: min_len
    DROP NIP \ Stack: ( c-addr1 c-addr2 )
    R> 0 DO
        OVER I + C@
        OVER I + C@
        2DUP - DUP 0<> IF
            -ROT 2DROP UNLOOP EXIT
        THEN
        2DROP
    LOOP
    2DROP
    \ If prefixes match, compare lengths: len1 - len2
    -
;

\ STR-PREFIX? ( c-addr len prefix-addr prefix-len -- flag )
\ Return TRUE if string begins with specified prefix.
: STR-PREFIX? ( c-addr len p-addr p-len -- flag )
    2DUP >R >R
    ROT OVER < IF
        2DROP 2DROP R> R> 2DROP FALSE EXIT
    THEN
    R> R> \ Stack: ( c-addr len p-addr p-len )
    ROT DROP \ Stack: ( c-addr p-addr p-len )
    0 DO
        OVER I + C@
        OVER I + C@ <> IF
            2DROP UNLOOP FALSE EXIT
        THEN
    LOOP
    2DROP TRUE
;

\ ------------------------------------------------------------------------------
\ Section 2: Number to String Conversion
\ ------------------------------------------------------------------------------

VARIABLE STR-CONV-BUF 64 ALLOT

\ U>STR ( u dst_buf -- len )
\ Convert unsigned 64-bit integer to decimal string at dst_buf.
: U>STR ( u dst -- len )
    SWAP DUP 0= IF
        DROP [CHAR] 0 OVER C!
        0 OVER 1+ C!
        DROP 1 EXIT
    THEN
    \ Stack: ( dst u )
    STR-CONV-BUF 63 + 0 OVER C! \ Null-terminate scratch buffer
    >R \ R: scratch_ptr
    BEGIN
        DUP 0>
    WHILE
        10 /MOD
        [CHAR] 0 +
        R@ 1- DUP >R C!
        R>
    REPEAT
    DROP \ Drop remaining 0
    \ Copy scratch string to destination
    STR-CONV-BUF 63 + R@ - >R \ R: len
    R@ OVER R> MOVE
    DROP R>
;

\ HEX>STR ( u dst_buf -- len )
\ Convert 32-bit unsigned integer to 8-character uppercase hex string.
: HEX>STR ( u dst -- len )
    SWAP $FFFFFFFF AND SWAP
    8 0 DO
        OVER 7 I - 4 * RSHIFT $0F AND
        DUP 10 < IF
            [CHAR] 0 +
        ELSE
            10 - [CHAR] A +
        THEN
        OVER I + C!
    LOOP
    NIP
    8
;

\ ------------------------------------------------------------------------------
\ Section 3: Diagnostic Hex Memory Dump
\ ------------------------------------------------------------------------------

\ HEX-DUMP-LINE ( addr len -- )
\ Dump up to 16 bytes at addr in hex and ASCII columns.
: HEX-DUMP-LINE ( addr len -- )
    \ Print 16-bit offset
    OVER $FFFF AND BASE @ >R HEX 0 U. R> BASE ! [CHAR] : EMIT SPACE

    \ Print hex columns
    16 0 DO
        DUP I > IF
            OVER I + C@
            BASE @ >R HEX
            DUP 16 < IF [CHAR] 0 EMIT THEN
            U. R> BASE !
        ELSE
            ."    "
        THEN
        I 7 = IF SPACE THEN
    LOOP

    ."  | "

    \ Print printable ASCII columns
    0 DO
        DUP I + C@
        DUP 32 >= OVER 126 <= AND IF
            EMIT
        ELSE
            DROP [CHAR] . EMIT
        THEN
    LOOP
    DROP
    CR
;

\ HEX-DUMP ( addr len -- )
\ Display formatted hexadecimal dump of memory region.
: HEX-DUMP ( addr len -- )
    BEGIN
        DUP 0>
    WHILE
        2DUP 16 MIN HEX-DUMP-LINE
        DUP 16 > IF
            16 - SWAP 16 + SWAP
        ELSE
            2DROP EXIT
        THEN
    REPEAT
    2DROP
;
