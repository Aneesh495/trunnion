\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_core_utils.fs
\ Description: Test Suite for Ring Buffer, Bitwise Operations, and Strings
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Core Utilities, Ring Buffer, Bit Manipulation, String Formatting" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Circular FIFO Ring Buffer
\ ------------------------------------------------------------------------------

CREATE TEST-RB 128 ALLOT

S" Ring buffer initialization and initial empty state" TEST-CASE-START
TEST-RB 8 8 RINGBUF-INIT
TEST-RB RINGBUF-EMPTY? ASSERT-TRUE
TEST-RB RINGBUF-FULL? ASSERT-FALSE
TEST-RB RINGBUF-COUNT 0 ASSERT-EQ

S" Ring buffer push 3 items" TEST-CASE-START
100 TEST-RB RINGBUF-PUSH-CELL ASSERT-TRUE
200 TEST-RB RINGBUF-PUSH-CELL ASSERT-TRUE
300 TEST-RB RINGBUF-PUSH-CELL ASSERT-TRUE
TEST-RB RINGBUF-COUNT 3 ASSERT-EQ
TEST-RB RINGBUF-EMPTY? ASSERT-FALSE

S" Ring buffer pop items in FIFO order" TEST-CASE-START
TEST-RB RINGBUF-POP-CELL ASSERT-TRUE 100 ASSERT-EQ
TEST-RB RINGBUF-POP-CELL ASSERT-TRUE 200 ASSERT-EQ
TEST-RB RINGBUF-POP-CELL ASSERT-TRUE 300 ASSERT-EQ
TEST-RB RINGBUF-EMPTY? ASSERT-TRUE

: RUN-RB-FILL-TEST ( -- )
    TEST-RB RINGBUF-CLEAR
    7 0 DO
        I 10 * TEST-RB RINGBUF-PUSH-CELL DROP
    LOOP
;

S" Ring buffer fill to capacity" TEST-CASE-START
RUN-RB-FILL-TEST
TEST-RB RINGBUF-FULL? ASSERT-TRUE
999 TEST-RB RINGBUF-PUSH-CELL ASSERT-FALSE

\ ------------------------------------------------------------------------------
\ Test Group 2: Bitwise Operations, Population Count, Gray Code
\ ------------------------------------------------------------------------------

S" POPCOUNT of 0 is 0" TEST-CASE-START
0 POPCOUNT 0 ASSERT-EQ

S" POPCOUNT of 0xFFFFFFFF is 32" TEST-CASE-START
$FFFFFFFF POPCOUNT 32 ASSERT-EQ

S" POPCOUNT of alternating bits 0x55555555 is 16" TEST-CASE-START
$55555555 POPCOUNT 16 ASSERT-EQ

S" PARITY of even bits is 0, odd bits is 1" TEST-CASE-START
$03 PARITY 0 ASSERT-EQ
$07 PARITY 1 ASSERT-EQ

S" HAMMING-DISTANCE between 0x1234 and 0x1235 is 1" TEST-CASE-START
$1234 $1235 HAMMING-DISTANCE 1 ASSERT-EQ

S" BIT-REVERSE32: reverse of 0x80000000 is 1" TEST-CASE-START
$80000000 BIT-REVERSE32 1 ASSERT-EQ

: RUN-GRAY-ROUNDTRIP ( -- flag )
    TRUE
    16 0 DO
        I BIN>GRAY GRAY>BIN I <> IF DROP FALSE THEN
    LOOP
;

S" Gray code encode and decode round-trip for 0..15" TEST-CASE-START
RUN-GRAY-ROUNDTRIP ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 3: CRC-16 and CRC-32 Generators
\ ------------------------------------------------------------------------------

CREATE TEST-STR-DATA 16 ALLOT
S" 123456789" TEST-STR-DATA SWAP MOVE

S" CRC-16-CCITT check vector: 123456789" TEST-CASE-START
TEST-STR-DATA 9 CRC16-CCITT-BUFFER
$29B1 ASSERT-EQ

S" CRC-32-IEEE check vector: 123456789" TEST-CASE-START
TEST-STR-DATA 9 CRC32-BUFFER
$112D769B ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 4: String Comparisons and Conversions
\ ------------------------------------------------------------------------------

CREATE STR-BUF-A 32 ALLOT
CREATE STR-BUF-B 32 ALLOT

S" String comparison: identical strings return 0" TEST-CASE-START
S" SATELLITE" STR-BUF-A SWAP MOVE
S" SATELLITE" STR-BUF-B SWAP MOVE
STR-BUF-A 9 STR-BUF-B 9 STR-CMP 0 ASSERT-EQ

S" String prefix matching: SATELLITE has prefix SAT" TEST-CASE-START
S" SATELLITE" S" SAT" STR-PREFIX? ASSERT-TRUE

S" Number to decimal string conversion: 12345" TEST-CASE-START
12345 STR-BUF-A U>STR 5 ASSERT-EQ

S" Number to hex string conversion: 0xABCD" TEST-CASE-START
$ABCD STR-BUF-A HEX>STR 8 ASSERT-EQ
