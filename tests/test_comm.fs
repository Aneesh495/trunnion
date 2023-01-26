\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_comm.fs
\ Description: Test Suite for M-PPM Modem, Reed-Solomon, Interleaver & CCSDS
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Optical Communications, M-PPM Modem, Reed-Solomon Codec, and CCSDS Framing" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: M-PPM Optical Modulator & Decision Slicer
\ ------------------------------------------------------------------------------

S" PPM modem: initialization to 16-PPM order" TEST-CASE-START
PPM-INIT
PPM-ORDER @ 16 ASSERT-EQ
PPM-BITS-PER-SYMBOL @ 4 ASSERT-EQ

S" PPM encoder: byte splits into two 4-bit slot symbols" TEST-CASE-START
$A5 PPM-ENCODE-BYTE
\ $A5 = 1010 0101b -> high nibble = 10, low nibble = 5
5 ASSERT-EQ
10 ASSERT-EQ

S" PPM decision slicer: maximum likelihood peak slot detection" TEST-CASE-START
PPM-SLOT-BUFFER 16 CELLS ERASE
\ Inject photon counts: slot 7 has highest photon arrival count
10 5 CELLS PPM-SLOT-BUFFER + !
150 7 CELLS PPM-SLOT-BUFFER + !
20 9 CELLS PPM-SLOT-BUFFER + !
PPM-SLOT-BUFFER PPM-SLICE-FRAME
\ Returns ( symbol snr_metric )
130 ASSERT-EQ \ Extinction contrast: 150 - 20 = 130 counts
7 ASSERT-EQ   \ Best slot is 7

\ ------------------------------------------------------------------------------
\ Test Group 2: CCSDS Attached Sync Marker Correlator
\ ------------------------------------------------------------------------------

S" Sync marker: exact 32-bit marker has zero Hamming distance" TEST-CASE-START
CCSDS-ASM-32 SYNC-COMPUTE-DISTANCE
0 ASSERT-EQ

S" Sync marker: 1-bit flipped marker has Hamming distance 1" TEST-CASE-START
CCSDS-ASM-32 1 XOR SYNC-COMPUTE-DISTANCE
1 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 3: Galois Field Arithmetic and Reed-Solomon Codec
\ ------------------------------------------------------------------------------

S" Galois Field GF(2^8): multiplicative inverse identity a * inv(a) = 1" TEST-CASE-START
GF-TABLES-INIT
1 DUP GF-INV GF-MUL 1 ASSERT-EQ
42 DUP GF-INV GF-MUL 1 ASSERT-EQ
137 DUP GF-INV GF-MUL 1 ASSERT-EQ
254 DUP GF-INV GF-MUL 1 ASSERT-EQ

CREATE TEST-MSG-BUFFER 223 ALLOT
CREATE TEST-PARITY-BUFFER 32 ALLOT
CREATE TEST-CODEWORD 255 ALLOT

: SETUP-RS-TEST-MESSAGE ( -- )
    TEST-MSG-BUFFER 223 ERASE
    \ Populate message with deterministic non-zero test pattern
    223 0 DO
        I 1+ TEST-MSG-BUFFER I + C!
    LOOP
;

S" Reed-Solomon: encode 223-byte message and produce 32 parity bytes" TEST-CASE-START
RS-CODEC-INIT
SETUP-RS-TEST-MESSAGE
TEST-MSG-BUFFER TEST-PARITY-BUFFER RS-ENCODE-FRAME
\ Verify non-zero parity output
TEST-PARITY-BUFFER C@ 0<> ASSERT-TRUE
TEST-PARITY-BUFFER 31 + C@ 0<> ASSERT-TRUE

S" Reed-Solomon: uncorrupted codeword passes decoding with 0 errors" TEST-CASE-START
\ Assemble full 255-byte codeword: [ message (223) | parity (32) ]
TEST-MSG-BUFFER TEST-CODEWORD 223 MOVE
TEST-PARITY-BUFFER TEST-CODEWORD 223 + 32 MOVE
TEST-CODEWORD RS-DECODE-FRAME
\ Returns ( corrected_count valid_flag )
1 ASSERT-EQ \ valid flag
0 ASSERT-EQ \ 0 errors corrected

\ ------------------------------------------------------------------------------
\ Test Group 4: Matrix Block Interleaver Invariance
\ ------------------------------------------------------------------------------

CREATE RAW-BURST-DATA 2040 ALLOT
CREATE INTERLEAVED-DATA 2040 ALLOT
CREATE RECOVERED-DATA 2040 ALLOT

: SETUP-INTERLEAVER-TEST-DATA ( -- )
    RAW-BURST-DATA 2040 ERASE
    2040 0 DO
        I 251 MOD RAW-BURST-DATA I + C!
    LOOP
;

S" Matrix block interleaver: interleave and deinterleave round-trip identity" TEST-CASE-START
INTERLEAVE-INIT
8 INTERLEAVE-SET-DEPTH ASSERT-TRUE
SETUP-INTERLEAVER-TEST-DATA

RAW-BURST-DATA INTERLEAVED-DATA INTERLEAVE-BLOCK
INTERLEAVED-DATA RECOVERED-DATA DEINTERLEAVE-BLOCK

\ Verify first, middle, and last bytes match
RAW-BURST-DATA C@ RECOVERED-DATA C@ ASSERT-EQ
RAW-BURST-DATA 500 + C@ RECOVERED-DATA 500 + C@ ASSERT-EQ
RAW-BURST-DATA 1000 + C@ RECOVERED-DATA 1000 + C@ ASSERT-EQ
RAW-BURST-DATA 2039 + C@ RECOVERED-DATA 2039 + C@ ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 5: CCSDS Space Packet Protocol Construction
\ ------------------------------------------------------------------------------

CREATE TEST-SPP-BUFFER 128 ALLOT

S" Space Packet: header construction and field serialization" TEST-CASE-START
TEST-SPP-BUFFER 128 ERASE
\ Build packet: APID = $010, is_tc = 0, flags = UNSEGMENTED (3), seq = 42, payload_len = 20
APID-PAYLOAD-HEALTH 0 SPP-UNSEGMENTED 42 20 TEST-SPP-BUFFER SPP-BUILD-HEADER
\ Check length in header bytes 4..5: len - 1 = 19 ($13)
TEST-SPP-BUFFER 5 + C@ 19 ASSERT-EQ
TEST-SPP-BUFFER 4 + C@ 0 ASSERT-EQ
\ Check sequence counter byte 3: 42
TEST-SPP-BUFFER 3 + C@ 42 ASSERT-EQ
\ Check APID in byte 1: $10
TEST-SPP-BUFFER 1 + C@ $10 ASSERT-EQ
