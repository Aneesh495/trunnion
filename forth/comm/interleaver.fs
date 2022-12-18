\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/interleaver.fs
\ Description: Matrix block interleaver and de-interleaver for optical fading
\              burst error mitigation across Reed-Solomon codewords.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Interleaver Depth and Buffer Storage
\ ------------------------------------------------------------------------------

VARIABLE INTERLEAVE-DEPTH       \ I = 1, 2, 4, 8
8 CONSTANT INTERLEAVE-MAX-DEPTH
255 CONSTANT INTERLEAVE-BLOCK-SIZE

\ Total buffer: 8 * 255 = 2040 bytes
CREATE INTERLEAVE-BUF 2048 ALLOT
CREATE DEINTERLEAVE-BUF 2048 ALLOT

\ ------------------------------------------------------------------------------
\ Section 2: Configuration and Reset
\ ------------------------------------------------------------------------------

\ INTERLEAVE-SET-DEPTH ( depth -- success_flag )
\ Set interleaving depth (1, 2, 4, or 8).
: INTERLEAVE-SET-DEPTH ( depth -- flag )
    DUP 1 = OVER 2 = OR OVER 4 = OR OVER 8 = OR IF
        INTERLEAVE-DEPTH !
        1
    ELSE
        DROP 0
    THEN
;

\ INTERLEAVE-INIT ( -- )
: INTERLEAVE-INIT ( -- )
    4 INTERLEAVE-SET-DEPTH DROP \ Default depth = 4
    INTERLEAVE-BUF 2048 ERASE
    DEINTERLEAVE-BUF 2048 ERASE
;

\ ------------------------------------------------------------------------------
\ Section 3: Interleave and De-Interleave Matrix Transforms
\ ------------------------------------------------------------------------------

\ INTERLEAVE-BLOCK ( src_addr dst_addr -- )
\ Write I codewords row-by-row, read out column-by-column.
: INTERLEAVE-BLOCK ( src dst -- )
    >R >R
    INTERLEAVE-BLOCK-SIZE 0 DO
        INTERLEAVE-DEPTH @ 0 DO
            \ src index = J * 255 + I
            J INTERLEAVE-BLOCK-SIZE * I +
            2R@ SWAP DROP + C@

            \ dst index = I * depth + J
            I INTERLEAVE-DEPTH @ * J +
            R@ + C!
        LOOP
    LOOP
    2R> 2DROP
;

\ DEINTERLEAVE-BLOCK ( src_addr dst_addr -- )
\ Inverse transform: write column-by-column, read out row-by-row.
: DEINTERLEAVE-BLOCK ( src dst -- )
    >R >R
    INTERLEAVE-DEPTH @ 0 DO
        INTERLEAVE-BLOCK-SIZE 0 DO
            \ src index = J * depth + I
            J INTERLEAVE-DEPTH @ * I +
            2R@ SWAP DROP + C@

            \ dst index = I * 255 + J
            I INTERLEAVE-BLOCK-SIZE * J +
            R@ + C!
        LOOP
    LOOP
    2R> 2DROP
;
