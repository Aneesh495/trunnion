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

VARIABLE IL-SRC
VARIABLE IL-DST

\ INTERLEAVE-BLOCK ( src_addr dst_addr -- )
\ Write I codewords row-by-row, read out column-by-column.
: INTERLEAVE-BLOCK ( src dst -- )
    IL-DST !
    IL-SRC !
    INTERLEAVE-BLOCK-SIZE 0 DO
        INTERLEAVE-DEPTH @ 0 DO
            \ src index = row * 255 + col = I * 255 + J
            IL-SRC @ I INTERLEAVE-BLOCK-SIZE * J + + C@

            \ dst index = col * depth + row = J * depth + I
            IL-DST @ J INTERLEAVE-DEPTH @ * I + + C!
        LOOP
    LOOP
;

\ DEINTERLEAVE-BLOCK ( src_addr dst_addr -- )
\ Inverse transform: write column-by-column, read out row-by-row.
: DEINTERLEAVE-BLOCK ( src dst -- )
    IL-DST !
    IL-SRC !
    INTERLEAVE-BLOCK-SIZE 0 DO
        INTERLEAVE-DEPTH @ 0 DO
            \ src index = col * depth + row = J * depth + I
            IL-SRC @ J INTERLEAVE-DEPTH @ * I + + C@

            \ dst index = row * 255 + col = I * 255 + J
            IL-DST @ I INTERLEAVE-BLOCK-SIZE * J + + C!
        LOOP
    LOOP
;
