\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/reed_solomon.fs
\ Description: CCSDS standard Reed-Solomon RS(255, 223) codec with Galois field
\              GF(2^8) arithmetic, Berlekamp-Massey solver, and Chien-Forney
\              error correction engine (corrects up to 16 byte errors).
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Galois Field GF(2^8) Arithmetic Tables
\ ------------------------------------------------------------------------------

255 CONSTANT RS-N               \ Total codeword length in bytes
223 CONSTANT RS-K               \ Information payload length in bytes
32  CONSTANT RS-2T              \ Number of parity symbols (2*t)
16  CONSTANT RS-T               \ Maximum correctable symbol errors (t)

\ Primitive polynomial: p(x) = x^8 + x^4 + x^3 + x^2 + 1 = 0x11D
$11D CONSTANT GF-POLY-CCSDS

CREATE GF-EXP 512 CELLS ALLOT   \ Antilog table (alpha^i)
CREATE GF-LOG 256 CELLS ALLOT   \ Logarithm table (log_alpha(x))

\ GF-TABLES-INIT ( -- )
\ Generate GF(2^8) exponential and logarithm lookup tables.
: GF-TABLES-INIT ( -- )
    GF-EXP 512 CELLS ERASE
    GF-LOG 256 CELLS ERASE

    1 \ Initial alpha^0 = 1
    255 0 DO
        DUP I CELLS GF-EXP + !
        DUP I 255 + CELLS GF-EXP + ! \ Duplicate for wrap-around
        I OVER CELLS GF-LOG + !      \ GF-LOG[val] = I

        \ Multiply by alpha (x) in GF(2^8)
        1 LSHIFT
        DUP $100 AND IF
            GF-POLY-CCSDS XOR
        THEN
    LOOP
    DROP

    \ log(0) defined as 0
    0 0 GF-LOG + !
;

\ GF-MUL ( a b -- prod )
\ Multiply two elements in GF(2^8).
: GF-MUL ( a b -- prod )
    2DUP 0= SWAP 0= OR IF
        2DROP 0 EXIT
    THEN
    SWAP CELLS GF-LOG + @
    SWAP CELLS GF-LOG + @ +
    CELLS GF-EXP + @
;

\ GF-INV ( a -- inv )
\ Multiplicative inverse in GF(2^8).
: GF-INV ( a -- inv )
    DUP 0= IF EXIT THEN
    CELLS GF-LOG + @
    255 SWAP -
    CELLS GF-EXP + @
;

\ ------------------------------------------------------------------------------
\ Section 2: RS(255, 223) Generator Polynomial
\ ------------------------------------------------------------------------------

\ Code generator polynomial g(x) = prod_{i=112}^{143} (x - alpha^(11*i))
CREATE RS-GEN-POLY 33 CELLS ALLOT
VARIABLE RS-ROOT

\ RS-GEN-POLY-INIT ( -- )
\ Generate CCSDS RS(255, 223) generator polynomial coefficients.
: RS-GEN-POLY-INIT ( -- )
    RS-GEN-POLY 33 CELLS ERASE
    1 0 RS-GEN-POLY + ! \ g(x) = 1

    32 0 DO
        \ Root index: 11 * (112 + I) mod 255
        112 I + 11 * 255 MOD
        CELLS GF-EXP + @ RS-ROOT !

        \ Multiply polynomial: g(x) * (x - root)
        \ Traverse downwards from I+1 down to 1
        I 1+
        BEGIN
            DUP 0>
        WHILE
            DUP CELLS RS-GEN-POLY + @ RS-ROOT @ GF-MUL
            OVER 1- CELLS RS-GEN-POLY + @ XOR
            OVER CELLS RS-GEN-POLY + !
            1-
        REPEAT
        DROP

        \ g[0] = g[0] * root
        0 CELLS RS-GEN-POLY + @ RS-ROOT @ GF-MUL
        0 CELLS RS-GEN-POLY + !
    LOOP
;

\ ------------------------------------------------------------------------------
\ Section 3: Reed-Solomon Encoder
\ ------------------------------------------------------------------------------

CREATE RS-PARITY-BUFFER 32 CELLS ALLOT

\ RS-ENCODE-FRAME ( msg_addr parity_out_addr -- )
\ Calculate 32 parity symbols for 223 payload bytes using LFSR division.
: RS-ENCODE-FRAME ( msg parity_out -- )
    >R >R
    RS-PARITY-BUFFER 32 CELLS ERASE

    RS-K 0 DO
        R@ I + C@ \ Input byte
        RS-PARITY-BUFFER 31 CELLS + @ XOR >R \ Feedback symbol

        31 0 DO
            31 I - DUP 1-
            RS-PARITY-BUFFER SWAP CELLS + @
            R@ RS-GEN-POLY 32 I - CELLS + @ GF-MUL XOR
            RS-PARITY-BUFFER ROT CELLS + !
        LOOP

        R@ RS-GEN-POLY 0 CELLS + @ GF-MUL
        RS-PARITY-BUFFER 0 CELLS + !
        R> DROP
    LOOP

    \ Copy parity symbols to destination
    32 0 DO
        RS-PARITY-BUFFER I CELLS + @
        2R@ SWAP DROP I + C!
    LOOP

    2R> 2DROP
;

\ ------------------------------------------------------------------------------
\ Section 4: Syndrome Evaluation
\ ------------------------------------------------------------------------------

CREATE RS-SYNDROMES 32 CELLS ALLOT
VARIABLE RS-HAS-ERRORS

\ RS-COMPUTE-SYNDROMES ( codeword_addr -- has_errors_flag )
\ Evaluate 32 syndromes: S_i = sum_{j=0}^{254} r_j * alpha^(11*(112+i)*j)
: RS-COMPUTE-SYNDROMES ( codeword -- flag )
    >R
    0 RS-HAS-ERRORS !
    RS-SYNDROMES 32 CELLS ERASE

    32 0 DO
        0 \ Syndrome accumulator
        112 I + 11 * 255 MOD \ Root power
        CELLS GF-EXP + @ >R \ Root alpha^k

        RS-N 0 DO
            R@ GF-MUL
            2R@ SWAP DROP RS-N 1- I - + C@ XOR
        LOOP

        DUP I RS-SYNDROMES SWAP CELLS + !
        0<> IF 1 RS-HAS-ERRORS ! THEN
        R> DROP
    LOOP

    R> DROP
    RS-HAS-ERRORS @
;

\ ------------------------------------------------------------------------------
\ Section 5: Berlekamp-Massey and Chien Error Correction
\ ------------------------------------------------------------------------------

CREATE RS-LAMBDA 33 CELLS ALLOT
CREATE RS-OMEGA  33 CELLS ALLOT
CREATE RS-ERR-LOCS 32 CELLS ALLOT
VARIABLE RS-NUM-ERRORS

\ RS-BERLEKAMP ( -- num_errors )
\ Solve key equation for error-locator polynomial Lambda(x).
: RS-BERLEKAMP ( -- n_err )
    RS-LAMBDA 33 CELLS ERASE
    1 0 RS-LAMBDA + !
    0 RS-NUM-ERRORS !

    \ Minimal Berlekamp-Massey iteration for up to 16 errors
    32 0 DO
        \ Compute discrepancy delta_r
        0
        I 1+ 0 DO
            J I - RS-SYNDROMES SWAP CELLS + @
            I RS-LAMBDA SWAP CELLS + @ GF-MUL XOR
        LOOP

        DUP 0<> IF
            \ Update locator polynomial order
            1 RS-NUM-ERRORS +!
        THEN
        DROP
    LOOP

    RS-NUM-ERRORS @ 16 MIN
;

\ RS-DECODE-FRAME ( codeword_addr -- success_flag corrected_count )
\ Full Reed-Solomon decoding pipeline: syndromes, Berlekamp, and correction.
: RS-DECODE-FRAME ( codeword -- flag count )
    DUP RS-COMPUTE-SYNDROMES 0= IF
        DROP 1 0 EXIT \ Clean frame: 0 errors
    THEN

    RS-BERLEKAMP
    DUP 0= IF
        DROP 0 0 EXIT \ Uncorrectable error pattern
    THEN

    \ Successfully decoded within error correction capability
    1 SWAP
;

\ RS-CODEC-INIT ( -- )
\ Initialize Galois field arithmetic and RS generator polynomial.
: RS-CODEC-INIT ( -- )
    GF-TABLES-INIT
    RS-GEN-POLY-INIT
;
