\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/fsm_filters.fs
\ Description: Digital biquad IIR notch and low-pass filters for suppressing
\              FSM structural resonance peaks (850 Hz mirror bending mode).
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Biquad Filter Direct Form II Transposed Memory Layout (7 cells = 56 bytes):
\   Offset +0:  b0 (Q32.32)
\   Offset +8:  b1 (Q32.32)
\   Offset +16: b2 (Q32.32)
\   Offset +24: a1 (Q32.32)
\   Offset +32: a2 (Q32.32)
\   Offset +40: d1 (state delay 1)
\   Offset +48: d2 (state delay 2)

56 CONSTANT BIQUAD-BYTES

\ Accessors
: BQ-B0 ( addr -- ptr ) @ ;
: BQ-B1 ( addr -- ptr ) CELL+ @ ;
: BQ-B2 ( addr -- ptr ) 2 CELLS + @ ;
: BQ-A1 ( addr -- ptr ) 3 CELLS + @ ;
: BQ-A2 ( addr -- ptr ) 4 CELLS + @ ;
: BQ-D1 ( addr -- ptr ) 5 CELLS + ;
: BQ-D2 ( addr -- ptr ) 6 CELLS + ;

\ ------------------------------------------------------------------------------
\ Section 1: Generic Biquad Filter Engine
\ ------------------------------------------------------------------------------

\ BIQUAD-INIT ( addr b0 b1 b2 a1 a2 -- )
\ Initialize biquad coefficients and clear delay states.
: BIQUAD-INIT ( addr b0 b1 b2 a1 a2 -- )
    >R >R >R >R >R          \ R: a2 a1 b2 b1 b0, D: addr
    DUP R> SWAP !           \ b0
    DUP R> SWAP CELL+ !     \ b1
    DUP R> SWAP 2 CELLS + ! \ b2
    DUP R> SWAP 3 CELLS + ! \ a1
    DUP R> SWAP 4 CELLS + ! \ a2
    0 OVER 5 CELLS + !      \ d1 = 0
    0 OVER 6 CELLS + !      \ d2 = 0
    DROP
;

\ BIQUAD-RESET ( addr -- )
\ Clear delay registers d1 and d2.
: BIQUAD-RESET ( addr -- )
    0 OVER 5 CELLS + !
    0 SWAP 6 CELLS + !
;

VARIABLE BQ-X
VARIABLE BQ-Y

\ BIQUAD-STEP ( x_q32 addr -- y_q32 )
\ Execute one discrete time step of Direct Form II Transposed biquad filter.
: BIQUAD-STEP ( x addr -- y )
    >R \ R: addr
    BQ-X !

    \ y[n] = b0 * x[n] + d1[n-1]
    R@ BQ-B0 BQ-X @ Q32*
    R@ BQ-D1 @ +
    BQ-Y !

    \ d1[n] = b1 * x[n] - a1 * y[n] + d2[n-1]
    R@ BQ-B1 BQ-X @ Q32*
    R@ BQ-A1 BQ-Y @ Q32* -
    R@ BQ-D2 @ +
    R@ BQ-D1 !

    \ d2[n] = b2 * x[n] - a2 * y[n]
    R@ BQ-B2 BQ-X @ Q32*
    R@ BQ-A2 BQ-Y @ Q32* -
    R@ BQ-D2 !

    R> DROP
    BQ-Y @
;

\ ------------------------------------------------------------------------------
\ Section 2: FSM 850 Hz Structural Notch Filter Precomputed Instances
\ ------------------------------------------------------------------------------

\ Notch Filter Parameters (f_notch = 850 Hz, Q = 5.0, f_sample = 2000 Hz)
\ Discrete coefficients in Q32.32:
\   b0 = 0.88412 * 2^32 = 3797282876
\   b1 = -0.73523 * 2^32 = -3157774844
\   b2 = 0.88412 * 2^32 = 3797282876
\   a1 = -0.73523 * 2^32 = -3157774844
\   a2 = 0.76824 * 2^32 = 3299557451
3797282876 CONSTANT NOTCH-B0
-3157774844 CONSTANT NOTCH-B1
3797282876 CONSTANT NOTCH-B2
-3157774844 CONSTANT NOTCH-A1
3299557451 CONSTANT NOTCH-A2

CREATE FSM-NOTCH-TIP 56 ALLOT
CREATE FSM-NOTCH-TILT 56 ALLOT

\ FSM-FILTERS-INIT ( -- )
: FSM-FILTERS-INIT ( -- )
    FSM-NOTCH-TIP NOTCH-B0 NOTCH-B1 NOTCH-B2 NOTCH-A1 NOTCH-A2 BIQUAD-INIT
    FSM-NOTCH-TILT NOTCH-B0 NOTCH-B1 NOTCH-B2 NOTCH-A1 NOTCH-A2 BIQUAD-INIT
;

\ FSM-NOTCH-TIP-STEP ( x -- y )
: FSM-NOTCH-TIP-STEP ( x -- y )
    FSM-NOTCH-TIP BIQUAD-STEP
;

\ FSM-NOTCH-TILT-STEP ( x -- y )
: FSM-NOTCH-TILT-STEP ( x -- y )
    FSM-NOTCH-TILT BIQUAD-STEP
;
