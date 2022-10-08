\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/cordic.fs
\ Description: 32-iteration CORDIC (Coordinate Rotation Digital Computer)
\              trigonometric and vectoring engine in signed Q32.32 fixed-point.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ CORDIC algorithm iteratively rotates a 2D vector by elementary angles:
\   x_{i+1} = x_i - d_i * (y_i >> i)
\   y_{i+1} = y_i + d_i * (x_i >> i)
\   z_{i+1} = z_i - d_i * atan(2^-i)
\ Where d_i = +1 if z_i >= 0 (in rotation mode) else -1.
\ Aggregate scale factor K = prod(sqrt(1 + 2^(-2i))) = 1.646760258121066
\ Inverse gain 1/K = 0.607252935008881 = 2608097897 in Q32.32.

\ ------------------------------------------------------------------------------
\ Section 1: CORDIC Elementary Angle Table (32 Iterations in Q32.32)
\ ------------------------------------------------------------------------------

2608097897 CONSTANT CORDIC-INV-K
32 CONSTANT CORDIC-ITERATIONS

CREATE CORDIC-ATAN-TABLE
    3373259426 , \ i=0:  atan(2^0)   = 45.00000000 deg = 0.785398163 rad
    1991484084 , \ i=1:  atan(2^-1)  = 26.56505118 deg = 0.463647609 rad
    1052028682 , \ i=2:  atan(2^-2)  = 14.03624347 deg = 0.244978663 rad
     533966567 , \ i=3:  atan(2^-3)  =  7.12501635 deg = 0.124354995 rad
     268249826 , \ i=4:  atan(2^-4)  =  3.57633437 deg = 0.062418810 rad
     134280590 , \ i=5:  atan(2^-5)  =  1.78991061 deg = 0.031239833 rad
      67159789 , \ i=6:  atan(2^-6)  =  0.89517371 deg = 0.015623729 rad
      33582455 , \ i=7:  atan(2^-7)  =  0.44761417 deg = 0.007812341 rad
      16791552 , \ i=8:  atan(2^-8)  =  0.22381050 deg = 0.003906230 rad
       8395817 , \ i=9:  atan(2^-9)  =  0.11190568 deg = 0.001953123 rad
       4197914 , \ i=10: atan(2^-10) =  0.05595289 deg = 0.000976562 rad
       2098958 , \ i=11: atan(2^-11) =  0.02797645 deg = 0.000488281 rad
       1049479 , \ i=12: atan(2^-12) =  0.01398823 deg = 0.000244141 rad
        524740 , \ i=13: atan(2^-13) =  0.00699411 deg = 0.000122070 rad
        262370 , \ i=14: atan(2^-14) =  0.00349706 deg = 0.000061035 rad
        131185 , \ i=15: atan(2^-15) =  0.00174853 deg = 0.000030518 rad
         65592 , \ i=16: atan(2^-16) =  0.00087426 deg = 0.000015259 rad
         32796 , \ i=17: atan(2^-17) =  0.00043713 deg = 0.000007629 rad
         16398 , \ i=18: atan(2^-18) =  0.00021857 deg = 0.000003815 rad
          8199 , \ i=19: atan(2^-19) =  0.00010928 deg = 0.000001907 rad
          4100 , \ i=20: atan(2^-20) =  0.00005464 deg = 0.000000954 rad
          2050 , \ i=21: atan(2^-21) =  0.00002732 deg = 0.000000477 rad
          1025 , \ i=22: atan(2^-22) =  0.00001366 deg = 0.000000238 rad
           512 , \ i=23: atan(2^-23) =  0.00000683 deg = 0.000000119 rad
           256 , \ i=24: atan(2^-24) =  0.00000342 deg = 0.000000060 rad
           128 , \ i=25: atan(2^-25) =  0.00000171 deg = 0.000000030 rad
            64 , \ i=26: atan(2^-26) =  0.00000085 deg = 0.000000015 rad
            32 , \ i=27: atan(2^-27) =  0.00000043 deg = 0.000000007 rad
            16 , \ i=28: atan(2^-28) =  0.00000021 deg = 0.000000004 rad
             8 , \ i=29: atan(2^-29) =  0.00000011 deg = 0.000000002 rad
             4 , \ i=30: atan(2^-30) =  0.00000005 deg = 0.000000001 rad
             2 , \ i=31: atan(2^-31) =  0.00000003 deg = 0.000000000 rad

\ CORDIC-ATAN@ ( i -- angle_q32 )
: CORDIC-ATAN@ ( i -- angle_q32 )
    CELLS CORDIC-ATAN-TABLE + @
;

\ Working Registers
VARIABLE CORDIC-X
VARIABLE CORDIC-Y
VARIABLE CORDIC-Z
VARIABLE CORDIC-I

\ ------------------------------------------------------------------------------
\ Section 2: CORDIC Vector Rotation Engine (Sin / Cos)
\ ------------------------------------------------------------------------------

: CORDIC-ROTATE-STEP ( i -- )
    CORDIC-I !
    CORDIC-Z @ 0< IF
        \ d_i = -1:
        \ x_new = x + (y >> i)
        \ y_new = y - (x >> i)
        \ z_new = z + atan(i)
        CORDIC-X @ CORDIC-Y @ CORDIC-I @ ARSHIFT +
        CORDIC-Y @ CORDIC-X @ CORDIC-I @ ARSHIFT -
        CORDIC-Z @ CORDIC-I @ CORDIC-ATAN@ +
    ELSE
        \ d_i = +1:
        \ x_new = x - (y >> i)
        \ y_new = y + (x >> i)
        \ z_new = z - atan(i)
        CORDIC-X @ CORDIC-Y @ CORDIC-I @ ARSHIFT -
        CORDIC-Y @ CORDIC-X @ CORDIC-I @ ARSHIFT +
        CORDIC-Z @ CORDIC-I @ CORDIC-ATAN@ -
    THEN
    CORDIC-Z !
    CORDIC-Y !
    CORDIC-X !
;

\ CORDIC-ROTATE-CORE ( x0 y0 z0 -- x y z )
: CORDIC-ROTATE-CORE ( x0 y0 z0 -- x y z )
    CORDIC-Z !
    CORDIC-Y !
    CORDIC-X !
    32 0 DO
        I CORDIC-ROTATE-STEP
    LOOP
    CORDIC-X @
    CORDIC-Y @
    CORDIC-Z @
;

\ NORMALIZE-ANGLE-Q32 ( angle -- norm_angle sign_cos sign_sin )
\ Normalize arbitrary angle to principal range [-pi/2, +pi/2].
: NORMALIZE-ANGLE-Q32 ( angle -- norm_angle sign_cos sign_sin )
    \ Wrap into [-pi, +pi]
    BEGIN
        DUP Q32-PI >
    WHILE
        Q32-TWO-PI -
    REPEAT
    BEGIN
        DUP Q32-PI NEGATE <
    WHILE
        Q32-TWO-PI +
    REPEAT

    \ Check quadrants
    DUP Q32-HALF-PI > IF
        Q32-PI - -1 1 EXIT
    THEN
    DUP Q32-HALF-PI NEGATE < IF
        Q32-PI + -1 1 EXIT
    THEN
    1 1
;

\ CORDIC-SIN-COS ( angle_q32 -- sin_q32 cos_q32 )
\ Compute sine and cosine simultaneously in Q32.32 format.
: CORDIC-SIN-COS ( angle_q32 -- sin_q32 cos_q32 )
    NORMALIZE-ANGLE-Q32 >R >R
    \ Initial vector: x0 = 1/K, y0 = 0, z0 = angle
    CORDIC-INV-K 0 ROT
    CORDIC-ROTATE-CORE
    DROP \ Discard residual z
    \ Stack: ( cos sin )
    R> 0< IF NEGATE THEN \ Apply sign_cos
    SWAP
    R> 0< IF NEGATE THEN \ Apply sign_sin
    SWAP
;

\ CORDIC-SIN ( angle_q32 -- sin_q32 )
: CORDIC-SIN ( angle_q32 -- sin_q32 )
    CORDIC-SIN-COS DROP
;

\ CORDIC-COS ( angle_q32 -- cos_q32 )
: CORDIC-COS ( angle_q32 -- cos_q32 )
    CORDIC-SIN-COS NIP
;

\ CORDIC-TAN ( angle_q32 -- tan_q32 )
: CORDIC-TAN ( angle_q32 -- tan_q32 )
    CORDIC-SIN-COS SWAP OVER Q32/
;

\ ------------------------------------------------------------------------------
\ Section 3: CORDIC Vectoring Engine (Atan2 / Magnitude)
\ ------------------------------------------------------------------------------

: CORDIC-VECTOR-STEP ( i -- )
    CORDIC-I !
    CORDIC-Y @ 0< IF
        \ y < 0: rotate counterclockwise (d_i = -1)
        CORDIC-X @ CORDIC-Y @ CORDIC-I @ ARSHIFT -
        CORDIC-Y @ CORDIC-X @ CORDIC-I @ ARSHIFT +
        CORDIC-Z @ CORDIC-I @ CORDIC-ATAN@ -
    ELSE
        \ y >= 0: rotate clockwise (d_i = +1)
        CORDIC-X @ CORDIC-Y @ CORDIC-I @ ARSHIFT +
        CORDIC-Y @ CORDIC-X @ CORDIC-I @ ARSHIFT -
        CORDIC-Z @ CORDIC-I @ CORDIC-ATAN@ +
    THEN
    CORDIC-Z !
    CORDIC-Y !
    CORDIC-X !
;

\ CORDIC-VECTOR-CORE ( x0 y0 z0 -- x y z )
: CORDIC-VECTOR-CORE ( x0 y0 z0 -- x y z )
    CORDIC-Z !
    CORDIC-Y !
    CORDIC-X !
    32 0 DO
        I CORDIC-VECTOR-STEP
    LOOP
    CORDIC-X @
    CORDIC-Y @
    CORDIC-Z @
;

\ CORDIC-ATAN2 ( y_q32 x_q32 -- angle_q32 )
\ Compute 4-quadrant arctangent atan2(y, x) in Q32.32 radians.
: CORDIC-ATAN2 ( y x -- angle )
    DUP 0= IF
        DROP
        DUP 0> IF DROP Q32-HALF-PI EXIT THEN
        DUP 0< IF DROP Q32-HALF-PI NEGATE EXIT THEN
        DROP 0 EXIT
    THEN

    \ Check for negative x (quadrants 2 and 3)
    DUP 0< IF
        NEGATE SWAP NEGATE SWAP
        0 CORDIC-VECTOR-CORE
        NIP NIP
        SWAP 0< IF Q32-PI - ELSE Q32-PI + THEN
    ELSE
        0 CORDIC-VECTOR-CORE
        NIP NIP
    THEN
;

\ CORDIC-MAGNITUDE ( y_q32 x_q32 -- mag_q32 )
\ Compute Euclidean vector magnitude sqrt(x^2 + y^2) in Q32.32.
: CORDIC-MAGNITUDE ( y x -- mag )
    ABS SWAP ABS 0
    CORDIC-VECTOR-CORE
    DROP DROP
    \ Compensate for CORDIC gain K by multiplying by 1/K
    CORDIC-INV-K Q32*
;

\ CORDIC-SQRT ( x_q32 -- sqrt_q32 )
: CORDIC-SQRT ( x -- sqrt_x )
    DUP 0<= IF DROP 0 EXIT THEN
    DUP S>F FSQRT F>S S>Q32
;
