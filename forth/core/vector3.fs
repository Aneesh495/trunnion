\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/vector3.fs
\ Description: 3D Cartesian vector mathematics in Q32.32 signed fixed-point
\              for satellite line-of-sight geometry and gimbal coordinates.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ 3D Vector Stack convention: ( x y z ) with z on top.
\ 3D Vector Memory format: 3 contiguous 64-bit cells (24 bytes):
\   Offset +0:  x (Q32.32)
\   Offset +8:  y (Q32.32)
\   Offset +16: z (Q32.32)

24 CONSTANT VEC3-BYTES

\ ------------------------------------------------------------------------------
\ Section 1: Memory Load / Store Operations
\ ------------------------------------------------------------------------------

\ V3@ ( addr -- x y z )
\ Load 3D vector from memory address onto stack.
: V3@ ( addr -- x y z )
    DUP @
    OVER CELL+ @
    ROT 2 CELLS + @
;

\ V3! ( x y z addr -- )
\ Store 3D vector from stack into memory address.
: V3! ( x y z addr -- )
    >R
    R@ 2 CELLS + !
    R@ CELL+ !
    R> !
;

\ V3-ZERO ( addr -- )
\ Clear 3D vector at addr to (0, 0, 0).
: V3-ZERO ( addr -- )
    DUP 0 SWAP !
    DUP CELL+ 0 SWAP !
    2 CELLS + 0 SWAP !
;

\ V3-COPY ( src_addr dst_addr -- )
\ Copy 3D vector from src_addr to dst_addr.
: V3-COPY ( src dst -- )
    VEC3-BYTES MOVE
;

\ ------------------------------------------------------------------------------
\ Section 2: Stack Vector Arithmetic
\ ------------------------------------------------------------------------------

\ V3+ ( x1 y1 z1 x2 y2 z2 -- x1+x2 y1+y2 z1+z2 )
\ Element-wise addition of two 3D vectors.
: V3+ ( x1 y1 z1 x2 y2 z2 -- x3 y3 z3 )
    >R >R >R
    ROT R> + -ROT
    SWAP R> + SWAP
    R> +
;

\ V3- ( x1 y1 z1 x2 y2 z2 -- x1-x2 y1-y2 z1-z2 )
\ Element-wise subtraction of two 3D vectors.
: V3- ( x1 y1 z1 x2 y2 z2 -- x3 y3 z3 )
    >R >R >R
    ROT R> - -ROT
    SWAP R> - SWAP
    R> -
;

\ V3-NEG ( x y z -- -x -y -z )
\ Negate all components of 3D vector.
: V3-NEG ( x y z -- -x -y -z )
    NEGATE >R NEGATE SWAP NEGATE SWAP R>
;

VARIABLE V3-SCALE-REG

\ V3*SCALE ( x y z scale_q32 -- x*s y*s z*s )
\ Multiply 3D vector by scalar in Q32.32 format.
: V3*SCALE ( x y z s -- x' y' z' )
    V3-SCALE-REG !
    V3-SCALE-REG @ Q32* >R
    V3-SCALE-REG @ Q32* >R
    V3-SCALE-REG @ Q32*
    R> R>
;

\ V3/SCALE ( x y z scale_q32 -- x/s y/s z/s )
\ Divide 3D vector by scalar in Q32.32 format.
: V3/SCALE ( x y z s -- x' y' z' )
    DUP 0= IF DROP EXIT THEN
    V3-SCALE-REG !
    V3-SCALE-REG @ Q32/ >R
    V3-SCALE-REG @ Q32/ >R
    V3-SCALE-REG @ Q32/
    R> R>
;

\ ------------------------------------------------------------------------------
\ Section 3: Inner Product, Cross Product, and Norms
\ ------------------------------------------------------------------------------

\ V3-DOT ( x1 y1 z1 x2 y2 z2 -- dot_q32 )
\ Compute dot product (v1 . v2) in Q32.32 format.
: V3-DOT ( x1 y1 z1 x2 y2 z2 -- dot )
    >R >R >R
    ROT R> Q32*
    ROT R> Q32* +
    SWAP R> Q32* +
;

VARIABLE V3-A-X
VARIABLE V3-A-Y
VARIABLE V3-A-Z
VARIABLE V3-B-X
VARIABLE V3-B-Y
VARIABLE V3-B-Z

\ V3-CROSS ( x1 y1 z1 x2 y2 z2 -- cx cy cz )
\ Compute vector cross product (v1 x v2):
\   cx = y1*z2 - z1*y2
\   cy = z1*x2 - x1*z2
\   cz = x1*y2 - y1*x2
: V3-CROSS ( x1 y1 z1 x2 y2 z2 -- cx cy cz )
    V3-B-Z !
    V3-B-Y !
    V3-B-X !
    V3-A-Z !
    V3-A-Y !
    V3-A-X !
    \ cx = y1*z2 - z1*y2
    V3-A-Y @ V3-B-Z @ Q32*
    V3-A-Z @ V3-B-Y @ Q32* -
    \ cy = z1*x2 - x1*z2
    V3-A-Z @ V3-B-X @ Q32*
    V3-A-X @ V3-B-Z @ Q32* -
    \ cz = x1*y2 - y1*x2
    V3-A-X @ V3-B-Y @ Q32*
    V3-A-Y @ V3-B-X @ Q32* -
;

\ V3-NORM-SQ ( x y z -- norm_sq_q32 )
\ Compute squared Euclidean norm ||v||^2 = x^2 + y^2 + z^2.
: V3-NORM-SQ ( x y z -- norm_sq )
    DUP Q32* >R
    DUP Q32* >R
    DUP Q32*
    R> + R> +
;

\ V3-NORM ( x y z -- norm_q32 )
\ Compute Euclidean norm ||v|| = sqrt(x^2 + y^2 + z^2).
: V3-NORM ( x y z -- norm )
    V3-NORM-SQ CORDIC-SQRT
;

\ V3-NORMALIZE ( x y z -- ux uy uz )
\ Normalize vector to unit length. If zero vector, returns (0, 0, 0).
: V3-NORMALIZE ( x y z -- ux uy uz )
    3DUP V3-NORM
    DUP 0= IF
        DROP EXIT
    THEN
    V3/SCALE
;

\ V3-DISTANCE ( x1 y1 z1 x2 y2 z2 -- dist_q32 )
\ Compute Euclidean distance between two 3D points.
: V3-DISTANCE ( x1 y1 z1 x2 y2 z2 -- dist )
    V3- V3-NORM
;

\ V3-LERP ( x1 y1 z1 x2 y2 z2 t_q32 -- xt yt zt )
\ Linear interpolation: v1 + t*(v2 - v1).
: V3-LERP ( x1 y1 z1 x2 y2 z2 t -- xt yt zt )
    >R
    3DUP >R >R >R
    V3- R@ V3*SCALE
    R> R> R>
    V3+
    R> DROP
;

\ ------------------------------------------------------------------------------
\ Section 4: Projections and Geometric Angles
\ ------------------------------------------------------------------------------

\ V3-PROJ ( vx vy vz ux uy uz -- px py pz )
\ Project vector v onto unit vector u: (v . u) * u.
: V3-PROJ ( vx vy vz ux uy uz -- px py pz )
    4DUP >R >R >R >R >R >R
    V3-DOT
    R> R> R> R> R> R> 2DROP 2DROP DROP
    V3*SCALE
;

\ V3-REJECT ( vx vy vz ux uy uz -- rx ry rz )
\ Compute component of v perpendicular to unit vector u: v - proj_u(v).
: V3-REJECT ( vx vy vz ux uy uz -- rx ry rz )
    4DUP >R >R >R >R >R >R
    V3-PROJ
    R> R> R> R> R> R> 2DROP 2DROP DROP
    V3-
;

\ V3-ANGLE ( x1 y1 z1 x2 y2 z2 -- angle_rad_q32 )
\ Compute angle between two vectors in radians: acos((v1 . v2) / (||v1||*||v2||)).
: V3-ANGLE ( x1 y1 z1 x2 y2 z2 -- angle )
    3DUP V3-NORM >R
    4DUP >R >R >R
    V3-NORM R> * 32 RSHIFT \ Denominator
    R> R> R>
    V3-DOT SWAP Q32/ \ cos(theta)
    \ Clamp cos(theta) to [-1, +1]
    Q32-ONE NEGATE Q32-ONE CLAMP
    \ Angle = atan2(sqrt(1 - cos^2), cos)
    DUP DUP Q32* Q32-ONE SWAP -
    CORDIC-SQRT SWAP
    CORDIC-ATAN2
    R> DROP
;

\ ------------------------------------------------------------------------------
\ Section 5: Vector Formatting and Printing
\ ------------------------------------------------------------------------------

\ V3. ( x y z -- )
\ Print 3D vector formatted as [x, y, z].
: V3. ( x y z -- )
    [CHAR] [ EMIT SPACE
    ROT Q32. [CHAR] , EMIT SPACE
    SWAP Q32. [CHAR] , EMIT SPACE
    Q32. SPACE [CHAR] ] EMIT
;
