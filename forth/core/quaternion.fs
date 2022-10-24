\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/quaternion.fs
\ Description: Hamilton unit quaternion algebra, vector rotations, SLERP
\              interpolation, and DCM conversions in Q32.32 fixed-point.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Quaternion memory format: 4 contiguous 64-bit cells (32 bytes):
\   Offset +0:  qw (scalar component, cos(theta/2))
\   Offset +8:  qx (vector x component, ux * sin(theta/2))
\   Offset +16: qy (vector y component, uy * sin(theta/2))
\   Offset +24: qz (vector z component, uz * sin(theta/2))
\ Stack representation: ( qw qx qy qz ) with qz on top.

32 CONSTANT QUAT-BYTES

VARIABLE Q-TEMP-W1  VARIABLE Q-TEMP-X1  VARIABLE Q-TEMP-Y1  VARIABLE Q-TEMP-Z1
VARIABLE Q-TEMP-W2  VARIABLE Q-TEMP-X2  VARIABLE Q-TEMP-Y2  VARIABLE Q-TEMP-Z2

\ ------------------------------------------------------------------------------
\ Section 1: Memory Operations and Identity
\ ------------------------------------------------------------------------------

\ Q@ ( addr -- qw qx qy qz )
\ Load quaternion from memory onto stack.
: Q@ ( addr -- qw qx qy qz )
    DUP @
    OVER CELL+ @
    OVER 2 CELLS + @
    ROT 3 CELLS + @
;

\ Q! ( qw qx qy qz addr -- )
\ Store quaternion from stack into memory.
: Q! ( qw qx qy qz addr -- )
    >R
    R@ 3 CELLS + !
    R@ 2 CELLS + !
    R@ CELL+ !
    R> !
;

\ Q-IDENTITY ( addr -- )
\ Initialize memory buffer to identity quaternion (1, 0, 0, 0).
: Q-IDENTITY ( addr -- )
    Q32-ONE OVER !
    0 OVER CELL+ !
    0 OVER 2 CELLS + !
    0 SWAP 3 CELLS + !
;

\ Q-COPY ( src dst -- )
: Q-COPY ( src dst -- )
    QUAT-BYTES MOVE
;

\ ------------------------------------------------------------------------------
\ Section 2: Conjugation, Norms, and Normalization
\ ------------------------------------------------------------------------------

\ Q-CONJ ( qw qx qy qz -- qw -qx -qy -qz )
\ Compute quaternion conjugate q* = [qw, -qx, -qy, -qz].
: Q-CONJ ( qw qx qy qz -- qw -qx -qy -qz )
    NEGATE >R
    NEGATE >R
    NEGATE >R
    R> R> R>
;

\ Q-NORM-SQ ( qw qx qy qz -- norm_sq_q32 )
\ Compute squared magnitude ||q||^2 = qw^2 + qx^2 + qy^2 + qz^2.
: Q-NORM-SQ ( qw qx qy qz -- norm_sq )
    DUP Q32* >R
    DUP Q32* >R
    DUP Q32* >R
    DUP Q32*
    R> + R> + R> +
;

\ Q-NORM ( qw qx qy qz -- norm_q32 )
\ Compute quaternion magnitude ||q||.
: Q-NORM ( qw qx qy qz -- norm )
    Q-NORM-SQ CORDIC-SQRT
;

\ Q-NORMALIZE ( qw qx qy qz -- qw' qx' qy' qz' )
\ Normalize quaternion to unit length.
: Q-NORMALIZE ( qw qx qy qz -- qw' qx' qy' qz' )
    4DUP Q-NORM
    DUP 0= IF DROP EXIT THEN
    >R
    R@ Q32/ >R
    R@ Q32/ >R
    R@ Q32/ >R
    R> Q32/
    R> R> R> R> DROP
;

\ ------------------------------------------------------------------------------
\ Section 3: Quaternion Multiplication (Hamilton Product)
\ ------------------------------------------------------------------------------

\ Q*Q ( qw1 qx1 qy1 qz1 qw2 qx2 qy2 qz2 -- qw qx qy qz )
\ Compute Hamilton product q = q1 * q2:
\   qw = w1*w2 - x1*x2 - y1*y2 - z1*z2
\   qx = w1*x2 + x1*w2 + y1*z2 - z1*y2
\   qy = w1*y2 - x1*z2 + y1*w2 + z1*x2
\   qz = w1*z2 + x1*y2 - y1*x2 + z1*w2
: Q*Q ( qw1 qx1 qy1 qz1 qw2 qx2 qy2 qz2 -- qw qx qy qz )
    Q-TEMP-Z2 ! Q-TEMP-Y2 ! Q-TEMP-X2 ! Q-TEMP-W2 !
    Q-TEMP-Z1 ! Q-TEMP-Y1 ! Q-TEMP-X1 ! Q-TEMP-W1 !

    \ qw
    Q-TEMP-W1 @ Q-TEMP-W2 @ Q32*
    Q-TEMP-X1 @ Q-TEMP-X2 @ Q32* -
    Q-TEMP-Y1 @ Q-TEMP-Y2 @ Q32* -
    Q-TEMP-Z1 @ Q-TEMP-Z2 @ Q32* -

    \ qx
    Q-TEMP-W1 @ Q-TEMP-X2 @ Q32*
    Q-TEMP-X1 @ Q-TEMP-W2 @ Q32* +
    Q-TEMP-Y1 @ Q-TEMP-Z2 @ Q32* +
    Q-TEMP-Z1 @ Q-TEMP-Y2 @ Q32* -

    \ qy
    Q-TEMP-W1 @ Q-TEMP-Y2 @ Q32*
    Q-TEMP-X1 @ Q-TEMP-Z2 @ Q32* -
    Q-TEMP-Y1 @ Q-TEMP-W2 @ Q32* +
    Q-TEMP-Z1 @ Q-TEMP-X2 @ Q32* +

    \ qz
    Q-TEMP-W1 @ Q-TEMP-Z2 @ Q32*
    Q-TEMP-X1 @ Q-TEMP-Y2 @ Q32* +
    Q-TEMP-Y1 @ Q-TEMP-X2 @ Q32* -
    Q-TEMP-Z1 @ Q-TEMP-W2 @ Q32* +
;

\ ------------------------------------------------------------------------------
\ Section 4: Vector Rotation via Quaternion
\ ------------------------------------------------------------------------------

VARIABLE Q-VEC-X  VARIABLE Q-VEC-Y  VARIABLE Q-VEC-Z
VARIABLE Q-CROSS1-X  VARIABLE Q-CROSS1-Y  VARIABLE Q-CROSS1-Z

\ Q-ROT-V3 ( qw qx qy qz vx vy vz -- rx ry rz )
\ Rotate 3D vector v by unit quaternion q using Rodrigues formula:
\   t = 2 * (q_v x v)
\   v' = v + q_w * t + (q_v x t)
: Q-ROT-V3 ( qw qx qy qz vx vy vz -- rx ry rz )
    Q-VEC-Z ! Q-VEC-Y ! Q-VEC-X !
    Q-TEMP-Z1 ! Q-TEMP-Y1 ! Q-TEMP-X1 ! Q-TEMP-W1 !

    \ t = 2 * (q_v x v)
    Q-TEMP-X1 @ Q-TEMP-Y1 @ Q-TEMP-Z1 @
    Q-VEC-X @ Q-VEC-Y @ Q-VEC-Z @
    V3-CROSS
    2 S>Q32 V3*SCALE
    Q-CROSS1-Z ! Q-CROSS1-Y ! Q-CROSS1-X !

    \ term1 = q_w * t
    Q-CROSS1-X @ Q-CROSS1-Y @ Q-CROSS1-Z @
    Q-TEMP-W1 @ V3*SCALE

    \ term2 = (q_v x t)
    Q-TEMP-X1 @ Q-TEMP-Y1 @ Q-TEMP-Z1 @
    Q-CROSS1-X @ Q-CROSS1-Y @ Q-CROSS1-Z @
    V3-CROSS

    \ v' = v + term1 + term2
    V3+
    Q-VEC-X @ Q-VEC-Y @ Q-VEC-Z @
    V3+
;

\ ------------------------------------------------------------------------------
\ Section 5: Axis-Angle and DCM Conversions
\ ------------------------------------------------------------------------------

\ Q-AXIS-ANGLE ( ux uy uz angle_rad -- qw qx qy qz )
\ Construct unit quaternion from normalized axis (ux, uy, uz) and rotation angle.
: Q-AXIS-ANGLE ( ux uy uz angle -- qw qx qy qz )
    2/ \ angle / 2
    CORDIC-SIN-COS \ Stack: ( ux uy uz sin(theta/2) cos(theta/2) )
    >R >R \ R: cos sin
    R@ V3*SCALE \ Scale (ux, uy, uz) by sin(theta/2)
    R> DROP \ Drop sin
    R> -ROT \ Bring cos(theta/2) to bottom of quaternion: ( qw qx qy qz )
    ROT SWAP
;

\ Q>DCM ( qw qx qy qz m_out -- )
\ Convert unit quaternion to 3x3 Direction Cosine Matrix:
\   m00 = 1 - 2*(y^2 + z^2),  m01 = 2*(x*y - w*z),      m02 = 2*(x*z + w*y)
\   m10 = 2*(x*y + w*z),      m11 = 1 - 2*(x^2 + z^2),  m12 = 2*(y*z - w*x)
\   m20 = 2*(x*z - w*y),      m21 = 2*(y*z + w*x),      m22 = 1 - 2*(x^2 + y^2)
: Q>DCM ( qw qx qy qz m_out -- )
    >R \ R: m_out
    Q-TEMP-Z1 ! Q-TEMP-Y1 ! Q-TEMP-X1 ! Q-TEMP-W1 !

    \ Row 0
    Q32-ONE
    Q-TEMP-Y1 @ DUP Q32* Q-TEMP-Z1 @ DUP Q32* + 1 LSHIFT -
    R@ 0 0 M3!

    Q-TEMP-X1 @ Q-TEMP-Y1 @ Q32* Q-TEMP-W1 @ Q-TEMP-Z1 @ Q32* - 1 LSHIFT
    R@ 0 1 M3!

    Q-TEMP-X1 @ Q-TEMP-Z1 @ Q32* Q-TEMP-W1 @ Q-TEMP-Y1 @ Q32* + 1 LSHIFT
    R@ 0 2 M3!

    \ Row 1
    Q-TEMP-X1 @ Q-TEMP-Y1 @ Q32* Q-TEMP-W1 @ Q-TEMP-Z1 @ Q32* + 1 LSHIFT
    R@ 1 0 M3!

    Q32-ONE
    Q-TEMP-X1 @ DUP Q32* Q-TEMP-Z1 @ DUP Q32* + 1 LSHIFT -
    R@ 1 1 M3!

    Q-TEMP-Y1 @ Q-TEMP-Z1 @ Q32* Q-TEMP-W1 @ Q-TEMP-X1 @ Q32* - 1 LSHIFT
    R@ 1 2 M3!

    \ Row 2
    Q-TEMP-X1 @ Q-TEMP-Z1 @ Q32* Q-TEMP-W1 @ Q-TEMP-Y1 @ Q32* - 1 LSHIFT
    R@ 2 0 M3!

    Q-TEMP-Y1 @ Q-TEMP-Z1 @ Q32* Q-TEMP-W1 @ Q-TEMP-X1 @ Q32* + 1 LSHIFT
    R@ 2 1 M3!

    Q32-ONE
    Q-TEMP-X1 @ DUP Q32* Q-TEMP-Y1 @ DUP Q32* + 1 LSHIFT -
    R@ 2 2 M3!

    R> DROP
;

\ ------------------------------------------------------------------------------
\ Section 6: Spherical Linear Interpolation (SLERP)
\ ------------------------------------------------------------------------------

VARIABLE Q-SLERP-DOT
VARIABLE Q-SLERP-T

\ Q-SLERP ( qw1 qx1 qy1 qz1 qw2 qx2 qy2 qz2 t_q32 -- qw qx qy qz )
\ Spherical Linear Interpolation between two unit quaternions with parameter t in [0, 1].
: Q-SLERP ( q1 q2 t -- q_interp )
    Q-SLERP-T !
    Q-TEMP-Z2 ! Q-TEMP-Y2 ! Q-TEMP-X2 ! Q-TEMP-W2 !
    Q-TEMP-Z1 ! Q-TEMP-Y1 ! Q-TEMP-X1 ! Q-TEMP-W1 !

    \ Compute cosine of angle between quaternions: cos_omega = q1 . q2
    Q-TEMP-W1 @ Q-TEMP-W2 @ Q32*
    Q-TEMP-X1 @ Q-TEMP-X2 @ Q32* +
    Q-TEMP-Y1 @ Q-TEMP-Y2 @ Q32* +
    Q-TEMP-Z1 @ Q-TEMP-Z2 @ Q32* +
    Q-SLERP-DOT !

    \ If negative dot product, invert q2 to take the shortest geodesic arc
    Q-SLERP-DOT @ 0< IF
        Q-SLERP-DOT @ NEGATE Q-SLERP-DOT !
        Q-TEMP-W2 @ NEGATE Q-TEMP-W2 !
        Q-TEMP-X2 @ NEGATE Q-TEMP-X2 !
        Q-TEMP-Y2 @ NEGATE Q-TEMP-Y2 !
        Q-TEMP-Z2 @ NEGATE Q-TEMP-Z2 !
    THEN

    \ If quaternions are very close, use fast linear interpolation (LERP)
    Q-SLERP-DOT @ 4080218931 > IF \ cos_omega > 0.95
        \ scale1 = 1 - t
        Q32-ONE Q-SLERP-T @ -
        \ scale2 = t
        Q-SLERP-T @
        >R >R
        Q-TEMP-W1 @ R@ Q32* Q-TEMP-W2 @ 2 PICK Q32* +
        Q-TEMP-X1 @ R@ Q32* Q-TEMP-X2 @ 2 PICK Q32* +
        Q-TEMP-Y1 @ R@ Q32* Q-TEMP-Y2 @ 2 PICK Q32* +
        Q-TEMP-Z1 @ R> Q32* Q-TEMP-Z2 @ R> Q32* +
        Q-NORMALIZE
        EXIT
    THEN

    \ Full spherical trigonometry
    \ omega = acos(cos_omega)
    Q32-ONE Q-SLERP-DOT @ DUP Q32* - CORDIC-SQRT
    Q-SLERP-DOT @ CORDIC-ATAN2 \ omega in radians
    DUP >R \ R: omega

    CORDIC-SIN \ sin(omega)
    DUP >R \ R: omega sin_omega

    \ scale1 = sin((1 - t) * omega) / sin(omega)
    Q32-ONE Q-SLERP-T @ - 2 PICK Q32* CORDIC-SIN
    R@ Q32/ >R \ R: omega sin_omega scale1

    \ scale2 = sin(t * omega) / sin(omega)
    Q-SLERP-T @ 3 PICK Q32* CORDIC-SIN
    2 PICK Q32/ \ Stack: scale2

    R> R> R> 2DROP DROP \ Stack: scale2 scale1
    SWAP >R >R \ R: scale2 scale1

    Q-TEMP-W1 @ R@ Q32* Q-TEMP-W2 @ 2 PICK Q32* +
    Q-TEMP-X1 @ R@ Q32* Q-TEMP-X2 @ 2 PICK Q32* +
    Q-TEMP-Y1 @ R@ Q32* Q-TEMP-Y2 @ 2 PICK Q32* +
    Q-TEMP-Z1 @ R> Q32* Q-TEMP-Z2 @ R> Q32* +
    R> DROP
;

\ ------------------------------------------------------------------------------
\ Section 7: Quaternion Display
\ ------------------------------------------------------------------------------

\ Q. ( qw qx qy qz -- )
\ Formatted display: (qw + qx*i + qy*j + qz*k)
: Q. ( qw qx qy qz -- )
    [CHAR] ( EMIT SPACE
    ROT ROT SWAP
    Q32. ." + "
    Q32. ." i + "
    Q32. ." j + "
    Q32. ." k" SPACE [CHAR] ) EMIT
;
