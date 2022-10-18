\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/matrix3.fs
\ Description: 3x3 Direction Cosine Matrix (DCM) algebra, determinant,
\              inversion, and Euler rotation operators in signed Q32.32 fixed-point.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Matrix memory format: 9 contiguous 64-bit cells (72 bytes), row-major:
\   m[0,0] (offset 0)   m[0,1] (offset 8)   m[0,2] (offset 16)
\   m[1,0] (offset 24)  m[1,1] (offset 32)  m[1,2] (offset 40)
\   m[2,0] (offset 48)  m[2,1] (offset 56)  m[2,2] (offset 64)

72 CONSTANT MAT3-BYTES

\ ------------------------------------------------------------------------------
\ Section 1: Matrix Element Indexing and Initialization
\ ------------------------------------------------------------------------------

\ M3-ADDR ( addr r c -- elem_addr )
\ Compute address of element at row r and column c (0 <= r, c < 3).
: M3-ADDR ( addr r c -- elem_addr )
    SWAP 3 * + CELLS +
;

\ M3@ ( addr r c -- val_q32 )
\ Retrieve element at row r, column c.
: M3@ ( addr r c -- val )
    M3-ADDR @
;

\ M3! ( val_q32 addr r c -- )
\ Store element at row r, column c.
: M3! ( val addr r c -- )
    M3-ADDR !
;

\ M3-ZERO ( addr -- )
\ Clear 3x3 matrix to all zeros.
: M3-ZERO ( addr -- )
    MAT3-BYTES ERASE
;

\ M3-IDENTITY ( addr -- )
\ Initialize 3x3 matrix to identity matrix I.
: M3-IDENTITY ( addr -- )
    DUP M3-ZERO
    DUP Q32-ONE SWAP 0 0 M3!
    DUP Q32-ONE SWAP 1 1 M3!
    Q32-ONE SWAP 2 2 M3!
;

\ M3-COPY ( src dst -- )
\ Copy 72-byte 3x3 matrix from src to dst.
: M3-COPY ( src dst -- )
    MAT3-BYTES MOVE
;

\ ------------------------------------------------------------------------------
\ Section 2: Matrix Arithmetic & Transpose
\ ------------------------------------------------------------------------------

\ M3+ ( m1 m2 m_out -- )
\ Element-wise addition of two 3x3 matrices.
: M3+ ( m1 m2 m_out -- )
    >R
    9 0 DO
        OVER I CELLS + @
        OVER I CELLS + @ +
        R@ I CELLS + !
    LOOP
    2DROP R> DROP
;

\ M3- ( m1 m2 m_out -- )
\ Element-wise subtraction of two 3x3 matrices.
: M3- ( m1 m2 m_out -- )
    >R
    9 0 DO
        OVER I CELLS + @
        OVER I CELLS + @ -
        R@ I CELLS + !
    LOOP
    2DROP R> DROP
;

\ M3*SCALE ( m scale_q32 m_out -- )
\ Scale 3x3 matrix by scalar in Q32.32 format.
: M3*SCALE ( m s m_out -- )
    >R >R
    9 0 DO
        DUP I CELLS + @ R@ Q32*
        R@ I CELLS + !
    LOOP
    DROP R> R> 2DROP
;

\ M3-TRANSPOSE ( m m_out -- )
\ Transpose 3x3 matrix: m_out[c, r] = m[r, c].
: M3-TRANSPOSE ( m m_out -- )
    3 0 DO
        3 0 DO
            OVER J I M3@
            OVER I J M3!
        LOOP
    LOOP
    2DROP
;

\ ------------------------------------------------------------------------------
\ Section 3: Matrix Multiplication and Vector Transform
\ ------------------------------------------------------------------------------

VARIABLE M3-PROD-ACC

\ M3*M3 ( m1 m2 m_out -- )
\ Multiply two 3x3 matrices: m_out = m1 * m2.
: M3*M3 ( m1 m2 m_out -- )
    >R
    3 0 DO
        3 0 DO
            0 M3-PROD-ACC !
            3 0 DO
                3 PICK K I M3@
                3 PICK I J M3@ Q32*
                M3-PROD-ACC @ + M3-PROD-ACC !
            LOOP
            M3-PROD-ACC @ R@ K J M3!
        LOOP
    LOOP
    2DROP R> DROP
;

VARIABLE M3-RX
VARIABLE M3-RY
VARIABLE M3-RZ

\ M3*V3 ( m vx vy vz -- rx ry rz )
\ Multiply 3x3 matrix by 3D vector: r = M * v.
: M3*V3 ( m vx vy vz -- rx ry rz )
    V3-B-Z !
    V3-B-Y !
    V3-B-X !
    DUP 0 0 M3@ V3-B-X @ Q32*
    OVER 0 1 M3@ V3-B-Y @ Q32* +
    OVER 0 2 M3@ V3-B-Z @ Q32* + M3-RX !
    DUP 1 0 M3@ V3-B-X @ Q32*
    OVER 1 1 M3@ V3-B-Y @ Q32* +
    OVER 1 2 M3@ V3-B-Z @ Q32* + M3-RY !
    DUP 2 0 M3@ V3-B-X @ Q32*
    OVER 2 1 M3@ V3-B-Y @ Q32* +
    SWAP 2 2 M3@ V3-B-Z @ Q32* + M3-RZ !
    M3-RX @ M3-RY @ M3-RZ @
;

\ ------------------------------------------------------------------------------
\ Section 4: Determinant and Inverse
\ ------------------------------------------------------------------------------

\ M3-DETERMINANT ( m -- det_q32 )
\ Compute determinant of 3x3 matrix via Laplace expansion along row 0:
\   det = m00*(m11*m22 - m12*m21) - m01*(m10*m22 - m12*m20) + m02*(m10*m21 - m11*m20)
: M3-DETERMINANT ( m -- det )
    >R
    \ Term 0: m00 * (m11*m22 - m12*m21)
    R@ 1 1 M3@ R@ 2 2 M3@ Q32*
    R@ 1 2 M3@ R@ 2 1 M3@ Q32* -
    R@ 0 0 M3@ Q32*

    \ Term 1: m01 * (m10*m22 - m12*m20)
    R@ 1 0 M3@ R@ 2 2 M3@ Q32*
    R@ 1 2 M3@ R@ 2 0 M3@ Q32* -
    R@ 0 1 M3@ Q32* -

    \ Term 2: m02 * (m10*m21 - m11*m20)
    R@ 1 0 M3@ R@ 2 1 M3@ Q32*
    R@ 1 1 M3@ R@ 2 0 M3@ Q32* -
    R@ 0 2 M3@ Q32* +
    R> DROP
;

\ M3-INVERSE ( m m_out -- success_flag )
\ Invert 3x3 matrix using adjugate / determinant. Returns TRUE if invertible.
: M3-INVERSE ( m m_out -- flag )
    OVER M3-DETERMINANT
    DUP ABS 100 < IF \ Singularity check
        2DROP DROP FALSE EXIT
    THEN
    >R \ R: det
    >R >R \ R: det m_out m
    \ Compute classical adjugate transpose:
    \ row 0 of adj (col 0 of cofactors)
    R@ 1 1 M3@ R@ 2 2 M3@ Q32* R@ 1 2 M3@ R@ 2 1 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 0 0 M3! >R >R
    R@ 1 2 M3@ R@ 2 0 M3@ Q32* R@ 1 0 M3@ R@ 2 2 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 0 1 M3! >R >R
    R@ 1 0 M3@ R@ 2 1 M3@ Q32* R@ 1 1 M3@ R@ 2 0 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 0 2 M3! >R >R

    \ row 1 of adj
    R@ 0 2 M3@ R@ 2 1 M3@ Q32* R@ 0 1 M3@ R@ 2 2 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 1 0 M3! >R >R
    R@ 0 0 M3@ R@ 2 2 M3@ Q32* R@ 0 2 M3@ R@ 2 0 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 1 1 M3! >R >R
    R@ 0 1 M3@ R@ 2 0 M3@ Q32* R@ 0 0 M3@ R@ 2 1 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 1 2 M3! >R >R

    \ row 2 of adj
    R@ 0 1 M3@ R@ 1 2 M3@ Q32* R@ 0 2 M3@ R@ 1 1 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 2 0 M3! >R >R
    R@ 0 2 M3@ R@ 1 0 M3@ Q32* R@ 0 0 M3@ R@ 1 2 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 2 1 M3! >R >R
    R@ 0 0 M3@ R@ 1 1 M3@ Q32* R@ 0 1 M3@ R@ 1 0 M3@ Q32* - 2 PICK Q32/ R> R> 2DUP 2 2 M3! >R >R

    R> R> 2DROP R> DROP
    TRUE
;

\ ------------------------------------------------------------------------------
\ Section 5: Elementary Rotation Matrices & Euler Angles
\ ------------------------------------------------------------------------------

\ M3-ROT-Z ( angle_q32 m_out -- )
\ Generate rotation matrix around Z-axis (Yaw) by angle:
\   [  cos  -sin    0 ]
\   [  sin   cos    0 ]
\   [    0     0    1 ]
: M3-ROT-Z ( angle m_out -- )
    DUP M3-ZERO
    SWAP CORDIC-SIN-COS \ Stack: ( m_out sin cos )
    ROT >R \ R: m_out
    2DUP R@ 0 0 M3!
    R@ 1 1 M3!
    DUP NEGATE R@ 0 1 M3!
    R@ 1 0 M3!
    Q32-ONE R> 2 2 M3!
;

\ M3-ROT-Y ( angle_q32 m_out -- )
\ Generate rotation matrix around Y-axis (Pitch) by angle:
\   [  cos    0   sin ]
\   [    0    1     0 ]
\   [ -sin    0   cos ]
: M3-ROT-Y ( angle m_out -- )
    DUP M3-ZERO
    SWAP CORDIC-SIN-COS \ Stack: ( m_out sin cos )
    ROT >R
    2DUP R@ 0 0 M3!
    R@ 2 2 M3!
    DUP R@ 0 2 M3!
    NEGATE R@ 2 0 M3!
    Q32-ONE R> 1 1 M3!
;

\ M3-ROT-X ( angle_q32 m_out -- )
\ Generate rotation matrix around X-axis (Roll) by angle:
\   [    1    0     0 ]
\   [    0   cos -sin ]
\   [    0   sin  cos ]
: M3-ROT-X ( angle m_out -- )
    DUP M3-ZERO
    SWAP CORDIC-SIN-COS \ Stack: ( m_out sin cos )
    ROT >R
    2DUP R@ 1 1 M3!
    R@ 2 2 M3!
    DUP NEGATE R@ 1 2 M3!
    R@ 2 1 M3!
    Q32-ONE R> 0 0 M3!
;

\ ------------------------------------------------------------------------------
\ Section 6: Matrix Diagnostic Display
\ ------------------------------------------------------------------------------

\ M3. ( m -- )
\ Print 3x3 matrix in formatted 3-row layout.
: M3. ( m -- )
    CR
    3 0 DO
        ." | "
        3 0 DO
            DUP J I M3@ Q32. SPACE
        LOOP
        ." |" CR
    LOOP
    DROP
;
