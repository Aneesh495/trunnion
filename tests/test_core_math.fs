\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_core_math.fs
\ Description: Comprehensive Test Suite for Fixed-Point, CORDIC & Linear Algebra
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Core Math, Fixed-Point, CORDIC, Vector3, Matrix3, Quaternion" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Q32.32 Fixed-Point Arithmetic
\ ------------------------------------------------------------------------------

S" Q32.32 integer conversion round-trip" TEST-CASE-START
42 S>Q32 Q32>S 42 ASSERT-EQ

S" Q32.32 addition: 1.5 + 2.5 = 4.0" TEST-CASE-START
Q32-ONE Q32-ONE 2/ +
Q32-ONE 2* Q32-ONE 2/ +
Q32+
Q32-ONE 4 * 100 ASSERT-Q32-CLOSE

S" Q32.32 subtraction: 10.0 - 3.0 = 7.0" TEST-CASE-START
Q32-ONE 10 * Q32-ONE 3 * Q32- Q32-ONE 7 * 100 ASSERT-Q32-CLOSE

S" Q32.32 multiplication: 2.5 * 4.0 = 10.0" TEST-CASE-START
Q32-ONE 2* Q32-ONE 2/ +
Q32-ONE 4 *
Q32*
Q32-ONE 10 * 200 ASSERT-Q32-CLOSE

S" Q32.32 division: 15.0 / 3.0 = 5.0" TEST-CASE-START
Q32-ONE 15 * Q32-ONE 3 * Q32/ Q32-ONE 5 * 200 ASSERT-Q32-CLOSE

S" Q32.32 negative multiplication: -2.0 * 3.0 = -6.0" TEST-CASE-START
Q32-ONE -2 * Q32-ONE 3 * Q32* Q32-ONE -6 * 200 ASSERT-Q32-CLOSE

S" CORDIC square root: sqrt(16.0) = 4.0" TEST-CASE-START
Q32-ONE 16 * CORDIC-SQRT Q32-ONE 4 * 200 ASSERT-Q32-CLOSE

S" CORDIC square root: sqrt(2.0) approx 1.41421" TEST-CASE-START
Q32-ONE 2* CORDIC-SQRT 6074001000 50000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 2: CORDIC Trigonometry and Vectoring
\ ------------------------------------------------------------------------------

S" CORDIC: sin(0) = 0, cos(0) = 1" TEST-CASE-START
0 CORDIC-SIN-COS
\ Stack: ( sin cos ) with cos at TOS
Q32-ONE 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE

S" CORDIC: sin(pi/2) = 1, cos(pi/2) = 0" TEST-CASE-START
Q32-HALF-PI CORDIC-SIN-COS
0 1000000 ASSERT-Q32-CLOSE
Q32-ONE 1000000 ASSERT-Q32-CLOSE

S" CORDIC: sin(pi) approx 0, cos(pi) approx -1" TEST-CASE-START
Q32-PI CORDIC-SIN-COS
Q32-ONE NEGATE 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE

S" CORDIC: sin(pi/4) approx 0.707106, cos(pi/4) approx 0.707106" TEST-CASE-START
Q32-QUARTER-PI CORDIC-SIN-COS
3037000499 1000000 ASSERT-Q32-CLOSE
3037000499 1000000 ASSERT-Q32-CLOSE

S" CORDIC: atan2(0, 1) = 0" TEST-CASE-START
0 Q32-ONE CORDIC-ATAN2
0 1000000 ASSERT-Q32-CLOSE

S" CORDIC: atan2(1, 0) = pi/2" TEST-CASE-START
Q32-ONE 0 CORDIC-ATAN2
Q32-HALF-PI 1000000 ASSERT-Q32-CLOSE

S" CORDIC: atan2(1, 1) = pi/4" TEST-CASE-START
Q32-ONE Q32-ONE CORDIC-ATAN2
Q32-QUARTER-PI 1000000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 3: 3D Vector Linear Algebra
\ ------------------------------------------------------------------------------

S" Vector3 addition: [1, 2, 3] + [4, 5, 6] = [5, 7, 9]" TEST-CASE-START
Q32-ONE 1 * Q32-ONE 2 * Q32-ONE 3 *
Q32-ONE 4 * Q32-ONE 5 * Q32-ONE 6 *
V3+
Q32-ONE 9 * 100 ASSERT-Q32-CLOSE
Q32-ONE 7 * 100 ASSERT-Q32-CLOSE
Q32-ONE 5 * 100 ASSERT-Q32-CLOSE

S" Vector3 dot product: [1, 2, 3] . [4, -5, 6] = 4 - 10 + 18 = 12" TEST-CASE-START
Q32-ONE 1 * Q32-ONE 2 * Q32-ONE 3 *
Q32-ONE 4 * Q32-ONE -5 * Q32-ONE 6 *
V3-DOT
Q32-ONE 12 * 200 ASSERT-Q32-CLOSE

S" Vector3 cross product: [1, 0, 0] x [0, 1, 0] = [0, 0, 1]" TEST-CASE-START
Q32-ONE 0 0
0 Q32-ONE 0
V3-CROSS
Q32-ONE 100 ASSERT-Q32-CLOSE
0 100 ASSERT-Q32-CLOSE
0 100 ASSERT-Q32-CLOSE

S" Vector3 norm: norm([3, 4, 0]) = 5" TEST-CASE-START
Q32-ONE 3 * Q32-ONE 4 * 0 V3-NORM
Q32-ONE 5 * 200 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 4: 3x3 Matrix Transforms
\ ------------------------------------------------------------------------------

CREATE TEST-MA 9 CELLS ALLOT

S" Matrix3 identity times vector [7, 8, 9] = [7, 8, 9]" TEST-CASE-START
TEST-MA M3-IDENTITY
TEST-MA Q32-ONE 7 * Q32-ONE 8 * Q32-ONE 9 * M3*V3
Q32-ONE 9 * 100 ASSERT-Q32-CLOSE
Q32-ONE 8 * 100 ASSERT-Q32-CLOSE
Q32-ONE 7 * 100 ASSERT-Q32-CLOSE

S" Matrix3 90-degree Z rotation transforms [1, 0, 0] to [0, 1, 0]" TEST-CASE-START
Q32-HALF-PI TEST-MA M3-ROT-Z
TEST-MA Q32-ONE 0 0 M3*V3
0 1000000 ASSERT-Q32-CLOSE
Q32-ONE 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 5: Hamilton Quaternion Algebra
\ ------------------------------------------------------------------------------

S" Quaternion identity: norm([1, 0, 0, 0]) = 1.0" TEST-CASE-START
Q32-ONE 0 0 0 Q-NORM
Q32-ONE 200 ASSERT-Q32-CLOSE

S" Quaternion 90-deg Z rotation of [1, 0, 0] yields [0, 1, 0]" TEST-CASE-START
0 0 Q32-ONE Q32-HALF-PI Q-AXIS-ANGLE
Q32-ONE 0 0 Q-ROT-V3
0 1000000 ASSERT-Q32-CLOSE
Q32-ONE 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE

S" Quaternion SLERP at t=0.0 returns q1" TEST-CASE-START
Q32-ONE 0 0 0
0 0 Q32-ONE Q32-HALF-PI Q-AXIS-ANGLE
0 Q-SLERP
0 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE
Q32-ONE 1000000 ASSERT-Q32-CLOSE
