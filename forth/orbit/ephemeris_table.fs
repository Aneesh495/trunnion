\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/orbit/ephemeris_table.fs
\ Description: Ephemeris waypoint buffer and cubic Hermite spline interpolation
\              engine for continuous position and velocity evaluation.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Waypoint Structure and Table Storage
\ ------------------------------------------------------------------------------

\ Waypoint record layout (7 cells = 56 bytes):
\ Offset 0: timestamp_sec (Q32.32)
\ Offset 1..3: pos_x, pos_y, pos_z (km Q32.32)
\ Offset 4..6: vel_x, vel_y, vel_z (km/s Q32.32)
7 CONSTANT EPHEM-ENTRY-CELLS
EPHEM-ENTRY-CELLS CELLS CONSTANT EPHEM-ENTRY-BYTES

64 CONSTANT EPHEM-MAX-ENTRIES
CREATE EPHEM-TABLE EPHEM-MAX-ENTRIES EPHEM-ENTRY-BYTES * ALLOT

VARIABLE EPHEM-NUM-ENTRIES
VARIABLE EPHEM-HEAD-IDX

\ Interpolated Output States
VARIABLE EPHEM-OUT-POS-X  VARIABLE EPHEM-OUT-POS-Y  VARIABLE EPHEM-OUT-POS-Z
VARIABLE EPHEM-OUT-VEL-X  VARIABLE EPHEM-OUT-VEL-Y  VARIABLE EPHEM-OUT-VEL-Z

\ ------------------------------------------------------------------------------
\ Section 2: Table Maintenance API
\ ------------------------------------------------------------------------------

\ EPHEM-CLEAR ( -- )
: EPHEM-CLEAR ( -- )
    0 EPHEM-NUM-ENTRIES !
    0 EPHEM-HEAD-IDX !
    EPHEM-TABLE EPHEM-MAX-ENTRIES EPHEM-ENTRY-BYTES * ERASE
;

\ EPHEM-ADDR ( idx -- addr )
: EPHEM-ADDR ( idx -- addr )
    EPHEM-ENTRY-BYTES * EPHEM-TABLE +
;

\ EPHEM-ADD-WAYPOINT ( t_sec x y z vx vy vz -- )
\ Append a new position/velocity ephemeris state waypoint.
: EPHEM-ADD-WAYPOINT ( t x y z vx vy vz -- )
    EPHEM-NUM-ENTRIES @ EPHEM-MAX-ENTRIES >= IF
        7DROP EXIT
    THEN

    EPHEM-NUM-ENTRIES @ EPHEM-ADDR >R
    R@ 6 CELLS + !
    R@ 5 CELLS + !
    R@ 4 CELLS + !
    R@ 3 CELLS + !
    R@ 2 CELLS + !
    R@ 1 CELLS + !
    R> !
    1 EPHEM-NUM-ENTRIES +!
;

\ ------------------------------------------------------------------------------
\ Section 3: Cubic Hermite Spline Evaluation
\ ------------------------------------------------------------------------------

VARIABLE HERM-T
VARIABLE HERM-DT
VARIABLE HERM-TAU
VARIABLE HERM-TAU2
VARIABLE HERM-TAU3

VARIABLE HERM-H00  VARIABLE HERM-H10
VARIABLE HERM-H01  VARIABLE HERM-H11

VARIABLE HERM-DH00 VARIABLE HERM-DH10
VARIABLE HERM-DH01 VARIABLE HERM-DH11

VARIABLE EPHEM-BRACKET-K
VARIABLE EPHEM-ADDR-K
VARIABLE EPHEM-ADDR-K1
VARIABLE HERM-P0
VARIABLE HERM-V0
VARIABLE HERM-P1
VARIABLE HERM-V1

\ HERM-COMPUTE-BASIS ( tau_q32 dt_q32 -- )
: HERM-COMPUTE-BASIS ( tau dt -- )
    HERM-DT !
    DUP HERM-TAU !
    DUP Q32* DUP HERM-TAU2 !
    HERM-TAU @ Q32* HERM-TAU3 !

    \ h00 = 2*tau^3 - 3*tau^2 + 1
    2 S>Q32 HERM-TAU3 @ Q32*
    3 S>Q32 HERM-TAU2 @ Q32* -
    Q32-ONE + HERM-H00 !

    \ h10 = (tau^3 - 2*tau^2 + tau) * dt
    HERM-TAU3 @
    2 S>Q32 HERM-TAU2 @ Q32* -
    HERM-TAU @ +
    HERM-DT @ Q32* HERM-H10 !

    \ h01 = -2*tau^3 + 3*tau^2
    -2 S>Q32 HERM-TAU3 @ Q32*
    3 S>Q32 HERM-TAU2 @ Q32* + HERM-H01 !

    \ h11 = (tau^3 - tau^2) * dt
    HERM-TAU3 @ HERM-TAU2 @ -
    HERM-DT @ Q32* HERM-H11 !
;

\ EPHEM-EVALUATE ( t_sec_q32 -- valid_flag )
\ Evaluate spacecraft position and velocity at continuous time t via Hermite spline.
: EPHEM-EVALUATE ( t -- flag )
    DUP HERM-T !
    EPHEM-NUM-ENTRIES @ 2 < IF
        DROP 0 EXIT
    THEN

    \ Scan for bracket interval [k, k+1]
    0 EPHEM-BRACKET-K !
    0 BEGIN
        DUP EPHEM-NUM-ENTRIES @ 1- <
    WHILE
        DUP EPHEM-ADDR @ HERM-T @ <=
        OVER 1+ EPHEM-ADDR @ HERM-T @ >= AND IF
            DUP EPHEM-BRACKET-K !
            1000
        ELSE
            1+
        THEN
    REPEAT
    DROP

    EPHEM-BRACKET-K @ EPHEM-ADDR EPHEM-ADDR-K !
    EPHEM-BRACKET-K @ 1+ EPHEM-ADDR EPHEM-ADDR-K1 !

    \ Interval duration: dt = t_{k+1} - t_k
    EPHEM-ADDR-K1 @ 0 CELLS + @
    EPHEM-ADDR-K @ 0 CELLS + @ -
    DUP 0<= IF
        DROP 0 EXIT
    THEN
    DUP HERM-DT !

    \ Normalized tau = (t - t_k) / dt
    HERM-T @ EPHEM-ADDR-K @ 0 CELLS + @ -
    HERM-DT @ Q32/
    0 S>Q32 Q32-ONE CLAMP

    \ Compute basis functions
    HERM-DT @ HERM-COMPUTE-BASIS

    \ Hermite interpolation for Position (X, Y, Z):
    \ p(tau) = h00*p0 + h10*v0 + h01*p1 + h11*v1
    3 0 DO
        I 1+ CELLS
        EPHEM-ADDR-K @ OVER + @ HERM-P0 !
        EPHEM-ADDR-K @ OVER + 3 CELLS + @ HERM-V0 !
        EPHEM-ADDR-K1 @ OVER + @ HERM-P1 !
        EPHEM-ADDR-K1 @ SWAP + 3 CELLS + @ HERM-V1 !

        HERM-P0 @ HERM-H00 @ Q32*
        HERM-V0 @ HERM-H10 @ Q32* +
        HERM-P1 @ HERM-H01 @ Q32* +
        HERM-V1 @ HERM-H11 @ Q32* +

        I CASE
            0 OF EPHEM-OUT-POS-X ! ENDOF
            1 OF EPHEM-OUT-POS-Y ! ENDOF
            2 OF EPHEM-OUT-POS-Z ! ENDOF
        ENDCASE
    LOOP

    1
;

\ EPHEM-GET-POS ( -- x y z )
: EPHEM-GET-POS ( -- x y z )
    EPHEM-OUT-POS-X @ EPHEM-OUT-POS-Y @ EPHEM-OUT-POS-Z @
;

\ EPHEM-GET-VEL ( -- vx vy vz )
: EPHEM-GET-VEL ( -- vx vy vz )
    EPHEM-OUT-VEL-X @ EPHEM-OUT-VEL-Y @ EPHEM-OUT-VEL-Z @
;
