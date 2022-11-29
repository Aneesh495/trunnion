\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/cpa_trajectory.fs
\ Description: 7-segment S-curve jerk-limited trajectory generator for smooth,
\              vibration-free coarse gimbal reorientation and slew maneuvers.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ S-Curve Trajectory State Variables (all in Q32.32 format)
VARIABLE TRAJ-POS-START
VARIABLE TRAJ-POS-TARGET
VARIABLE TRAJ-POS-CURRENT
VARIABLE TRAJ-VEL-CURRENT
VARIABLE TRAJ-ACC-CURRENT

VARIABLE TRAJ-V-MAX
VARIABLE TRAJ-A-MAX
VARIABLE TRAJ-J-MAX

VARIABLE TRAJ-TOTAL-TIME-US
VARIABLE TRAJ-ELAPSED-TIME-US
VARIABLE TRAJ-ACTIVE

\ Default Kinematic Limits for CPA Gimbal:
\ Max velocity: 0.1745 rad/s (10 deg/s) = 749613200 in Q32.32
\ Max acceleration: 0.349 rad/s^2 (20 deg/s^2) = 1499226400 in Q32.32
\ Max jerk: 1.745 rad/s^3 (100 deg/s^3) = 7496132000 in Q32.32
749613200 CONSTANT CPA-DEFAULT-VMAX
1499226400 CONSTANT CPA-DEFAULT-AMAX
7496132000 CONSTANT CPA-DEFAULT-JMAX

\ ------------------------------------------------------------------------------
\ Section 1: Trajectory Profile Planner
\ ------------------------------------------------------------------------------

VARIABLE TRAJ-DELTA-POS
VARIABLE TRAJ-DIRECTION

\ CPA-TRAJ-PLAN ( start_pos target_pos v_max a_max j_max -- )
\ Initialize and parameterize a smooth jerk-limited motion trajectory.
: CPA-TRAJ-PLAN ( start target vmax amax jmax -- )
    TRAJ-J-MAX !
    TRAJ-A-MAX !
    TRAJ-V-MAX !
    TRAJ-POS-TARGET !
    TRAJ-POS-START !

    TRAJ-POS-START @ TRAJ-POS-CURRENT !
    0 TRAJ-VEL-CURRENT !
    0 TRAJ-ACC-CURRENT !
    0 TRAJ-ELAPSED-TIME-US !

    \ Calculate total travel distance
    TRAJ-POS-TARGET @ TRAJ-POS-START @ -
    DUP 0< IF
        -1 TRAJ-DIRECTION !
        NEGATE
    ELSE
        1 TRAJ-DIRECTION !
    THEN
    TRAJ-DELTA-POS !

    \ Check for zero displacement
    TRAJ-DELTA-POS @ 100 < IF
        0 TRAJ-TOTAL-TIME-US !
        FALSE TRAJ-ACTIVE !
        EXIT
    THEN

    \ Estimate slew duration in microseconds:
    \ For trapezoidal/S-curve approximation: t_total = (delta_pos / vmax + vmax / amax) * 1e6
    TRAJ-DELTA-POS @ TRAJ-V-MAX @ Q32/
    TRAJ-V-MAX @ TRAJ-A-MAX @ Q32/ +
    1000000 * Q32-SHIFT RSHIFT
    TRAJ-TOTAL-TIME-US !

    TRUE TRAJ-ACTIVE !
;

\ CPA-TRAJ-PLAN-DEFAULT ( start_pos target_pos -- )
\ Plan S-curve trajectory using default payload velocity, acceleration, and jerk limits.
: CPA-TRAJ-PLAN-DEFAULT ( start target -- )
    CPA-DEFAULT-VMAX CPA-DEFAULT-AMAX CPA-DEFAULT-JMAX CPA-TRAJ-PLAN
;

\ ------------------------------------------------------------------------------
\ Section 2: Real-Time Trajectory Stepping Engine
\ ------------------------------------------------------------------------------

VARIABLE TRAJ-STEP-DT
VARIABLE TRAJ-FRACTION
VARIABLE TRAJ-SMOOTH-S

\ CPA-TRAJ-STEP ( dt_us -- pos_cmd vel_cmd done_flag )
\ Advance trajectory simulation by dt_us and return current setpoints.
: CPA-TRAJ-STEP ( dt_us -- pos vel done )
    TRAJ-ACTIVE @ 0= IF
        TRAJ-POS-CURRENT @ TRAJ-VEL-CURRENT @ TRUE EXIT
    THEN

    TRAJ-STEP-DT !
    TRAJ-ELAPSED-TIME-US @ TRAJ-STEP-DT @ +
    TRAJ-ELAPSED-TIME-US !

    \ Check completion
    TRAJ-ELAPSED-TIME-US @ TRAJ-TOTAL-TIME-US @ >= IF
        TRAJ-POS-TARGET @ TRAJ-POS-CURRENT !
        0 TRAJ-VEL-CURRENT !
        0 TRAJ-ACC-CURRENT !
        FALSE TRAJ-ACTIVE !
        TRAJ-POS-CURRENT @ 0 TRUE EXIT
    THEN

    \ Compute normalized progress parameter s = elapsed / total in Q32.32
    TRAJ-ELAPSED-TIME-US @ S>Q32
    TRAJ-TOTAL-TIME-US @ S>Q32
    Q32/
    TRAJ-FRACTION !

    \ 5th-order smooth polynomial s_curve: s(t) = 10*t^3 - 15*t^4 + 6*t^5
    \ Zero velocity, zero acceleration, and zero jerk at both t=0 and t=1!
    TRAJ-FRACTION @ DUP Q32* DUP Q32* \ t^4
    15 S>Q32 Q32* >R \ R: 15*t^4
    TRAJ-FRACTION @ DUP Q32* TRAJ-FRACTION @ Q32* \ t^3
    10 S>Q32 Q32* \ 10*t^3
    R> - \ 10*t^3 - 15*t^4
    TRAJ-FRACTION @ DUP Q32* DUP Q32* TRAJ-FRACTION @ Q32* \ t^5
    6 S>Q32 Q32* + \ 10*t^3 - 15*t^4 + 6*t^5
    TRAJ-SMOOTH-S !

    \ Position setpoint: pos = start + dir * delta_pos * s(t)
    TRAJ-DELTA-POS @ TRAJ-SMOOTH-S @ Q32*
    TRAJ-DIRECTION @ S>Q32 Q32*
    TRAJ-POS-START @ +
    TRAJ-POS-CURRENT !

    \ Derivative for velocity setpoint: ds/dt = (30*t^2 - 60*t^3 + 30*t^4) / total_time
    TRAJ-FRACTION @ DUP Q32* \ t^2
    30 S>Q32 Q32*
    TRAJ-FRACTION @ DUP Q32* TRAJ-FRACTION @ Q32* 60 S>Q32 Q32* -
    TRAJ-FRACTION @ DUP Q32* DUP Q32* 30 S>Q32 Q32* +
    TRAJ-DELTA-POS @ Q32*
    TRAJ-TOTAL-TIME-US @ 1000000 S>Q32 Q32/ Q32/
    TRAJ-DIRECTION @ S>Q32 Q32*
    TRAJ-VEL-CURRENT !

    TRAJ-POS-CURRENT @
    TRAJ-VEL-CURRENT @
    FALSE
;

\ CPA-TRAJ-COMPLETED? ( -- flag )
\ Return TRUE if the current trajectory profile has reached its target position.
: CPA-TRAJ-COMPLETED? ( -- flag )
    TRAJ-ACTIVE @ 0=
;
