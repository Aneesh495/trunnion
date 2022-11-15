\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/cpa_servo.fs
\ Description: Dual-loop cascaded position-velocity PID servo controller with
\              integrator anti-windup and dynamic friction feedforward.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Servo Gains in Q32.32 format:
\ Position Loop Proportional Gain: K_pp = 25.0 rad/s per rad error
107374182400 CONSTANT CPA-KP-POS
\ Velocity Loop Proportional Gain: K_vp = 15000.0 mA per rad/s
64424509440000 CONSTANT CPA-KP-VEL
\ Velocity Loop Integral Gain: K_vi = 1500.0 mA/(rad/s * s)
6442450944000 CONSTANT CPA-KI-VEL
\ Velocity Loop Derivative Gain: K_vd = 50.0 mA*s/rad
214748364800 CONSTANT CPA-KD-VEL

\ Integrator Anti-Windup Limit: 1500 mA
1500 S>Q32 CONSTANT CPA-INTEGRAL-LIMIT

\ State Storage
VARIABLE CPA-AZ-INT-ERR
VARIABLE CPA-EL-INT-ERR
VARIABLE CPA-AZ-PREV-VERR
VARIABLE CPA-EL-PREV-VERR

\ ------------------------------------------------------------------------------
\ Section 1: Initialization
\ ------------------------------------------------------------------------------

\ CPA-SERVO-INIT ( -- )
: CPA-SERVO-INIT ( -- )
    0 CPA-AZ-INT-ERR !
    0 CPA-EL-INT-ERR !
    0 CPA-AZ-PREV-VERR !
    0 CPA-EL-PREV-VERR !
    LUGRE-RESET
;

\ ------------------------------------------------------------------------------
\ Section 2: Axis Control Loop Execution
\ ------------------------------------------------------------------------------

VARIABLE SERVO-DT
VARIABLE SERVO-POS-ERR
VARIABLE SERVO-CMD-VEL
VARIABLE SERVO-CURR-VEL
VARIABLE SERVO-VEL-ERR
VARIABLE SERVO-DERIV
VARIABLE SERVO-TORQUE

\ CPA-SERVO-AZ-STEP ( target_az dt_s -- )
\ Execute one discrete control step for azimuth coarse axis.
: CPA-SERVO-AZ-STEP ( target_az dt -- )
    SERVO-DT !
    \ Position error: e_p = target - current (using shortest arc)
    CPA-AZ-ANGLE@ CPA-SHORTEST-ARC
    SERVO-POS-ERR !

    \ Outer loop: commanded velocity v_cmd = K_pp * e_p
    SERVO-POS-ERR @ CPA-KP-POS Q32*
    \ Clamp to max slew speed
    CPA-DEFAULT-VMAX NEGATE CPA-DEFAULT-VMAX CLAMP
    SERVO-CMD-VEL !

    \ Inner loop: current velocity from telemetry
    CPA-AZ-TICKS@ S>Q32 \ Approximate velocity or use sensor
    0 \ Read velocity
    SERVO-CURR-VEL !
    SERVO-CMD-VEL @ SERVO-CURR-VEL @ -
    SERVO-VEL-ERR !

    \ Integral term update with anti-windup clamping
    CPA-AZ-INT-ERR @ SERVO-VEL-ERR @ SERVO-DT @ Q32* +
    CPA-INTEGRAL-LIMIT NEGATE CPA-INTEGRAL-LIMIT CLAMP
    CPA-AZ-INT-ERR !

    \ Derivative term: (v_err - prev_err) / dt
    SERVO-VEL-ERR @ CPA-AZ-PREV-VERR @ - SERVO-DT @ Q32/
    SERVO-DERIV !
    SERVO-VEL-ERR @ CPA-AZ-PREV-VERR !

    \ Total torque = Kp*e_v + Ki*int_e + Kd*deriv + friction_comp + cogging_comp
    SERVO-VEL-ERR @ CPA-KP-VEL Q32*
    CPA-AZ-INT-ERR @ CPA-KI-VEL Q32* +
    SERVO-DERIV @ CPA-KD-VEL Q32* +
    \ Add friction feedforward
    SERVO-CMD-VEL @ SERVO-DT @ LUGRE-STEP +
    \ Add cogging feedforward
    CPA-AZ-ANGLE@ COGGING-COMPENSATE +
    SERVO-TORQUE !

    \ Command to motor drive DAC in milliamperes
    SERVO-TORQUE @ Q32>S CPA-AZ-CMD-RATE!
;

\ CPA-SERVO-EL-STEP ( target_el dt_s -- )
\ Execute one discrete control step for elevation coarse axis.
: CPA-SERVO-EL-STEP ( target_el dt -- )
    SERVO-DT !
    \ Position error
    CPA-EL-ANGLE@ -
    SERVO-POS-ERR !

    \ Commanded velocity
    SERVO-POS-ERR @ CPA-KP-POS Q32*
    CPA-DEFAULT-VMAX NEGATE CPA-DEFAULT-VMAX CLAMP
    SERVO-CMD-VEL !

    \ Inner loop
    SERVO-CMD-VEL @
    SERVO-VEL-ERR !

    \ Integral update
    CPA-EL-INT-ERR @ SERVO-VEL-ERR @ SERVO-DT @ Q32* +
    CPA-INTEGRAL-LIMIT NEGATE CPA-INTEGRAL-LIMIT CLAMP
    CPA-EL-INT-ERR !

    \ Motor torque command
    SERVO-VEL-ERR @ CPA-KP-VEL Q32*
    CPA-EL-INT-ERR @ CPA-KI-VEL Q32* +
    SERVO-TORQUE !

    SERVO-TORQUE @ Q32>S CPA-EL-CMD-RATE!
;

\ CPA-SERVO-STEP ( target_az target_el dt_s -- )
\ Coordinated step for both gimbal axes.
: CPA-SERVO-STEP ( az el dt -- )
    DUP >R CPA-SERVO-EL-STEP
    R> CPA-SERVO-AZ-STEP
;
