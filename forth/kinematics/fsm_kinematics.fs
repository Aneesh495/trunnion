\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/fsm_kinematics.fs
\ Description: Fast Steering Mirror (FSM) tip-tilt to 3-piezo tripod kinematic
\              transformations and push-pull actuator coordinate mapping.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ FSM 3-Piezo Equilateral Tripod Geometry:
\ Three high-voltage piezoelectric stack actuators positioned on radius R = 15.0 mm:
\   Piezo 1 at phi_1 =   0 deg (x = +R, y = 0)
\   Piezo 2 at phi_2 = 120 deg (x = -R/2, y = +R*sqrt(3)/2)
\   Piezo 3 at phi_3 = 240 deg (x = -R/2, y = -R*sqrt(3)/2)
\
\ Actuator stroke delta_z_i = R * (theta_x * cos(phi_i) + theta_y * sin(phi_i))
\ Actuator voltage V_i = V_bias + K_piezo * delta_z_i
\
\ In inverse transformation:
\   theta_x = (2*delta_z_1 - delta_z_2 - delta_z_3) / (3*R)
\   theta_y = (delta_z_2 - delta_z_3) / (sqrt(3)*R)

\ sqrt(3)/2 in Q32.32 = 0.86602540378 * 2^32 = 3719572520
3719572520 CONSTANT FSM-SQRT3-HALF-Q32
\ Half in Q32.32
$80000000 CONSTANT FSM-HALF-Q32

VARIABLE FSM-KIN-TIP
VARIABLE FSM-KIN-TILT
VARIABLE FSM-V1
VARIABLE FSM-V2
VARIABLE FSM-V3

\ ------------------------------------------------------------------------------
\ Section 1: Angles to Actuator Push-Pull Drives
\ ------------------------------------------------------------------------------

\ FSM-ANGLES>PIEZO ( tip_nrad tilt_nrad -- v1 v2 v3 )
\ Convert mirror tip/tilt angles to three push-pull actuator DAC voltages (0..65535).
: FSM-ANGLES>PIEZO ( tip tilt -- v1 v2 v3 )
    FSM-KIN-TILT !
    FSM-KIN-TIP !

    \ Delta 1: +tip
    FSM-KIN-TIP @ S>Q32
    \ Scaled to DAC counts: 32768 + tip * scale
    FSM-MAX-NRAD S>Q32 Q32/
    32000 S>Q32 Q32* Q32>S
    32768 +
    FSM-V1 !

    \ Delta 2: -0.5*tip + sqrt(3)/2 * tilt
    FSM-KIN-TIP @ S>Q32 FSM-HALF-Q32 Q32* NEGATE
    FSM-KIN-TILT @ S>Q32 FSM-SQRT3-HALF-Q32 Q32* +
    FSM-MAX-NRAD S>Q32 Q32/
    32000 S>Q32 Q32* Q32>S
    32768 +
    FSM-V2 !

    \ Delta 3: -0.5*tip - sqrt(3)/2 * tilt
    FSM-KIN-TIP @ S>Q32 FSM-HALF-Q32 Q32* NEGATE
    FSM-KIN-TILT @ S>Q32 FSM-SQRT3-HALF-Q32 Q32* -
    FSM-MAX-NRAD S>Q32 Q32/
    32000 S>Q32 Q32* Q32>S
    32768 +
    FSM-V3 !

    FSM-V1 @ 0 FSM-DAC-MAX CLAMP
    FSM-V2 @ 0 FSM-DAC-MAX CLAMP
    FSM-V3 @ 0 FSM-DAC-MAX CLAMP
;

\ ------------------------------------------------------------------------------
\ Section 2: Sensor Feedback Transformation
\ ------------------------------------------------------------------------------

\ FSM-PIEZO>ANGLES ( v1 v2 v3 -- tip_nrad tilt_nrad )
\ Convert actuator capacitive readbacks to tip/tilt angles in nanoradians.
: FSM-PIEZO>ANGLES ( v1 v2 v3 -- tip tilt )
    32768 - S>Q32 32000 S>Q32 Q32/ FSM-MAX-NRAD S>Q32 Q32* Q32>S >R \ z3
    32768 - S>Q32 32000 S>Q32 Q32/ FSM-MAX-NRAD S>Q32 Q32* Q32>S >R \ z2
    32768 - S>Q32 32000 S>Q32 Q32/ FSM-MAX-NRAD S>Q32 Q32* Q32>S \ z1

    \ tip = (2*z1 - z2 - z3) / 3
    2 * R@ - 2 PICK - 3 / \ Stack: ( tip )

    \ tilt = (z2 - z3) / sqrt(3) ~ (z2 - z3) * 0.57735
    R> R> - S>Q32 2479715014 Q32* Q32>S \ Stack: ( tip tilt )
;

\ ------------------------------------------------------------------------------
\ Section 3: Hardware Actuation Dispatch
\ ------------------------------------------------------------------------------

\ FSM-ACTUATE-ANGLES ( tip_nrad tilt_nrad -- )
\ Compute tripod voltages and write directly to hardware DAC registers.
: FSM-ACTUATE-ANGLES ( tip tilt -- )
    2DUP FSM-TILT-CMD! FSM-TIP-CMD!
    FSM-ANGLES>PIEZO
    REG-FSM-ACT3-DAC HW!
    REG-FSM-ACT2-DAC HW!
    REG-FSM-ACT1-DAC HW!
;
