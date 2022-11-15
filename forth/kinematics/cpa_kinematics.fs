\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/cpa_kinematics.fs
\ Description: Coarse Pointing Assembly (CPA) 2-axis gimbal forward and inverse
\              kinematics, coordinate frames, and zenith singularity avoidance.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ CPA Gimbal Coordinate System:
\ Elevation-over-Azimuth mounting geometry:
\ Azimuth (psi):   Rotation around spacecraft body Z-axis (yaw)
\ Elevation (theta): Rotation around trunnion tilt axis Y' (pitch)
\
\ Forward Kinematics (Angles to LOS Unit Vector):
\   u_x = cos(theta) * cos(psi)
\   u_y = cos(theta) * sin(psi)
\   u_z = sin(theta)
\
\ Inverse Kinematics (LOS Unit Vector to Angles):
\   psi   = atan2(u_y, u_x)
\   theta = atan2(u_z, sqrt(u_x^2 + u_y^2))

\ Keyhole Singularity Limit:
\ When elevation angle approaches 90 deg (zenith), cos(theta) -> 0.
\ Required azimuth rate psi_dot = theta_dot_cross / cos(theta) diverges.
\ Near zenith (+/- 1.5 deg = 0.026 rad), gimbal lock avoidance is engaged.
88 S>Q32 DEG>RAD-Q32 CONSTANT CPA-ZENITH-LIMIT-RAD
42949673 CONSTANT CPA-KEYHOLE-THRESHOLD-RAD \ ~0.01 rad (0.57 deg)

\ ------------------------------------------------------------------------------
\ Section 1: Forward Kinematics Solver
\ ------------------------------------------------------------------------------

VARIABLE CPA-FWD-AZ
VARIABLE CPA-FWD-EL
VARIABLE CPA-COS-EL
VARIABLE CPA-SIN-EL
VARIABLE CPA-COS-AZ
VARIABLE CPA-SIN-AZ

\ CPA-ANGLES>LOS ( az_rad el_rad -- ux uy uz )
\ Transform gimbal azimuth and elevation angles into line-of-sight unit vector.
: CPA-ANGLES>LOS ( az el -- ux uy uz )
    CPA-FWD-EL !
    CPA-FWD-AZ !

    \ Compute trigonometric functions
    CPA-FWD-EL @ CORDIC-SIN-COS
    CPA-COS-EL ! CPA-SIN-EL !

    CPA-FWD-AZ @ CORDIC-SIN-COS
    CPA-COS-AZ ! CPA-SIN-AZ !

    \ ux = cos(el) * cos(az)
    CPA-COS-EL @ CPA-COS-AZ @ Q32*

    \ uy = cos(el) * sin(az)
    CPA-COS-EL @ CPA-SIN-AZ @ Q32*

    \ uz = sin(el)
    CPA-SIN-EL @
;

\ ------------------------------------------------------------------------------
\ Section 2: Inverse Kinematics Solver with Keyhole Protection
\ ------------------------------------------------------------------------------

VARIABLE CPA-INV-X
VARIABLE CPA-INV-Y
VARIABLE CPA-INV-Z
VARIABLE CPA-RHO

\ CPA-LOS>ANGLES ( ux uy uz -- az_rad el_rad )
\ Transform line-of-sight unit vector into gimbal azimuth and elevation angles.
: CPA-LOS>ANGLES ( ux uy uz -- az el )
    CPA-INV-Z !
    CPA-INV-Y !
    CPA-INV-X !

    \ Compute horizontal projection: rho = sqrt(ux^2 + uy^2)
    CPA-INV-X @ DUP Q32*
    CPA-INV-Y @ DUP Q32* +
    CORDIC-SQRT
    CPA-RHO !

    \ Check for zenith singular keyhole
    CPA-RHO @ CPA-KEYHOLE-THRESHOLD-RAD < IF
        \ In zenith singularity: preserve current azimuth to avoid high-rate spin
        CPA-AZ-ANGLE@
        CPA-INV-Z @ 0> IF Q32-HALF-PI ELSE Q32-HALF-PI NEGATE THEN
        EXIT
    THEN

    \ Compute azimuth: psi = atan2(uy, ux)
    CPA-INV-Y @ CPA-INV-X @ CORDIC-ATAN2

    \ Compute elevation: theta = atan2(uz, rho)
    CPA-INV-Z @ CPA-RHO @ CORDIC-ATAN2
;

\ ------------------------------------------------------------------------------
\ Section 3: Azimuth Shortest Path Angle Wrapping
\ ------------------------------------------------------------------------------

\ CPA-SHORTEST-ARC ( current_az target_az -- delta_az )
\ Compute minimal angular difference in [-pi, +pi] to prevent unnecessary 360 unwinding.
: CPA-SHORTEST-ARC ( cur target -- delta )
    SWAP -
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
;

\ CPA-TARGET-UNWRAP ( current_az target_az -- unwrapped_target )
\ Return target azimuth unwrapped relative to current position.
: CPA-TARGET-UNWRAP ( cur target -- unwrapped )
    2DUP CPA-SHORTEST-ARC +
    NIP
;
