\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kinematics/cpa_friction.fs
\ Description: Non-linear dynamic LuGre friction observer and harmonic drive
\              cogging torque feedforward compensator for gimbal bearings.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ LuGre Friction Parameters (calibrated for spaceflight trunnion bearings):
\ Bristle stiffness sigma_0 = 1000.0 N*m/rad in Q32.32
4294967296000 CONSTANT LUGRE-SIGMA-0
\ Bristle micro-damping sigma_1 = 50.0 N*m*s/rad in Q32.32
214748364800 CONSTANT LUGRE-SIGMA-1
\ Viscous friction sigma_v = 0.5 N*m*s/rad in Q32.32
2147483648 CONSTANT LUGRE-SIGMA-V
\ Coulomb friction F_c = 0.08 N*m in Q32.32
343597383 CONSTANT LUGRE-FC
\ Static breakaway friction F_s = 0.12 N*m in Q32.32
515396075 CONSTANT LUGRE-FS
\ Stribeck velocity v_s = 0.005 rad/s in Q32.32
21474836 CONSTANT LUGRE-VS

VARIABLE LUGRE-STATE-Z     \ Average bristle deflection (Q32.32 rad)
VARIABLE LUGRE-LAST-VEL    \ Previous velocity sample

\ ------------------------------------------------------------------------------
\ Section 1: Dynamic Friction Observer
\ ------------------------------------------------------------------------------

VARIABLE LUGRE-VEL
VARIABLE LUGRE-DT
VARIABLE LUGRE-G-V
VARIABLE LUGRE-DZ-DT

\ LUGRE-RESET ( -- )
\ Clear internal bristle state.
: LUGRE-RESET ( -- )
    0 LUGRE-STATE-Z !
    0 LUGRE-LAST-VEL !
;

\ LUGRE-STEP ( vel_q32 dt_s_q32 -- torque_q32 )
\ Integrate dynamic LuGre friction equation and return estimated friction torque.
: LUGRE-STEP ( vel dt -- torque )
    LUGRE-DT !
    LUGRE-VEL !

    \ Compute Stribeck curve g(v) = Fc + (Fs - Fc) * exp(-(v/vs)^2)
    LUGRE-VEL @ DUP Q32* LUGRE-VS DUP Q32* Q32/ \ (v/vs)^2
    DUP 4 S>Q32 > IF
        \ When (v/vs)^2 is large, exp term approaches 0
        DROP LUGRE-FC LUGRE-G-V !
    ELSE
        \ 2nd-order Taylor approximation: exp(-x) ~ 1 - x + 0.5*x^2
        DUP DUP Q32* 2 S>Q32 Q32/ SWAP - Q32-ONE SWAP - \ 1 - x + 0.5*x^2
        0 S>Q32 MAX
        LUGRE-FS LUGRE-FC - Q32* LUGRE-FC +
        LUGRE-G-V !
    THEN

    \ dz/dt = vel - (sigma_0 * |vel| / g(v)) * z
    LUGRE-SIGMA-0 LUGRE-VEL @ ABS Q32*
    LUGRE-G-V @ Q32/
    LUGRE-STATE-Z @ Q32*
    LUGRE-VEL @ SWAP -
    LUGRE-DZ-DT !

    \ Update bristle deflection: z = z + dz/dt * dt
    LUGRE-STATE-Z @ LUGRE-DZ-DT @ LUGRE-DT @ Q32* +
    LUGRE-STATE-Z !

    \ Friction torque = sigma_0*z + sigma_1*dz/dt + sigma_v*vel
    LUGRE-SIGMA-0 LUGRE-STATE-Z @ Q32*
    LUGRE-SIGMA-1 LUGRE-DZ-DT @ Q32* +
    LUGRE-SIGMA-V LUGRE-VEL @ Q32* +
;

\ ------------------------------------------------------------------------------
\ Section 2: Harmonic Drive Cogging Torque Cancellation
\ ------------------------------------------------------------------------------

\ Cogging torque amplitude (N*m in Q32.32)
\ Typically ~0.015 N*m at 100x gear tooth meshing frequency
64424509 CONSTANT COGGING-AMP-Q32
100 CONSTANT GEAR-TOOTH-HARMONIC

\ COGGING-COMPENSATE ( angle_rad_q32 -- torque_q32 )
\ Compute feedforward cogging torque cancellation: -A * sin(N * theta).
: COGGING-COMPENSATE ( angle -- torque )
    GEAR-TOOTH-HARMONIC S>Q32 Q32*
    CORDIC-SIN
    COGGING-AMP-Q32 Q32*
    NEGATE
;
