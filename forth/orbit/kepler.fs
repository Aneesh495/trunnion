\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/orbit/kepler.fs
\ Description: Keplerian two-body orbital propagation, fixed-point Newton-Raphson
\              eccentric anomaly solver, perifocal state calculation, and
\              Perifocal-to-ECI coordinate transformation.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Classical Keplerian Elements State Structure
\ ------------------------------------------------------------------------------

VARIABLE ORB-SMA-KM-Q32         \ Semi-major axis a (km Q32.32)
VARIABLE ORB-ECC-Q32            \ Eccentricity e (dimensionless Q32.32)
VARIABLE ORB-INC-RAD-Q32        \ Inclination i (radians Q32.32)
VARIABLE ORB-RAAN-RAD-Q32       \ Right Ascension of Ascending Node Omega (radians Q32.32)
VARIABLE ORB-AOP-RAD-Q32        \ Argument of Perigee omega (radians Q32.32)
VARIABLE ORB-MA-RAD-Q32         \ Mean Anomaly M (radians Q32.32)
VARIABLE ORB-EPOCH-SEC          \ Epoch timestamp in seconds

\ Computed Intermediate Parameters
VARIABLE ORB-MEAN-MOTION-Q32    \ Mean motion n = sqrt(mu / a^3) (rad/s Q32.32)
VARIABLE ORB-SEMI-LATUS-KM-Q32  \ Semi-latus rectum p = a*(1 - e^2) (km Q32.32)
VARIABLE ORB-ECC-ANOM-Q32       \ Eccentric Anomaly E (radians Q32.32)
VARIABLE ORB-TRUE-ANOM-Q32      \ True Anomaly nu (radians Q32.32)
VARIABLE ORB-RADIUS-KM-Q32      \ Orbital distance r (km Q32.32)

\ Perifocal Frame State Vectors (P-frame: P, Q, W)
VARIABLE ORB-RP-X-Q32  VARIABLE ORB-RP-Y-Q32  VARIABLE ORB-RP-Z-Q32
VARIABLE ORB-VP-X-Q32  VARIABLE ORB-VP-Y-Q32  VARIABLE ORB-VP-Z-Q32

\ ECI Frame State Vectors (I, J, K)
VARIABLE ORB-ECI-X-Q32 VARIABLE ORB-ECI-Y-Q32 VARIABLE ORB-ECI-Z-Q32
VARIABLE ORB-ECI-VX-Q32 VARIABLE ORB-ECI-VY-Q32 VARIABLE ORB-ECI-VZ-Q32

\ ------------------------------------------------------------------------------
\ Section 2: Mean Motion and Orbit Setup
\ ------------------------------------------------------------------------------

\ ORB-SET-ELEMENTS ( a_km e inc_rad raan_rad aop_rad m0_rad -- )
\ Configure initial Keplerian orbital elements.
: ORB-SET-ELEMENTS ( a e inc raan aop m0 -- )
    ORB-MA-RAD-Q32 !
    ORB-AOP-RAD-Q32 !
    ORB-RAAN-RAD-Q32 !
    ORB-INC-RAD-Q32 !
    ORB-ECC-Q32 !
    ORB-SMA-KM-Q32 !

    \ Compute semi-latus rectum: p = a * (1 - e^2)
    Q32-ONE ORB-ECC-Q32 @ DUP Q32* -
    ORB-SMA-KM-Q32 @ Q32*
    ORB-SEMI-LATUS-KM-Q32 !

    \ Mean motion: n = sqrt(mu / a) / a
    ORB-MU-KM3-S2-Q32 ORB-SMA-KM-Q32 @ Q32/
    CORDIC-SQRT
    ORB-SMA-KM-Q32 @ Q32/
    ORB-MEAN-MOTION-Q32 !
;

\ ------------------------------------------------------------------------------
\ Section 3: Kepler's Equation Solver (Newton-Raphson Iteration)
\ ------------------------------------------------------------------------------

VARIABLE KEPLER-M
VARIABLE KEPLER-E
VARIABLE KEPLER-CURR-E
VARIABLE KEPLER-SIN-E
VARIABLE KEPLER-COS-E

\ KEPLER-SOLVE ( m_rad_q32 e_q32 -- e_anom_rad_q32 )
\ Solve M = E - e*sin(E) for E via 6 Newton-Raphson iterations.
: KEPLER-SOLVE ( m_rad e_q32 -- e_anom )
    KEPLER-E !
    KEPLER-M !
    KEPLER-M @ KEPLER-CURR-E ! \ Initial guess: E0 = M

    6 0 DO
        \ Compute sin(E) and cos(E) using CORDIC
        KEPLER-CURR-E @ CORDIC-SIN-COS
        KEPLER-SIN-E !
        KEPLER-COS-E !

        \ Numerator: f(E) = E - e*sin(E) - M
        KEPLER-CURR-E @
        KEPLER-E @ KEPLER-SIN-E @ Q32* -
        KEPLER-M @ -

        \ Denominator: f'(E) = 1 - e*cos(E)
        Q32-ONE
        KEPLER-E @ KEPLER-COS-E @ Q32* -

        \ Delta: f(E) / f'(E)
        Q32/

        \ Update: E = E - Delta
        KEPLER-CURR-E @ SWAP -
        KEPLER-CURR-E !
    LOOP

    KEPLER-CURR-E @
;

\ ------------------------------------------------------------------------------
\ Section 4: True Anomaly and Perifocal Coordinates
\ ------------------------------------------------------------------------------

VARIABLE NU-SIN
VARIABLE NU-COS
VARIABLE NU-DENOM
VARIABLE PERI-H-FACTOR

\ ORB-COMPUTE-PERIFOCAL ( -- )
\ Calculate perifocal position and velocity vectors from orbital elements and E.
: ORB-COMPUTE-PERIFOCAL ( -- )
    \ Compute sin(E) and cos(E)
    ORB-ECC-ANOM-Q32 @ CORDIC-SIN-COS
    KEPLER-SIN-E !
    KEPLER-COS-E !

    \ Radius: r = a * (1 - e * cos(E))
    Q32-ONE ORB-ECC-Q32 @ KEPLER-COS-E @ Q32* -
    DUP NU-DENOM !
    ORB-SMA-KM-Q32 @ Q32*
    ORB-RADIUS-KM-Q32 !

    \ sin(nu) = sqrt(1 - e^2) * sin(E) / (1 - e * cos(E))
    Q32-ONE ORB-ECC-Q32 @ DUP Q32* - CORDIC-SQRT
    KEPLER-SIN-E @ Q32*
    NU-DENOM @ Q32/
    NU-SIN !

    \ cos(nu) = (cos(E) - e) / (1 - e * cos(E))
    KEPLER-COS-E @ ORB-ECC-Q32 @ -
    NU-DENOM @ Q32/
    NU-COS !

    \ True anomaly: nu = atan2(sin(nu), cos(nu))
    NU-SIN @ NU-COS @ CORDIC-ATAN2
    ORB-TRUE-ANOM-Q32 !

    \ Perifocal position:
    \ r_p = r * cos(nu) * P + r * sin(nu) * Q
    ORB-RADIUS-KM-Q32 @ NU-COS @ Q32* ORB-RP-X-Q32 !
    ORB-RADIUS-KM-Q32 @ NU-SIN @ Q32* ORB-RP-Y-Q32 !
    0 ORB-RP-Z-Q32 !

    \ Velocity factor: sqrt(mu / p)
    ORB-MU-KM3-S2-Q32 ORB-SEMI-LATUS-KM-Q32 @ Q32/
    CORDIC-SQRT
    PERI-H-FACTOR !

    \ Perifocal velocity:
    \ v_p = sqrt(mu/p) * [ -sin(nu)*P + (e + cos(nu))*Q ]
    PERI-H-FACTOR @ NU-SIN @ Q32-NEG Q32* ORB-VP-X-Q32 !
    PERI-H-FACTOR @ ORB-ECC-Q32 @ NU-COS @ + Q32* ORB-VP-Y-Q32 !
    0 ORB-VP-Z-Q32 !
;

\ ------------------------------------------------------------------------------
\ Section 5: Perifocal to Earth-Centered Inertial (ECI) Transformation
\ ------------------------------------------------------------------------------

VARIABLE ROT-SIN-O  VARIABLE ROT-COS-O  \ RAAN
VARIABLE ROT-SIN-I  VARIABLE ROT-COS-I  \ Inclination
VARIABLE ROT-SIN-W  VARIABLE ROT-COS-W  \ Arg of Perigee

CREATE ORB-DCM-P2ECI 9 CELLS ALLOT

\ ORB-BUILD-TRANSFORM ( -- )
\ Construct 3x3 direction cosine rotation matrix from Perifocal to ECI frame:
\ R = R_z(-Omega) * R_x(-i) * R_z(-omega)
: ORB-BUILD-TRANSFORM ( -- )
    ORB-RAAN-RAD-Q32 @ CORDIC-SIN-COS ROT-SIN-O ! ROT-COS-O !
    ORB-INC-RAD-Q32  @ CORDIC-SIN-COS ROT-SIN-I ! ROT-COS-I !
    ORB-AOP-RAD-Q32  @ CORDIC-SIN-COS ROT-SIN-W ! ROT-COS-W !

    \ Row 0:
    \ R00 = cos(O)*cos(w) - sin(O)*sin(w)*cos(i)
    ROT-COS-O @ ROT-COS-W @ Q32*
    ROT-SIN-O @ ROT-SIN-W @ Q32* ROT-COS-I @ Q32* -
    ORB-DCM-P2ECI 0 0 M3!

    \ R01 = -cos(O)*sin(w) - sin(O)*cos(w)*cos(i)
    ROT-COS-O @ ROT-SIN-W @ Q32* NEGATE
    ROT-SIN-O @ ROT-COS-W @ Q32* ROT-COS-I @ Q32* -
    ORB-DCM-P2ECI 0 1 M3!

    \ R02 = sin(O)*sin(i)
    ROT-SIN-O @ ROT-SIN-I @ Q32*
    ORB-DCM-P2ECI 0 2 M3!

    \ Row 1:
    \ R10 = sin(O)*cos(w) + cos(O)*sin(w)*cos(i)
    ROT-SIN-O @ ROT-COS-W @ Q32*
    ROT-COS-O @ ROT-SIN-W @ Q32* ROT-COS-I @ Q32* +
    ORB-DCM-P2ECI 1 0 M3!

    \ R11 = -sin(O)*sin(w) + cos(O)*cos(w)*cos(i)
    ROT-SIN-O @ ROT-SIN-W @ Q32* NEGATE
    ROT-COS-O @ ROT-COS-W @ Q32* ROT-COS-I @ Q32* +
    ORB-DCM-P2ECI 1 1 M3!

    \ R12 = -cos(O)*sin(i)
    ROT-COS-O @ ROT-SIN-I @ Q32* NEGATE
    ORB-DCM-P2ECI 1 2 M3!

    \ Row 2:
    \ R20 = sin(w)*sin(i)
    ROT-SIN-W @ ROT-SIN-I @ Q32*
    ORB-DCM-P2ECI 2 0 M3!

    \ R21 = cos(w)*sin(i)
    ROT-COS-W @ ROT-SIN-I @ Q32*
    ORB-DCM-P2ECI 2 1 M3!

    \ R22 = cos(i)
    ROT-COS-I @
    ORB-DCM-P2ECI 2 2 M3!
;

\ ------------------------------------------------------------------------------
\ Section 6: Orbit Propagation Step
\ ------------------------------------------------------------------------------

\ ORB-PROPAGATE ( dt_sec_q32 -- )
\ Advance orbital position by dt_sec seconds and compute new ECI state vectors.
: ORB-PROPAGATE ( dt_sec -- )
    \ Advance Mean Anomaly: M = M0 + n * dt
    ORB-MEAN-MOTION-Q32 @ Q32*
    ORB-MA-RAD-Q32 @ +
    \ Modulo 2*pi
    Q32-TWO-PI /MOD DROP
    DUP 0< IF Q32-TWO-PI + THEN
    ORB-MA-RAD-Q32 !

    \ Solve Kepler's equation for E
    ORB-MA-RAD-Q32 @ ORB-ECC-Q32 @ KEPLER-SOLVE
    ORB-ECC-ANOM-Q32 !

    \ Calculate perifocal vectors r_p and v_p
    ORB-COMPUTE-PERIFOCAL

    \ Build rotation matrix and transform to ECI frame
    ORB-BUILD-TRANSFORM

    \ Transform position: r_eci = R * r_p
    ORB-DCM-P2ECI
    ORB-RP-X-Q32 @ ORB-RP-Y-Q32 @ ORB-RP-Z-Q32 @
    M3*V3
    ORB-ECI-Z-Q32 ! ORB-ECI-Y-Q32 ! ORB-ECI-X-Q32 !

    \ Transform velocity: v_eci = R * v_p
    ORB-DCM-P2ECI
    ORB-VP-X-Q32 @ ORB-VP-Y-Q32 @ ORB-VP-Z-Q32 @
    M3*V3
    ORB-ECI-VZ-Q32 ! ORB-ECI-VY-Q32 ! ORB-ECI-VX-Q32 !
;

\ ORB-GET-ECI-POS ( -- x_km y_km z_km )
: ORB-GET-ECI-POS ( -- x y z )
    ORB-ECI-X-Q32 @ ORB-ECI-Y-Q32 @ ORB-ECI-Z-Q32 @
;

\ ORB-GET-ECI-VEL ( -- vx_km_s vy_km_s vz_km_s )
: ORB-GET-ECI-VEL ( -- vx vy vz )
    ORB-ECI-VX-Q32 @ ORB-ECI-VY-Q32 @ ORB-ECI-VZ-Q32 @
;
