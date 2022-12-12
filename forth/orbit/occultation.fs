\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/orbit/occultation.fs
\ Description: Earth limb occultation geometry with 100 km atmospheric refraction
\              buffer, Solar blinding keep-out exclusion, and Lunar interference.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Occultation State and Coordinates
\ ------------------------------------------------------------------------------

VARIABLE OCC-RAY-MIN-DIST-KM-Q32 \ Closest approach distance of ray to Earth center
VARIABLE OCC-EARTH-BLOCKED       \ Boolean flag: Earth blocks line of sight
VARIABLE OCC-SUN-BLOCKED         \ Boolean flag: Sun within 15 deg exclusion cone
VARIABLE OCC-MOON-BLOCKED        \ Boolean flag: Moon within 5 deg exclusion cone

VARIABLE OCC-SUN-U-X-Q32  VARIABLE OCC-SUN-U-Y-Q32  VARIABLE OCC-SUN-U-Z-Q32
VARIABLE OCC-MOON-U-X-Q32 VARIABLE OCC-MOON-U-Y-Q32 VARIABLE OCC-MOON-U-Z-Q32

VARIABLE OCC-T-MIN-Q32
VARIABLE OCC-R-MIN-X-Q32  VARIABLE OCC-R-MIN-Y-Q32  VARIABLE OCC-R-MIN-Z-Q32

\ ------------------------------------------------------------------------------
\ Section 2: Celestial Bodies Direction Vectors Setup
\ ------------------------------------------------------------------------------

\ OCC-SET-SUN-VECTOR ( ux uy uz -- )
\ Set normalized unit vector toward the Sun in ECI frame.
: OCC-SET-SUN-VECTOR ( ux uy uz -- )
    OCC-SUN-U-Z-Q32 !
    OCC-SUN-U-Y-Q32 !
    OCC-SUN-U-X-Q32 !
;

\ OCC-SET-MOON-VECTOR ( ux uy uz -- )
\ Set normalized unit vector toward the Moon in ECI frame.
: OCC-SET-MOON-VECTOR ( ux uy uz -- )
    OCC-MOON-U-Z-Q32 !
    OCC-MOON-U-Y-Q32 !
    OCC-MOON-U-X-Q32 !
;

\ ------------------------------------------------------------------------------
\ Section 3: Ray-Sphere Earth Occultation Test
\ ------------------------------------------------------------------------------

\ OCC-CHECK-EARTH ( tx_x tx_y tx_z rx_x rx_y rx_z -- blocked_flag )
\ Determine whether the laser cross-link intersects the Earth's atmosphere.
: OCC-CHECK-EARTH ( tx_x tx_y tx_z rx_x rx_y rx_z -- flag )
    \ Compute relative vector rho = rx - tx
    6DUP
    LOS-COMPUTE-RELATIVE

    \ If range is zero, no occultation
    LOS-RANGE-KM-Q32 @ 0= IF
        6DROP 0 OCC-EARTH-BLOCKED ! 0 EXIT
    THEN

    \ Parameter t_min along segment:
    \ t_min = - (r_tx . rho) / ||rho||^2
    \ Dot product r_tx . rho:
    2DROP 2DROP \ Drop rx_z rx_y rx_x
    \ Stack: ( tx_x tx_y tx_z )
    DUP LOS-REL-Z-KM-Q32 @ Q32* >R
    OVER LOS-REL-Y-KM-Q32 @ Q32* R> + >R
    2 PICK LOS-REL-X-KM-Q32 @ Q32* R> +
    \ Negate: - (r_tx . rho)
    NEGATE

    \ Divide by range squared: ||rho||^2
    LOS-RANGE-KM-Q32 @ DUP Q32*
    Q32/

    \ Clamp t_min to inclusive segment [0, 1]
    0 S>Q32 Q32-ONE CLAMP
    DUP OCC-T-MIN-Q32 !

    \ Calculate closest approach point on segment: r(t_min) = r_tx + t_min * rho
    \ Z-axis:
    DUP LOS-REL-Z-KM-Q32 @ Q32* ROT + OCC-R-MIN-Z-Q32 !
    \ Y-axis:
    DUP LOS-REL-Y-KM-Q32 @ Q32* ROT + OCC-R-MIN-Y-Q32 !
    \ X-axis:
    LOS-REL-X-KM-Q32 @ Q32* ROT + OCC-R-MIN-X-Q32 !

    \ Compute distance: d_min = ||r(t_min)||
    OCC-R-MIN-X-Q32 @ OCC-R-MIN-Y-Q32 @ OCC-R-MIN-Z-Q32 @
    V3-NORM
    DUP OCC-RAY-MIN-DIST-KM-Q32 !

    \ Check against effective occultation radius: R_earth + 100 km (6,478 km)
    ORB-OCCULT-RADIUS-KM-Q32 < IF
        1 OCC-EARTH-BLOCKED !
        1
    ELSE
        0 OCC-EARTH-BLOCKED !
        0
    THEN
;

\ ------------------------------------------------------------------------------
\ Section 4: Solar and Lunar Keep-Out Angle Checks
\ ------------------------------------------------------------------------------

\ OCC-CHECK-SUN ( -- blocked_flag )
\ Check if Sun falls within the 15-degree detector blinding exclusion cone.
: OCC-CHECK-SUN ( -- flag )
    \ Dot product: cos(theta) = u_los . u_sun
    LOS-U-X-Q32 @ OCC-SUN-U-X-Q32 @ Q32*
    LOS-U-Y-Q32 @ OCC-SUN-U-Y-Q32 @ Q32* +
    LOS-U-Z-Q32 @ OCC-SUN-U-Z-Q32 @ Q32* +

    \ cos(15 deg) = 0.965925826 = 4148641950 in Q32.32
    \ If cos(theta) > cos(15 deg), angle is within 15 deg
    4148641950 >= IF
        1 OCC-SUN-BLOCKED !
        1
    ELSE
        0 OCC-SUN-BLOCKED !
        0
    THEN
;

\ OCC-CHECK-MOON ( -- blocked_flag )
\ Check if Moon falls within the 5-degree noise floor exclusion cone.
: OCC-CHECK-MOON ( -- flag )
    LOS-U-X-Q32 @ OCC-MOON-U-X-Q32 @ Q32*
    LOS-U-Y-Q32 @ OCC-MOON-U-Y-Q32 @ Q32* +
    LOS-U-Z-Q32 @ OCC-MOON-U-Z-Q32 @ Q32* +

    \ cos(5 deg) = 0.996194698 = 4278619600 in Q32.32
    4278619600 >= IF
        1 OCC-MOON-BLOCKED !
        1
    ELSE
        0 OCC-MOON-BLOCKED !
        0
    THEN
;

\ OCC-LINK-FEASIBLE? ( -- flag )
\ Return TRUE if link is free from Earth limb, Solar, and Lunar occultation.
: OCC-LINK-FEASIBLE? ( -- flag )
    OCC-EARTH-BLOCKED @ 0=
    OCC-SUN-BLOCKED @ 0= AND
    OCC-MOON-BLOCKED @ 0= AND
;
