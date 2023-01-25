\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/orbit/los_geometry.fs
\ Description: Inter-satellite Line-Of-Sight (LOS) vector kinematics, range,
\              range-rate, optical Doppler shift, transit time, and topocentric
\              azimuth/elevation line-of-sight angles.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: LOS Relative State Vectors
\ ------------------------------------------------------------------------------

VARIABLE LOS-REL-X-KM-Q32
VARIABLE LOS-REL-Y-KM-Q32
VARIABLE LOS-REL-Z-KM-Q32

VARIABLE LOS-VREL-X-KM-S-Q32
VARIABLE LOS-VREL-Y-KM-S-Q32
VARIABLE LOS-VREL-Z-KM-S-Q32

VARIABLE LOS-RANGE-KM-Q32       \ Range rho = ||r_rx - r_tx|| (km Q32.32)
VARIABLE LOS-RANGE-RATE-KM-S-Q32 \ Range-rate drho/dt = v_rel . u_los (km/s Q32.32)
VARIABLE LOS-TRANSIT-TIME-SEC-Q32 \ Light transit time tau = rho / c (seconds Q32.32)
VARIABLE LOS-DOPPLER-SHIFT-MHZ-Q32 \ Optical carrier frequency shift (MHz Q32.32)

\ Normalized LOS Unit Vector (ECI frame)
VARIABLE LOS-U-X-Q32
VARIABLE LOS-U-Y-Q32
VARIABLE LOS-U-Z-Q32

\ Commanded Line-Of-Sight Pointing Angles (radians Q32.32)
VARIABLE LOS-AZ-RAD-Q32
VARIABLE LOS-EL-RAD-Q32

\ ------------------------------------------------------------------------------
\ Section 2: Relative Vector and Doppler Calculation
\ ------------------------------------------------------------------------------

VARIABLE LOS-TX-X VARIABLE LOS-TX-Y VARIABLE LOS-TX-Z
VARIABLE LOS-RX-X VARIABLE LOS-RX-Y VARIABLE LOS-RX-Z

\ LOS-COMPUTE-RELATIVE ( tx_x tx_y tx_z rx_x rx_y rx_z -- )
\ Calculate relative separation vector and range between two spacecraft.
: LOS-COMPUTE-RELATIVE ( tx_x tx_y tx_z rx_x rx_y rx_z -- )
    LOS-RX-Z ! LOS-RX-Y ! LOS-RX-X !
    LOS-TX-Z ! LOS-TX-Y ! LOS-TX-X !

    LOS-RX-X @ LOS-TX-X @ - LOS-REL-X-KM-Q32 !
    LOS-RX-Y @ LOS-TX-Y @ - LOS-REL-Y-KM-Q32 !
    LOS-RX-Z @ LOS-TX-Z @ - LOS-REL-Z-KM-Q32 !

    \ Compute range: rho = sqrt(dx^2 + dy^2 + dz^2)
    LOS-REL-X-KM-Q32 @ LOS-REL-Y-KM-Q32 @ LOS-REL-Z-KM-Q32 @
    V3-NORM
    DUP LOS-RANGE-KM-Q32 !

    \ Normalized unit LOS vector: u = rho_vec / rho
    DUP 0> IF
        DUP >R
        LOS-REL-X-KM-Q32 @ R@ Q32/ LOS-U-X-Q32 !
        LOS-REL-Y-KM-Q32 @ R@ Q32/ LOS-U-Y-Q32 !
        LOS-REL-Z-KM-Q32 @ R> Q32/ LOS-U-Z-Q32 !
    ELSE
        DROP
        0 LOS-U-X-Q32 ! 0 LOS-U-Y-Q32 ! 0 LOS-U-Z-Q32 !
    THEN

    \ One-way transit time: tau = rho_km / c_km_s
    \ c = 299,792.458 km/s
    LOS-RANGE-KM-Q32 @ 299792 S>Q32 Q32/
    LOS-TRANSIT-TIME-SEC-Q32 !
;

\ LOS-COMPUTE-DOPPLER ( vtx_x vtx_y vtx_z vrx_x vrx_y vrx_z -- )
\ Calculate relative velocity, range-rate, and optical Doppler shift in MHz.
: LOS-COMPUTE-DOPPLER ( vtx_x vtx_y vtx_z vrx_x vrx_y vrx_z -- )
    LOS-RX-Z ! LOS-RX-Y ! LOS-RX-X !
    LOS-TX-Z ! LOS-TX-Y ! LOS-TX-X !

    LOS-RX-X @ LOS-TX-X @ - LOS-VREL-X-KM-S-Q32 !
    LOS-RX-Y @ LOS-TX-Y @ - LOS-VREL-Y-KM-S-Q32 !
    LOS-RX-Z @ LOS-TX-Z @ - LOS-VREL-Z-KM-S-Q32 !

    \ Range-rate: drho/dt = v_rel . u_los
    LOS-VREL-X-KM-S-Q32 @ LOS-U-X-Q32 @ Q32*
    LOS-VREL-Y-KM-S-Q32 @ LOS-U-Y-Q32 @ Q32* +
    LOS-VREL-Z-KM-S-Q32 @ LOS-U-Z-Q32 @ Q32* +
    DUP LOS-RANGE-RATE-KM-S-Q32 !

    \ Doppler shift: delta_f = f0 * (range_rate / c)
    \ f0 = 193.4145 THz = 193,414,500 MHz
    \ delta_f_MHz = 193414500 * (range_rate_km_s / 299792.458)
    \ Factor: 193414500 / 299792.458 = 645.16 MHz per km/s
    \ In Q32.32: 645.1613 * 2^32 = 2770932224
    2770932224 Q32*
    LOS-DOPPLER-SHIFT-MHZ-Q32 !
;

\ ------------------------------------------------------------------------------
\ Section 3: Azimuth and Elevation LOS Solver
\ ------------------------------------------------------------------------------

\ LOS-SOLVE-ANGLES ( -- az_rad_q32 el_rad_q32 )
\ Derive azimuth and elevation line-of-sight pointing angles from unit vector.
: LOS-SOLVE-ANGLES ( -- az el )
    \ Azimuth: atan2(u_y, u_x)
    LOS-U-Y-Q32 @ LOS-U-X-Q32 @ CORDIC-ATAN2
    DUP LOS-AZ-RAD-Q32 !

    \ Elevation: asin(u_z) = atan2(u_z, sqrt(u_x^2 + u_y^2))
    LOS-U-X-Q32 @ DUP Q32*
    LOS-U-Y-Q32 @ DUP Q32* +
    CORDIC-SQRT
    LOS-U-Z-Q32 @ SWAP CORDIC-ATAN2
    DUP LOS-EL-RAD-Q32 !
;

\ LOS-PRINT-SUMMARY ( -- )
\ Display comprehensive line-of-sight geometric parameters and Doppler.
: LOS-PRINT-SUMMARY ( -- )
    CR ." [INTER-SATELLITE LOS GEOMETRY]" CR
    ."   Range:         " LOS-RANGE-KM-Q32 @ Q32>S . ." km" CR
    ."   Transit Time:  " LOS-TRANSIT-TIME-SEC-Q32 @ 1000000 * Q32>S . ." us" CR
    ."   Range-Rate:    " LOS-RANGE-RATE-KM-S-Q32 @ Q32>S . ." km/s" CR
    ."   Doppler Shift: " LOS-DOPPLER-SHIFT-MHZ-Q32 @ Q32>S . ." MHz" CR
    ."   Azimuth Angle: " LOS-AZ-RAD-Q32 @ RAD>DEG-Q32 Q32>S . ." deg" CR
    ."   Elevation:     " LOS-EL-RAD-Q32 @ RAD>DEG-Q32 Q32>S . ." deg" CR
;
