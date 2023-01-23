\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/point_ahead.fs
\ Description: Special-relativistic velocity aberration and kinematic orbital
\              lookahead Point-Ahead Angle (PAA) vector calculator.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Point-Ahead Angle (PAA) Physical Theory:
\ In free-space optical inter-satellite links (OISL), laser beam divergence is
\ extremely narrow (typically 10 to 15 microradians FWHM).
\ For inter-satellite ranges of 2,000 to 5,000 km, the light transit time
\ is T_prop = R / c ~ 6.7 to 16.7 milliseconds.
\ During this transit, the target spacecraft travels:
\   Delta_r = v_target * T_prop ~ 50 to 130 meters.
\
\ Combining kinematic target displacement and special-relativistic Lorentz
\ velocity aberration:
\   theta_PA = (2 / c) * v_perp
\ Where v_perp = v_rel - (v_rel . u_LOS) * u_LOS is the transverse relative
\ velocity vector perpendicular to the line-of-sight.
\ Failure to inject theta_PA results in 100% optical link drop.

\ Speed of Light in Vacuum: c = 299,792,458 m/s
\ 2 / c in Q32.32 format: (2 / 299792458) * 2^32 = 28.6534 ~ 29
29 CONSTANT PAA-TWO-OVER-C-Q32

VARIABLE PAA-VTX-X  VARIABLE PAA-VTX-Y  VARIABLE PAA-VTX-Z
VARIABLE PAA-VRX-X  VARIABLE PAA-VRX-Y  VARIABLE PAA-VRX-Z
VARIABLE PAA-LOS-X  VARIABLE PAA-LOS-Y  VARIABLE PAA-LOS-Z
VARIABLE PAA-VREL-X VARIABLE PAA-VREL-Y VARIABLE PAA-VREL-Z
VARIABLE PAA-VPAR-X VARIABLE PAA-VPAR-Y VARIABLE PAA-VPAR-Z
VARIABLE PAA-VPERP-X VARIABLE PAA-VPERP-Y VARIABLE PAA-VPERP-Z

\ ------------------------------------------------------------------------------
\ Section 1: Relativistic Aberration Vector Computation
\ ------------------------------------------------------------------------------

\ PAA-COMPUTE ( v_tx_v3 v_rx_v3 u_los_v3 -- paa_x_rad paa_y_rad paa_z_rad )
\ Compute 3D point-ahead lead angle vector in radians (Q32.32 format).
: PAA-COMPUTE ( vtx_x vtx_y vtx_z vrx_x vrx_y vrx_z ux uy uz -- px py pz )
    PAA-LOS-Z ! PAA-LOS-Y ! PAA-LOS-X !
    PAA-VRX-Z ! PAA-VRX-Y ! PAA-VRX-X !
    PAA-VTX-Z ! PAA-VTX-Y ! PAA-VTX-X !

    \ Relative velocity v_rel = v_rx - v_tx (m/s)
    PAA-VRX-X @ PAA-VTX-X @ - PAA-VREL-X !
    PAA-VRX-Y @ PAA-VTX-Y @ - PAA-VREL-Y !
    PAA-VRX-Z @ PAA-VTX-Z @ - PAA-VREL-Z !

    \ Parallel component: v_par = (v_rel . u_LOS) * u_LOS
    PAA-VREL-X @ PAA-VREL-Y @ PAA-VREL-Z @
    PAA-LOS-X @ PAA-LOS-Y @ PAA-LOS-Z @
    V3-PROJ \ Stack: ( vpar_x vpar_y vpar_z )
    PAA-VPAR-Z ! PAA-VPAR-Y ! PAA-VPAR-X !

    \ Transverse component: v_perp = v_rel - v_par
    PAA-VREL-X @ PAA-VPAR-X @ - PAA-VPERP-X !
    PAA-VREL-Y @ PAA-VPAR-Y @ - PAA-VPERP-Y !
    PAA-VREL-Z @ PAA-VPAR-Z @ - PAA-VPERP-Z !

    \ Point-ahead angle = (2 / c) * v_perp
    PAA-VPERP-X @ PAA-TWO-OVER-C-Q32 Q32*
    PAA-VPERP-Y @ PAA-TWO-OVER-C-Q32 Q32*
    PAA-VPERP-Z @ PAA-TWO-OVER-C-Q32 Q32*
;

\ PAA-MAGNITUDE-URAD ( px py pz -- mag_urad )
\ Return magnitude of point-ahead angle vector in microradians.
: PAA-MAGNITUDE-URAD ( px py pz -- mag )
    V3-NORM
    RAD>URAD-Q32
;

\ ------------------------------------------------------------------------------
\ Section 2: Azimuth and Elevation Corrections
\ ------------------------------------------------------------------------------

VARIABLE PAA-AZ-CORRECTION
VARIABLE PAA-EL-CORRECTION

\ PAA-INIT ( -- )
: PAA-INIT ( -- )
    0 PAA-AZ-CORRECTION !
    0 PAA-EL-CORRECTION !
;

\ PAA-SET-CORRECTION ( az_q32 el_q32 -- )
: PAA-SET-CORRECTION ( az el -- )
    PAA-EL-CORRECTION !
    PAA-AZ-CORRECTION !
;
