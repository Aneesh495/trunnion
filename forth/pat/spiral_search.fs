\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/pat/spiral_search.fs
\ Description: Archimedean and hexagonal spatial spiral scanning algorithms for
\              ephemeris uncertainty cone spatial acquisition.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Uncertainty Cone Scanning:
\ Terminal scans an angular region of radius R_cone (typically 10 to 20 mrad)
\ to search for the counter-terminal's optical beacon.
\ Archimedean Spiral in polar coordinates:
\   r(phi) = (pitch / 2*pi) * phi
\ For constant linear velocity (CLV) beam sweeping:
\   phi(t) = sqrt( (4*pi * v_scan / pitch) * t )
\
\ Pitch between adjacent spiral turns:
\   pitch = 0.75 * FWHM_beam_divergence (ensures 25% overlap, zero blind spots)

VARIABLE SPIRAL-PITCH-NRAD
VARIABLE SPIRAL-MAX-RADIUS-NRAD
VARIABLE SPIRAL-SCAN-VEL-NRAD-S
VARIABLE SPIRAL-TIME-US
VARIABLE SPIRAL-ACTIVE

\ Default Spiral Configuration:
\ Max cone radius: 15.0 mrad = 15,000,000 nanoradians
15000000 CONSTANT SPIRAL-DEFAULT-MAX-RADIUS
\ Spiral pitch: 2.5 mrad = 2,500,000 nanoradians
2500000 CONSTANT SPIRAL-DEFAULT-PITCH
\ Linear scan speed: 50.0 mrad/s = 50,000,000 nanoradians/s
50000000 CONSTANT SPIRAL-DEFAULT-SCAN-SPEED

\ ------------------------------------------------------------------------------
\ Section 1: Spiral Parameter Initialization
\ ------------------------------------------------------------------------------

\ SPIRAL-INIT ( max_radius_nrad pitch_nrad scan_speed -- )
: SPIRAL-INIT ( max_rad pitch speed -- )
    SPIRAL-SCAN-VEL-NRAD-S !
    SPIRAL-PITCH-NRAD !
    SPIRAL-MAX-RADIUS-NRAD !
    0 SPIRAL-TIME-US !
    FALSE SPIRAL-ACTIVE !
;

\ SPIRAL-INIT-DEFAULT ( -- )
\ Initialize spiral search pattern with standard payload default parameters.
: SPIRAL-INIT-DEFAULT ( -- )
    SPIRAL-DEFAULT-MAX-RADIUS
    SPIRAL-DEFAULT-PITCH
    SPIRAL-DEFAULT-SCAN-SPEED
    SPIRAL-INIT
;

\ SPIRAL-START ( -- )
: SPIRAL-START ( -- )
    0 SPIRAL-TIME-US !
    TRUE SPIRAL-ACTIVE !
;

\ SPIRAL-STOP ( -- )
: SPIRAL-STOP ( -- )
    FALSE SPIRAL-ACTIVE !
;

\ ------------------------------------------------------------------------------
\ Section 2: Real-Time Scanning Trajectory Generation
\ ------------------------------------------------------------------------------

VARIABLE SPIRAL-R-NRAD
VARIABLE SPIRAL-PHI-RAD
VARIABLE SPIRAL-COS-PHI
VARIABLE SPIRAL-SIN-PHI

\ SPIRAL-STEP ( dt_us -- tip_offset_nrad tilt_offset_nrad done_flag )
\ Advance spiral pattern by dt_us and return instantaneous FSM tip/tilt offsets.
: SPIRAL-STEP ( dt -- off_x off_y done )
    SPIRAL-ACTIVE @ 0= IF
        0 0 TRUE EXIT
    THEN

    SPIRAL-TIME-US @ +
    SPIRAL-TIME-US !

    \ Time in seconds Q32.32
    SPIRAL-TIME-US @ S>Q32 1000000 S>Q32 Q32/ \ t_sec

    \ phi(t) = sqrt( (4*pi * v_scan / pitch) * t )
    \ Factor: (4*pi * v_scan / pitch)
    Q32-TWO-PI 2*
    SPIRAL-SCAN-VEL-NRAD-S @ S>Q32 Q32*
    SPIRAL-PITCH-NRAD @ S>Q32 Q32/
    SWAP Q32* \ Argument under sqrt
    CORDIC-SQRT \ phi in radians Q32.32
    DUP SPIRAL-PHI-RAD !

    \ r(phi) = (pitch / 2*pi) * phi in nanoradians
    SPIRAL-PITCH-NRAD @ S>Q32 Q32-TWO-PI Q32/
    SPIRAL-PHI-RAD @ Q32*
    Q32>S
    SPIRAL-R-NRAD !

    \ Check boundary limit
    SPIRAL-R-NRAD @ SPIRAL-MAX-RADIUS-NRAD @ >= IF
        SPIRAL-STOP
        0 0 TRUE EXIT
    THEN

    \ Compute Cartesian offsets:
    \   x = r * cos(phi)
    \   y = r * sin(phi)
    SPIRAL-PHI-RAD @ CORDIC-SIN-COS
    SPIRAL-COS-PHI ! SPIRAL-SIN-PHI !

    SPIRAL-R-NRAD @ S>Q32 SPIRAL-COS-PHI @ Q32* Q32>S
    SPIRAL-R-NRAD @ S>Q32 SPIRAL-SIN-PHI @ Q32* Q32>S
    FALSE
;
