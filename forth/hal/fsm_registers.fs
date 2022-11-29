\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/fsm_registers.fs
\ Description: Fast Steering Mirror (FSM) piezo actuator DAC drivers,
\              differential capacitive position sensors, and offload interface.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ FSM Angular Limits:
\ Mechanical range: +/- 10.0 mrad (+/- 10,000,000 nanoradians)
\ Optical beam deflection: 2x mirror tilt = +/- 20.0 mrad
10000000 CONSTANT FSM-MAX-NRAD
-10000000 CONSTANT FSM-MIN-NRAD

\ Capacitive sensor scaling: 1 count = 0.5 nanoradians
\ 16-bit DAC: 0..65535 corresponds to -100V..+100V drive
65535 CONSTANT FSM-DAC-MAX
32768 CONSTANT FSM-DAC-ZERO

\ ------------------------------------------------------------------------------
\ Section 1: Power and Status Control
\ ------------------------------------------------------------------------------

\ FSM-ENABLE ( -- )
\ Enable high-voltage piezo amplifiers and capacitive feedback conditioners.
: FSM-ENABLE ( -- )
    REG-FSM-CTRL HW@
    FSM-STAT-ENABLED OR
    REG-FSM-CTRL HW!
;

\ FSM-DISABLE ( -- )
\ Safely ramp down piezo DAC voltages and disable amplifier output.
: FSM-DISABLE ( -- )
    REG-FSM-CTRL HW@
    FSM-STAT-ENABLED INVERT AND
    REG-FSM-CTRL HW!
    0 REG-FSM-TIP-CMD HW!
    0 REG-FSM-TILT-CMD HW!
;

\ FSM-STATUS@ ( -- stat )
: FSM-STATUS@ ( -- stat )
    REG-FSM-STATUS HW@
;

\ FSM-ENABLED? ( -- flag )
: FSM-ENABLED? ( -- flag )
    FSM-STATUS@ FSM-STAT-ENABLED AND 0<>
;

\ ------------------------------------------------------------------------------
\ Section 2: Tip / Tilt Actuation Commands
\ ------------------------------------------------------------------------------

\ FSM-TIP-CMD! ( nrad -- )
\ Command fast mirror tip angle in nanoradians, clamped to safe actuator range.
: FSM-TIP-CMD! ( nrad -- )
    FSM-MIN-NRAD FSM-MAX-NRAD CLAMP
    REG-FSM-TIP-CMD HW!
;

\ FSM-TILT-CMD! ( nrad -- )
\ Command fast mirror tilt angle in nanoradians, clamped to safe actuator range.
: FSM-TILT-CMD! ( nrad -- )
    FSM-MIN-NRAD FSM-MAX-NRAD CLAMP
    REG-FSM-TILT-CMD HW!
;

\ FSM-TIP-FB@ ( -- nrad )
\ Read capacitive sensor tip angle in nanoradians.
: FSM-TIP-FB@ ( -- nrad )
    REG-FSM-TIP-FB HW@
;

\ FSM-TILT-FB@ ( -- nrad )
\ Read capacitive sensor tilt angle in nanoradians.
: FSM-TILT-FB@ ( -- nrad )
    REG-FSM-TILT-FB HW@
;

\ ------------------------------------------------------------------------------
\ Section 3: FSM DC Offloading to CPA
\ ------------------------------------------------------------------------------

\ FSM-OFFLOAD-X@ ( -- nrad )
\ Retrieve accumulated low-frequency tip error to be offloaded to CPA azimuth.
: FSM-OFFLOAD-X@ ( -- nrad )
    REG-FSM-OFFLOAD-X HW@
;

\ FSM-OFFLOAD-Y@ ( -- nrad )
\ Retrieve accumulated low-frequency tilt error to be offloaded to CPA elevation.
: FSM-OFFLOAD-Y@ ( -- nrad )
    REG-FSM-OFFLOAD-Y HW@
;

\ FSM-OFFLOAD-RESET ( -- )
\ Clear accumulated offload accumulators after gimbal handoff.
: FSM-OFFLOAD-RESET ( -- )
    0 REG-FSM-OFFLOAD-X HW!
    0 REG-FSM-OFFLOAD-Y HW!
;

\ FSM-SET-COMMAND ( tip_nrad tilt_nrad -- )
: FSM-SET-COMMAND ( tip tilt -- )
    FSM-TILT-CMD!
    FSM-TIP-CMD!
;

\ FSM-GET-DEFLECTION ( -- tip_urad tilt_urad )
: FSM-GET-DEFLECTION ( -- tip tilt )
    FSM-TIP-FB@ 1000 /
    FSM-TILT-FB@ 1000 /
;
