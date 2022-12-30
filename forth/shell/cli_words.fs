\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/shell/cli_words.fs
\ Description: Interactive Operator Command Dictionary
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Coarse Pointing Assembly Commands
\ ------------------------------------------------------------------------------

\ CMD-SLEW ( az_q32 el_q32 -- )
\ Initiate coordinated S-curve slew profile to target gimbal angles.
: CMD-SLEW ( az el -- )
    CPA-TRAJ-PLAN
    PAT-STATE-SLEW PAT-TRANSITION
    ." [CLI] S-curve slew initiated to target angles." CR
;

\ CMD-HOME ( -- )
\ Slew CPA gimbal to safe mechanical home (stow) position.
: CMD-HOME ( -- )
    0.0 S>Q32 0.0 S>Q32 CMD-SLEW
    ." [CLI] Slewing to stow home position (0, 0)." CR
;

\ CMD-FSM ( tip_urad tilt_urad -- )
\ Directly command Fast Steering Mirror angles in microradians.
: CMD-FSM ( tip tilt -- )
    2DUP FSM-ACTUATE-ANGLES
    ." [CLI] Commanded FSM angles: Tip=" . ." uRad, Tilt=" . ." uRad." CR
;

\ CMD-FSM-ZERO ( -- )
\ Center Fast Steering Mirror to optical boresight (0, 0).
: CMD-FSM-ZERO ( -- )
    0.0 S>Q32 0.0 S>Q32 FSM-ACTUATE-ANGLES
    ." [CLI] FSM centered to optical boresight." CR
;

\ ------------------------------------------------------------------------------
\ Section 2: Laser Transmitter Commands
\ ------------------------------------------------------------------------------

\ CMD-LASER-ON ( power_mw -- )
\ Arm laser transmitter, set output power, and open safety shutter.
: CMD-LASER-ON ( power -- )
    SAFETY-PERMIT-EMISSION? IF
        LASER-ARM
        DUP LASER-SET-POWER
        LASER-SHUTTER-OPEN
        ." [CLI] Laser transmitter ARMED and EMITTING at " . ." mW." CR
    ELSE
        DROP
        ." [CLI-ERROR] Laser arming rejected by safety interlock supervisor!" CR
    THEN
;

\ CMD-LASER-OFF ( -- )
\ Close optical safety shutter, ramp down pump, and disarm laser.
: CMD-LASER-OFF ( -- )
    SAFETY-ASSERT-ESTOP
    ." [CLI] Laser transmitter SAFELY DISARMED, shutter closed." CR
;

\ ------------------------------------------------------------------------------
\ Section 3: Pointing, Acquisition, and Tracking Commands
\ ------------------------------------------------------------------------------

\ CMD-START-PAT ( -- )
\ Transition PAT state machine from Standby into Acquisition mode.
: CMD-START-PAT ( -- )
    PAT-CURRENT-STATE @ PAT-STATE-STANDBY = IF
        PAT-STATE-ACQUIRE PAT-TRANSITION
        ." [CLI] PAT sequence started. Entering ACQUIRE mode." CR
    ELSE
        ." [CLI-WARN] Terminal not in STANDBY. Current state: "
        PAT-CURRENT-STATE @ . CR
    THEN
;

\ CMD-ABORT-PAT ( -- )
\ Abort current PAT operation and transition terminal to SAFE HOLD.
: CMD-ABORT-PAT ( -- )
    PAT-ENTER-SAFE-HOLD
    ." [CLI] PAT aborted. Terminal in SAFE HOLD mode." CR
;

\ CMD-STANDBY ( -- )
\ Return terminal from safe hold or idle to STANDBY mode.
: CMD-STANDBY ( -- )
    PAT-STATE-STANDBY PAT-TRANSITION
    ." [CLI] Terminal transitioned to STANDBY mode." CR
;

\ ------------------------------------------------------------------------------
\ Section 4: Communications & Modem Commands
\ ------------------------------------------------------------------------------

\ CMD-SET-PPM ( m -- )
\ Configure pulse position modulation order (4, 16, or 64).
: CMD-SET-PPM ( m -- )
    DUP PPM-SET-ORDER
    ." [CLI] Optical modem configured for M-PPM order M=" . CR
;

\ CMD-SEND-PING ( -- )
\ Construct and queue an optical cross-link test ping packet.
: CMD-SEND-PING ( -- )
    HERE 32 ERASE
    \ Populate 16 test data bytes
    16 0 DO
        I 16 * HERE I + C!
    LOOP
    66 0 HERE 16 HERE SPP-CREATE-PACKET
    ." [CLI] Optical ping telemetry packet constructed (Len=" . ." bytes)." CR
;

\ ------------------------------------------------------------------------------
\ Section 5: Real-Time Executive & Telemetry Commands
\ ------------------------------------------------------------------------------

\ CMD-STEP ( n_ms -- )
\ Step real-time flight software executive for n milliseconds.
: CMD-STEP ( n -- )
    DUP ." [CLI] Stepping executive for " . ." ms..." CR
    EXEC-RUN
    ." [CLI] Executive step complete." CR
;

\ CMD-RESET-FAULTS ( -- )
\ Clear all latched avionics and optical faults.
: CMD-RESET-FAULTS ( -- )
    FAULT-INIT
    ." [CLI] Fault table cleared and reset to nominal." CR
;

\ CMD-STATUS ( -- )
\ Display concise single-line system status summary.
: CMD-STATUS ( -- )
    ." [STATUS] Mode: " PAT-CURRENT-STATE @ .
    ." | Laser: " LASER-EMITTING? IF ." EMITTING" ELSE ." OFF" THEN
    ." | CPA Az: " CPA-AZ-TICKS@ 1000 / . ." mdeg"
    ." | CPA El: " CPA-EL-TICKS@ 1000 / . ." mdeg"
    ." | Cycles: " EXEC-CYCLE-COUNT @ . CR
;
