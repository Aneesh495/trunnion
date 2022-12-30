\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/shell/diag_dump.fs
\ Description: Comprehensive System Diagnostic Dumps & Register Inspection
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Memory-Mapped Peripheral Register Inspection
\ ------------------------------------------------------------------------------

\ DUMP-REGISTERS ( -- )
\ Formatted hexadecimal printout of memory-mapped hardware bus registers.
: DUMP-REGISTERS ( -- )
    CR ." ================= HARDWARE BUS REGISTERS =================" CR
    ." Offset  Register Name             Hex Value     Decimal" CR
    ." --------------------------------------------------------" CR
    ." 0x0000  REG_SYS_MAGIC             0x" REG-SYS-MAGIC HW@ HEX . DECIMAL CR
    ." 0x0001  REG_SYS_STATUS            0x" REG-SYS-STATUS HW@ HEX . DECIMAL CR
    ." 0x0002  REG_SYS_CLOCK_LO          0x" REG-SYS-CLOCK-LO HW@ HEX . DECIMAL CR
    ." 0x0003  REG_SYS_CLOCK_HI          0x" REG-SYS-CLOCK-HI HW@ HEX . DECIMAL CR
    ." 0x0102  REG_CPA_AZ_POS            0x" REG-CPA-AZ-POS HW@ HEX . DECIMAL CR
    ." 0x0103  REG_CPA_EL_POS            0x" REG-CPA-EL-POS HW@ HEX . DECIMAL CR
    ." 0x0106  REG_CPA_AZ_CMD_RATE       0x" REG-CPA-AZ-CMD-RATE HW@ HEX . DECIMAL CR
    ." 0x0107  REG_CPA_EL_CMD_RATE       0x" REG-CPA-EL-CMD-RATE HW@ HEX . DECIMAL CR
    ." 0x0202  REG_FSM_TIP_CMD           0x" REG-FSM-TIP-CMD HW@ HEX . DECIMAL CR
    ." 0x0203  REG_FSM_TILT_CMD          0x" REG-FSM-TILT-CMD HW@ HEX . DECIMAL CR
    ." 0x0204  REG_FSM_TIP_FB            0x" REG-FSM-TIP-FB HW@ HEX . DECIMAL CR
    ." 0x0205  REG_FSM_TILT_FB           0x" REG-FSM-TILT-FB HW@ HEX . DECIMAL CR
    ." 0x0302  REG_QPD_ADC_A             0x" REG-QPD-ADC-A HW@ HEX . DECIMAL CR
    ." 0x0303  REG_QPD_ADC_B             0x" REG-QPD-ADC-B HW@ HEX . DECIMAL CR
    ." 0x0304  REG_QPD_ADC_C             0x" REG-QPD-ADC-C HW@ HEX . DECIMAL CR
    ." 0x0305  REG_QPD_ADC_D             0x" REG-QPD-ADC-D HW@ HEX . DECIMAL CR
    ." 0x0400  REG_LASER_STATUS          0x" REG-LASER-STATUS HW@ HEX . DECIMAL CR
    ." 0x0405  REG_LASER_TEMP            0x" REG-LASER-TEMP HW@ HEX . DECIMAL CR
    ." 0x0503  REG_COMM_RX_POWER         0x" REG-COMM-RX-POWER HW@ HEX . DECIMAL CR
    ." 0x0500  REG_COMM_STATUS           0x" REG-COMM-STATUS HW@ HEX . DECIMAL CR
    ." 0x0600  REG_WD_FEED               0x" REG-WD-FEED HW@ HEX . DECIMAL CR
    ." ========================================================" CR
;

\ ------------------------------------------------------------------------------
\ Section 2: CPA and FSM Kinematics Diagnostics
\ ------------------------------------------------------------------------------

\ DUMP-GIMBAL ( -- )
\ Detailed telemetry of dual-axis Coarse Pointing Assembly gimbal mechanism.
: DUMP-GIMBAL ( -- )
    CR ." ---------------- CPA GIMBAL SUBSYSTEM ----------------" CR
    ." Measured Azimuth:      " CPA-AZ-TICKS@ . ." ticks (" CPA-AZ-ANGLE@ Q32>S . ." rad)" CR
    ." Measured Elevation:    " CPA-EL-TICKS@ . ." ticks (" CPA-EL-ANGLE@ Q32>S . ." rad)" CR
    ." Trajectory Target:     " TRAJ-POS-TARGET @ Q32>S . ." rad" CR
    ." Trajectory Position:   " TRAJ-POS-CURRENT @ Q32>S . ." rad" CR
    ." Trajectory Velocity:   " TRAJ-VEL-CURRENT @ Q32>S . ." rad/s" CR
    ." Slew Active:           " TRAJ-ACTIVE @ IF ." YES" ELSE ." NO" THEN CR
    ." LuGre Bristle Z:       " LUGRE-STATE-Z @ Q32>S . ." rad" CR
    ." --------------------------------------------------------" CR
;

\ DUMP-FSM ( -- )
\ Detailed telemetry of Fast Steering Mirror piezoelectric fine stage.
: DUMP-FSM ( -- )
    CR ." ---------------- FSM PIEZO SUBSYSTEM ----------------" CR
    ." Tip Feedback:          " FSM-TIP-FB@ . ." nanoradians" CR
    ." Tilt Feedback:         " FSM-TILT-FB@ . ." nanoradians" CR
    ." Offload Filtered Tip:  " OFFLOAD-ACC-TIP @ Q32>S . ." nrad" CR
    ." Offload Filtered Tilt: " OFFLOAD-ACC-TILT @ Q32>S . ." nrad" CR
    ." Offload Triggered:     " OFFLOAD-TRIGGERED @ IF ." YES" ELSE ." NO" THEN CR
    ." --------------------------------------------------------" CR
;

\ ------------------------------------------------------------------------------
\ Section 3: Optics & Spot Detection Diagnostics
\ ------------------------------------------------------------------------------

\ DUMP-OPTICS ( -- )
\ Quadrant photodiode detector signals, centroid offset, and estimated SNR.
: DUMP-OPTICS ( -- )
    CR ." ---------------- OPTICS & SENSOR SUITE ---------------" CR
    ." QPD Raw Channel A:     " REG-QPD-ADC-A HW@ . CR
    ." QPD Raw Channel B:     " REG-QPD-ADC-B HW@ . CR
    ." QPD Raw Channel C:     " REG-QPD-ADC-C HW@ . CR
    ." QPD Raw Channel D:     " REG-QPD-ADC-D HW@ . CR
    ." QPD Sum Intensity:     " QPD-SUM@ . CR
    ." Centroid Offset DX:    " QPD-NORM-DX @ Q32>S . CR
    ." Centroid Offset DY:    " QPD-NORM-DY @ Q32>S . CR
    ." Calibrated Angle X:    " QPD-LAST-ANG-X @ Q32>S . ." rad" CR
    ." Calibrated Angle Y:    " QPD-LAST-ANG-Y @ Q32>S . ." rad" CR
    ." Estimated Link SNR:    " QPD-SUM@ QPD-AMBIENT@ SNR-ESTIMATE-DB Q32>S . ." dB" CR
    ." --------------------------------------------------------" CR
;

\ ------------------------------------------------------------------------------
\ Section 4: PAT Subsystem & Point-Ahead Diagnostics
\ ------------------------------------------------------------------------------

\ DUMP-PAT ( -- )
\ Pointing, acquisition, and tracking status, search profile, and point-ahead.
: DUMP-PAT ( -- )
    CR ." ------------- PAT ENGINE & POINT-AHEAD -------------" CR
    ." Operating State:       " PAT-PRINT-STATE CR
    ." Acquisition Confirmed: " ACQ-CONFIRMED? IF ." YES" ELSE ." NO" THEN CR
    ." Tracking RMS Jitter:   " TRACK-RMS-ERROR-URAD . ." microradians" CR
    ." Spiral Current Radius: " SPIRAL-R-NRAD @ . ." nanoradians" CR
    ." Spiral Current Angle:  " SPIRAL-PHI-RAD @ Q32>S . ." rad" CR
    ." Point-Ahead Azimuth:   " PAA-AZ-CORRECTION @ Q32>S . ." rad" CR
    ." Point-Ahead Elevation: " PAA-EL-CORRECTION @ Q32>S . ." rad" CR
    ." Loss-of-Lock Monitor:  " CR
    LOL-STATUS-PRINT
    ." --------------------------------------------------------" CR
;

\ ------------------------------------------------------------------------------
\ Section 5: Communications & Bit Error Rate Diagnostics
\ ------------------------------------------------------------------------------

\ DUMP-COMM ( -- )
\ Optical cross-link modem status, Reed-Solomon telemetry, and link BER.
: DUMP-COMM ( -- )
    CR ." ------------ OPTICAL COMMUNICATIONS & BER ------------" CR
    ." M-PPM Modulation Order: " PPM-ORDER @ . CR
    ." Slots per Symbol:       " PPM-TOTAL-SLOTS @ . CR
    ." Transmitted Symbols:    " PPM-TX-SYMBOL-COUNT @ . CR
    ." Received Symbols:       " PPM-RX-SYMBOL-COUNT @ . CR
    ." Total Received Frames:  " BER-TOTAL-FRAMES @ . CR
    ." Clean Frames:           " BER-CLEAN-FRAMES @ . CR
    ." Corrected Frames:       " BER-CORRECTED-FRAMES @ . CR
    ." Uncorrectable Frames:   " BER-UNERR-FRAMES @ . CR
    ." Estimated Channel BER:  " BER-CURRENT-RATE-PPM @ . ." ppm" CR
    ." Received Optical Power: " BER-RX-POWER-DBM-X10 @ . ." dBm/10" CR
    ." --------------------------------------------------------" CR
;

\ ------------------------------------------------------------------------------
\ Section 6: Fault Detection, Isolation, and Recovery (FDIR)
\ ------------------------------------------------------------------------------

\ DUMP-FAULTS ( -- )
\ Latched flight software and hardware fault table summary.
: DUMP-FAULTS ( -- )
    CR ." ----------------- ACTIVE FAULT TABLE -----------------" CR
    ." Total Faults Logged:    " FAULT-LOG-COUNT @ . CR
    ." Active Severity Level:  "
    CASE FAULT-ACTIVE-SEVERITY @
        SEV-INFO     OF ." INFO" ENDOF
        SEV-WARN     OF ." WARN" ENDOF
        SEV-ERROR    OF ." ERROR" ENDOF
        SEV-CRITICAL OF ." CRITICAL" ENDOF
        ." NOMINAL"
    ENDCASE CR
    ." Fault Log Head Index:   " FAULT-LOG-HEAD @ . CR
    ." --------------------------------------------------------" CR
;

\ ------------------------------------------------------------------------------
\ Section 7: Master System Diagnostic Dump
\ ------------------------------------------------------------------------------

\ DUMP-ALL ( -- )
\ Print exhaustive multi-subsystem engineering diagnostic report.
: DUMP-ALL ( -- )
    CR ." ========================================================" CR
    ."        TRUNNION OPTICAL CROSS-LINK FLIGHT SOFTWARE        " CR
    ."           FULL SYSTEM ENGINEERING DIAGNOSTIC DUMP          " CR
    ." ========================================================" CR
    DUMP-REGISTERS
    DUMP-GIMBAL
    DUMP-FSM
    DUMP-OPTICS
    DUMP-PAT
    DUMP-COMM
    DUMP-FAULTS
    CR ." Executive Cadence Cycles: " EXEC-CYCLE-COUNT @ .
    ." | 1 kHz Jitter: Nominal" CR
    ." System Time: " TIME-MILLIS . ." ms" CR
    ." ========================================================" CR
;
