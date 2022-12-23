\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/fdir/thermal_monitor.fs
\ Description: Optical bench and laser diode thermal supervisor, closed-loop
\              PID heater controller, and over-temperature safety shutdown.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Temperature Sensor Telemetry (milli-Kelvin)
\ ------------------------------------------------------------------------------

VARIABLE THERM-TEMP-LASER-MK    \ Laser diode submount temperature
VARIABLE THERM-TEMP-EDFA-MK     \ EDFA pump module temperature
VARIABLE THERM-TEMP-BENCH-MK    \ Optical bench structure temperature
VARIABLE THERM-TEMP-MOTOR-AZ-MK \ CPA Azimuth motor stator temperature
VARIABLE THERM-TEMP-MOTOR-EL-MK \ CPA Elevation motor stator temperature

\ Thresholds (milli-Kelvin)
298150 CONSTANT THERM-SETPOINT-BENCH-MK \ Target 25.0 C (298.15 K)
323150 CONSTANT THERM-MAX-LASER-MK      \ 50.0 C laser over-temp trip limit
333150 CONSTANT THERM-MAX-MOTOR-MK      \ 60.0 C motor over-temp trip limit
263150 CONSTANT THERM-MIN-BENCH-MK      \ -10.0 C minimum cold start limit

VARIABLE THERM-OVERTEMP-TRIPPED \ Boolean flag: thermal shutdown active
VARIABLE THERM-HEATER-PWM-PCT   \ Optical bench heater duty cycle (0 - 100 %)

\ ------------------------------------------------------------------------------
\ Section 2: Heater PID Controller
\ ------------------------------------------------------------------------------

VARIABLE HEATER-KP-Q16
VARIABLE HEATER-KI-Q16
VARIABLE HEATER-KD-Q16
VARIABLE HEATER-INTEGRAL-Q16
VARIABLE HEATER-PREV-ERROR-MK

\ HEATER-PID-INIT ( -- )
: HEATER-PID-INIT ( -- )
    \ Kp = 2.0, Ki = 0.05, Kd = 5.0 in Q16.16
    131072 HEATER-KP-Q16 !
    3277   HEATER-KI-Q16 !
    327680 HEATER-KD-Q16 !
    0 HEATER-INTEGRAL-Q16 !
    0 HEATER-PREV-ERROR-MK !
    0 THERM-HEATER-PWM-PCT !
    0 THERM-OVERTEMP-TRIPPED !
;

\ HEATER-PID-STEP ( measured_temp_mk setpoint_mk dt_sec -- pwm_percent )
\ Compute closed-loop heater output (0 to 100 percent duty cycle).
: HEATER-PID-STEP ( meas setpoint dt -- pwm )
    >R
    SWAP - \ Error = setpoint - measured (in mK)
    DUP >R \ Stash error

    \ Proportional term: P = Kp * err
    R@ S>Q16 HEATER-KP-Q16 @ Q16*

    \ Integral term: I = I + Ki * err * dt
    R@ S>Q16 R@ S>Q16 Q16* HEATER-KI-Q16 @ Q16*
    HEATER-INTEGRAL-Q16 @ +
    \ Clamp integral anti-windup: [-50%, +50%]
    -3276800 3276800 CLAMP
    DUP HEATER-INTEGRAL-Q16 ! +

    \ Derivative term: D = Kd * (err - prev_err) / dt
    R@ HEATER-PREV-ERROR-MK @ - S>Q16
    R> DROP \ Drop stashed error
    R> S>Q16 Q16/
    HEATER-KD-Q16 @ Q16* +

    \ Store current error
    R@ HEATER-PREV-ERROR-MK !
    R> DROP

    \ Convert Q16 to integer percent, clamped to [0, 100]
    Q16>S 0 100 CLAMP
    DUP THERM-HEATER-PWM-PCT !
;

\ ------------------------------------------------------------------------------
\ Section 3: Periodic Thermal Supervision
\ ------------------------------------------------------------------------------

\ THERM-SAMPLE ( -- )
\ Poll hardware temperature telemetry registers.
: THERM-SAMPLE ( -- )
    LASER-TEMP@ THERM-TEMP-LASER-MK !
    REG-SYS-TEMP-BOARD HW@ THERM-TEMP-BENCH-MK !
    REG-CPA-TEMP-AZ HW@ THERM-TEMP-MOTOR-AZ-MK !
    REG-CPA-TEMP-EL HW@ THERM-TEMP-MOTOR-EL-MK !
;

\ THERM-SUPERVISE ( -- overtemp_fault_flag )
\ Evaluate thermal safety margins. If over-temp detected, trip laser off.
: THERM-SUPERVISE ( -- fault )
    THERM-SAMPLE

    \ Check laser submount temperature
    THERM-TEMP-LASER-MK @ THERM-MAX-LASER-MK >= IF
        1 THERM-OVERTEMP-TRIPPED !
        0 LASER-SET-SHUTTER
        0 LASER-SET-POWER
        1 EXIT
    THEN

    \ Check motor stator temperatures
    THERM-TEMP-MOTOR-AZ-MK @ THERM-MAX-MOTOR-MK >=
    THERM-TEMP-MOTOR-EL-MK @ THERM-MAX-MOTOR-MK >= OR IF
        1 THERM-OVERTEMP-TRIPPED !
        0 0 CPA-SET-CMD-RATE
        1 EXIT
    THEN

    \ Run optical bench heater PID (1 second step)
    THERM-TEMP-BENCH-MK @ THERM-SETPOINT-BENCH-MK 1
    HEATER-PID-STEP DROP

    0 THERM-OVERTEMP-TRIPPED !
    0
;

\ THERM-PRINT-STATUS ( -- )
: THERM-PRINT-STATUS ( -- )
    CR ." [THERMAL SUBSYSTEM STATUS]" CR
    ."   Laser Temp:    " THERM-TEMP-LASER-MK @ 1000 / . ." ."
    THERM-TEMP-LASER-MK @ 1000 MOD ABS . ." K" CR
    ."   Bench Temp:    " THERM-TEMP-BENCH-MK @ 1000 / . ." ."
    THERM-TEMP-BENCH-MK @ 1000 MOD ABS . ." K" CR
    ."   Heater Power:  " THERM-HEATER-PWM-PCT @ . ." %" CR
    ."   Trip Active:   " THERM-OVERTEMP-TRIPPED @ IF ." YES (SAFETY SHUTDOWN)" ELSE ." NO" THEN CR
;
