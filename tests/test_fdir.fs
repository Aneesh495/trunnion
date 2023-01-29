\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_fdir.fs
\ Description: Test Suite for Fault Detection, Isolation, Recovery (FDIR) and Safety
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Fault Detection, Isolation, Recovery (FDIR), Thermal PID, and Safety Interlocks" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Hardware Watchdog Supervisor & Task Heartbeats
\ ------------------------------------------------------------------------------

S" Watchdog supervisor: initialization and nominal heartbeat cycle" TEST-CASE-START
WD-INIT
WD-TIMEOUT-MS @ 1000 ASSERT-EQ
WD-ELAPSED-MS @ 0 ASSERT-EQ
WD-STALL-COUNT @ 0 ASSERT-EQ

\ Assert heartbeats for all 4 tasks
TASK-BIT-SERVO WD-HEARTBEAT
TASK-BIT-PAT WD-HEARTBEAT
TASK-BIT-COMM WD-HEARTBEAT
TASK-BIT-TELEMETRY WD-HEARTBEAT

\ Step supervisor by 10 ms: should detect all heartbeats and return 0 (Normal)
10 WD-SUPERVISE-STEP 0 ASSERT-EQ
\ Elapsed time reset to 0
WD-ELAPSED-MS @ 0 ASSERT-EQ
WD-STALL-COUNT @ 0 ASSERT-EQ

S" Watchdog supervisor: stall detection on missed task heartbeat" TEST-CASE-START
WD-INIT
\ Heartbeat only from servo task
TASK-BIT-SERVO WD-HEARTBEAT

\ Advance by 500 ms: status should be 1 (Pending)
500 WD-SUPERVISE-STEP 1 ASSERT-EQ
WD-ELAPSED-MS @ 500 ASSERT-EQ
WD-STALL-COUNT @ 0 ASSERT-EQ

\ Advance another 500 ms (total 1000 ms >= timeout): status should be 2 (Stall)
500 WD-SUPERVISE-STEP 2 ASSERT-EQ
WD-STALL-COUNT @ 1 ASSERT-EQ
WD-ELAPSED-MS @ 0 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 2: Optical Bench Thermal PID Controller
\ ------------------------------------------------------------------------------

S" Thermal PID: full heating response when bench is cold" TEST-CASE-START
HEATER-PID-INIT
\ Setpoint = 298150 mK (25 C), Measured = 273150 mK (0 C, delta = +25000 mK), dt = 1 sec
273150 298150 1 HEATER-PID-STEP
\ Should command high PWM duty cycle (clamped to 100%)
100 ASSERT-EQ
THERM-HEATER-PWM-PCT @ 100 ASSERT-EQ

S" Thermal PID: zero heating response when bench is above setpoint" TEST-CASE-START
HEATER-PID-INIT
\ Setpoint = 298150 mK, Measured = 305000 mK (delta = -6850 mK), dt = 1 sec
305000 298150 1 HEATER-PID-STEP
\ Should clamp output to 0%
0 ASSERT-EQ
THERM-HEATER-PWM-PCT @ 0 ASSERT-EQ

S" Thermal supervisor: over-temperature triggers safety shutdown" TEST-CASE-START
HEATER-PID-INIT
SAFETY-INIT
\ Open shutter and set power
1 LASER-SET-SHUTTER
100 LASER-SET-POWER

\ Inject laser temperature above trip limit (55 C = 328150 mK >= 323150 mK)
328150 REG-LASER-TEMP HW!
THERM-SUPERVISE 1 ASSERT-EQ
THERM-OVERTEMP-TRIPPED @ 1 ASSERT-EQ
\ Verify laser power and shutter were commanded to zero
REG-LASER-SHUTTER HW@ 0 ASSERT-EQ
REG-LASER-TX-POWER HW@ 0 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 3: Optical Emission Safety Interlocks
\ ------------------------------------------------------------------------------

S" Optical safety: emission permission checks" TEST-CASE-START
SAFETY-INIT
LASER-STAT-INTERLOCK-CLOSED REG-LASER-STATUS HW! \ Hardware interlock closed
SAFETY-PERMIT-EMISSION? ASSERT-TRUE

S" Optical safety: sun sensor blinding triggers ESTOP" TEST-CASE-START
SAFETY-INIT
LASER-STAT-INTERLOCK-CLOSED REG-LASER-STATUS HW!
\ Inject sun sensor overload on QPD ambient register (> 50000 counts)
60000 REG-QPD-AMBIENT HW!
SAFETY-CHECK-SUN-BLIND 1 ASSERT-EQ
SAFETY-SUN-BLIND-TRIP @ 1 ASSERT-EQ
SAFETY-EMERGENCY-STOP @ 1 ASSERT-EQ
SAFETY-PERMIT-EMISSION? ASSERT-FALSE

S" Optical safety: optical leakage detection with shutter closed" TEST-CASE-START
SAFETY-INIT
0 REG-LASER-SHUTTER HW! \ Shutter closed
\ Leakage power exceeds 10 mW threshold
15 REG-LASER-TX-POWER HW!
SAFETY-CHECK-LEAKAGE 1 ASSERT-EQ
SAFETY-LEAKAGE-FAULT @ 1 ASSERT-EQ
SAFETY-EMERGENCY-STOP @ 1 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 4: Master Fault Manager and Circular Log
\ ------------------------------------------------------------------------------

S" Fault manager: log posting, count accumulation, and severity tracking" TEST-CASE-START
FAULT-INIT
FAULT-LOG-COUNT @ 0 ASSERT-EQ
FAULT-ACTIVE-SEVERITY @ SEV-INFO ASSERT-EQ

\ Post a warning
FAULT-FRAME-SYNC-LOST SEV-WARN FAULT-POST
FAULT-LOG-COUNT @ 1 ASSERT-EQ
FAULT-ACTIVE-SEVERITY @ SEV-WARN ASSERT-EQ

\ Post an error: severity should elevate to ERROR
FAULT-MOTOR-OVERTEMP SEV-ERROR FAULT-POST
FAULT-LOG-COUNT @ 2 ASSERT-EQ
FAULT-ACTIVE-SEVERITY @ SEV-ERROR ASSERT-EQ

\ Clear active severity
FAULT-CLEAR
FAULT-ACTIVE-SEVERITY @ SEV-INFO ASSERT-EQ
\ Count remains 2
FAULT-LOG-COUNT @ 2 ASSERT-EQ

S" Fault manager: critical fault triggers ESTOP and PAT safe-hold" TEST-CASE-START
FAULT-INIT
SAFETY-INIT
PAT-FSM-INIT
\ Move PAT to ACQUIRE state
PAT-STATE-ACQUIRE PAT-CURRENT-STATE !
PAT-CURRENT-STATE @ PAT-STATE-ACQUIRE ASSERT-EQ

\ Post a CRITICAL fault
FAULT-LASER-OVERTEMP SEV-CRITICAL FAULT-POST
FAULT-ACTIVE-SEVERITY @ SEV-CRITICAL ASSERT-EQ
\ Verify ESTOP asserted
SAFETY-EMERGENCY-STOP @ 1 ASSERT-EQ
\ Verify PAT entered SAFE-HOLD state
PAT-CURRENT-STATE @ PAT-STATE-SAFE-HOLD ASSERT-EQ
