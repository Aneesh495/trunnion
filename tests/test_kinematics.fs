\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_kinematics.fs
\ Description: Test Suite for CPA/FSM Kinematics, Trajectory, and Control
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Kinematics, CPA Gimbal, FSM Actuation, S-Curve Trajectory, and LuGre Friction" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Coarse Pointing Assembly Forward Kinematics
\ ------------------------------------------------------------------------------

S" CPA forward kinematics: Az=0, El=0 -> LOS=(1, 0, 0)" TEST-CASE-START
0 S>Q32 0 S>Q32 CPA-ANGLES>LOS
\ Returns ( ux uy uz ) with uz on TOS
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ uz = 0
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ uy = 0
Q32-ONE 1000000 ASSERT-Q32-CLOSE \ ux = 1

S" CPA forward kinematics: Az=90 deg, El=0 -> LOS=(0, 1, 0)" TEST-CASE-START
Q32-HALF-PI 0 S>Q32 CPA-ANGLES>LOS
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ uz = 0
Q32-ONE 1000000 ASSERT-Q32-CLOSE \ uy = 1
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ ux = 0

S" CPA forward kinematics: Az=0, El=90 deg -> LOS=(0, 0, 1)" TEST-CASE-START
0 S>Q32 Q32-HALF-PI CPA-ANGLES>LOS
Q32-ONE 1000000 ASSERT-Q32-CLOSE \ uz = 1
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ uy = 0
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ ux = 0

\ ------------------------------------------------------------------------------
\ Test Group 2: CPA Inverse Kinematics and Keyhole Avoidance
\ ------------------------------------------------------------------------------

S" CPA inverse kinematics: LOS=(1, 0, 0) -> Az=0, El=0" TEST-CASE-START
Q32-ONE 0 S>Q32 0 S>Q32 CPA-LOS>ANGLES
\ Returns ( az el ) with el on TOS
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ el = 0
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ az = 0

S" CPA inverse kinematics: LOS=(0, 1, 0) -> Az=pi/2, El=0" TEST-CASE-START
0 S>Q32 Q32-ONE 0 S>Q32 CPA-LOS>ANGLES
0 S>Q32 1000000 ASSERT-Q32-CLOSE \ el = 0
Q32-HALF-PI 1000000 ASSERT-Q32-CLOSE \ az = pi/2

S" CPA inverse kinematics: Zenith keyhole singularity protection" TEST-CASE-START
\ Set known current azimuth resolver ticks in hardware register
45 S>Q32 DEG>RAD-Q32 CPA-RAD>TICKS REG-CPA-AZ-POS HW!
\ Request LOS pointing straight up along Z-axis: (0, 0, 1)
0 S>Q32 0 S>Q32 Q32-ONE CPA-LOS>ANGLES
\ Elevation clamps to pi/2, Azimuth preserves current angle
Q32-HALF-PI 1000000 ASSERT-Q32-CLOSE
45 S>Q32 DEG>RAD-Q32 1000000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 3: Azimuth Shortest Arc and Unwrapping
\ ------------------------------------------------------------------------------

S" CPA shortest arc: 350 deg to 10 deg gives +20 deg" TEST-CASE-START
350 S>Q32 DEG>RAD-Q32 10 S>Q32 DEG>RAD-Q32 CPA-SHORTEST-ARC
20 S>Q32 DEG>RAD-Q32 1000 ASSERT-Q32-CLOSE

S" CPA target unwrap: 350 deg to 10 deg gives 370 deg" TEST-CASE-START
350 S>Q32 DEG>RAD-Q32 10 S>Q32 DEG>RAD-Q32 CPA-TARGET-UNWRAP
370 S>Q32 DEG>RAD-Q32 1000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 4: S-Curve Trajectory Generation
\ ------------------------------------------------------------------------------

S" S-curve planner: initialize profile from 0 to 1 radian" TEST-CASE-START
0 S>Q32 Q32-ONE CPA-DEFAULT-VMAX CPA-DEFAULT-AMAX CPA-DEFAULT-JMAX CPA-TRAJ-PLAN
TRAJ-ACTIVE @ ASSERT-TRUE
TRAJ-TOTAL-TIME-US @ 0> ASSERT-TRUE

S" S-curve stepping: monotonic progress toward target" TEST-CASE-START
0 S>Q32 Q32-ONE CPA-DEFAULT-VMAX CPA-DEFAULT-AMAX CPA-DEFAULT-JMAX CPA-TRAJ-PLAN
\ Step 100 ms (100,000 microseconds)
100000 CPA-TRAJ-STEP DROP 2DROP
TRAJ-POS-CURRENT @ 0> ASSERT-TRUE
TRAJ-POS-CURRENT @ Q32-ONE < ASSERT-TRUE
TRAJ-VEL-CURRENT @ 0> ASSERT-TRUE
TRAJ-VEL-CURRENT @ CPA-DEFAULT-VMAX < ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 5: LuGre Dynamic Friction and Cogging Compensation
\ ------------------------------------------------------------------------------

S" LuGre observer reset: zero initial state and zero static torque" TEST-CASE-START
LUGRE-RESET
LUGRE-STATE-Z @ 0 ASSERT-EQ
0 S>Q32 1 S>Q32 LUGRE-STEP 0 ASSERT-EQ

S" LuGre friction step: positive velocity produces positive friction" TEST-CASE-START
LUGRE-RESET
1 S>Q32 16 LSHIFT 1 S>Q32 16 LSHIFT LUGRE-STEP \ vel = 1 rad/s, dt = 0.001 s
0> ASSERT-TRUE

S" Cogging torque feedforward compensation" TEST-CASE-START
0 S>Q32 COGGING-COMPENSATE 0 10000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 6: FSM Piezo Kinematics and Structural Filter
\ ------------------------------------------------------------------------------

S" FSM tripod kinematics round-trip: tip/tilt to piezo voltages and back" TEST-CASE-START
100000 -50000 FSM-ANGLES>PIEZO
FSM-PIEZO>ANGLES
\ Returns ( tip tilt ) with tilt on TOS
-50000 5000 ASSERT-CLOSE
100000 5000 ASSERT-CLOSE

: RUN-NOTCH-TEST ( -- )
    FSM-FILTERS-INIT
    30 0 DO
        Q32-ONE FSM-NOTCH-TIP-STEP DROP
    LOOP
    Q32-ONE FSM-NOTCH-TIP-STEP
    Q32-ONE 5000000 ASSERT-Q32-CLOSE
;

S" FSM biquad notch filter: DC unity gain verification" TEST-CASE-START
RUN-NOTCH-TEST

\ ------------------------------------------------------------------------------
\ Test Group 7: FSM Offload Controller
\ ------------------------------------------------------------------------------

S" Offload controller: quiescent state does not trigger offload" TEST-CASE-START
OFFLOAD-INIT
OFFLOAD-ACTIVE? ASSERT-FALSE
0 REG-FSM-TIP-FB HW! 0 REG-FSM-TILT-FB HW!
1 S>Q32 OFFLOAD-STEP
OFFLOAD-ACTIVE? ASSERT-FALSE

S" Offload controller: large persistent offset triggers coarse gimbal offload" TEST-CASE-START
OFFLOAD-INIT
\ Set persistent offset into accumulator exceeding threshold
OFFLOAD-THRESHOLD-NRAD 2* S>Q32 OFFLOAD-ACC-TIP !
1 S>Q32 OFFLOAD-STEP
OFFLOAD-ACTIVE? ASSERT-TRUE
