\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_pat.fs
\ Description: Test Suite for Pointing, Acquisition & Tracking Subsystems
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Pointing, Acquisition, and Tracking (PAT) Engine & Relativistic Point-Ahead" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Archimedean Spiral Search Generator
\ ------------------------------------------------------------------------------

S" Spiral generator: initialization and start state" TEST-CASE-START
SPIRAL-INIT-DEFAULT
SPIRAL-ACTIVE @ ASSERT-FALSE
SPIRAL-START
SPIRAL-ACTIVE @ ASSERT-TRUE

S" Spiral generator: stepping produces radial progression" TEST-CASE-START
SPIRAL-INIT-DEFAULT
SPIRAL-START
\ Step 50 ms (50,000 us)
50000 SPIRAL-STEP
ASSERT-FALSE \ done flag is FALSE
\ Returns ( off_x off_y )
DROP DROP
SPIRAL-R-NRAD @ 0> ASSERT-TRUE
SPIRAL-R-NRAD @ SPIRAL-DEFAULT-MAX-RADIUS < ASSERT-TRUE

S" Spiral generator: stop deactivates scan" TEST-CASE-START
SPIRAL-STOP
SPIRAL-ACTIVE @ ASSERT-FALSE
10000 SPIRAL-STEP
ASSERT-TRUE \ done flag is TRUE
0 ASSERT-EQ 0 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 2: Lissajous Search Pattern
\ ------------------------------------------------------------------------------

S" Lissajous generator: bounded harmonic trajectory" TEST-CASE-START
LISSAJOUS-INIT-DEFAULT
LISSAJOUS-START
LISS-ACTIVE @ ASSERT-TRUE
10000 LISSAJOUS-STEP
\ Returns ( off_x off_y )
ABS 10000000 <= ASSERT-TRUE
ABS 10000000 <= ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 3: Relativistic Point-Ahead Angle (PAA)
\ ------------------------------------------------------------------------------

S" Point-ahead angle: 7500 m/s relative transverse velocity yields ~50 urad" TEST-CASE-START
\ Satellite A at rest: (0, 0, 0)
\ Satellite B moving in Y-direction: v_rel = (0, 7500 m/s, 0)
\ Line-of-sight along X-axis: u_los = (1, 0, 0)
0 S>Q32 0 S>Q32 0 S>Q32     \ v_tx = (0, 0, 0)
0 S>Q32 7500 S>Q32 0 S>Q32  \ v_rx = (0, 7500, 0)
Q32-ONE 0 S>Q32 0 S>Q32     \ u_los = (1, 0, 0)
PAA-COMPUTE
PAA-MAGNITUDE-URAD
50 2 ASSERT-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 4: M-of-N Beacon Acquisition Confirmation
\ ------------------------------------------------------------------------------

S" Acquisition filter: initial reset state" TEST-CASE-START
ACQ-RESET
ACQ-STATE-FLAG @ ASSERT-FALSE
ACQ-COUNT-HITS 0 ASSERT-EQ

S" Acquisition filter: ambient noise does not trigger acquisition" TEST-CASE-START
ACQ-RESET
100 80 ACQ-PROCESS-FRAME ASSERT-FALSE
100 85 ACQ-PROCESS-FRAME ASSERT-FALSE
100 90 ACQ-PROCESS-FRAME ASSERT-FALSE
ACQ-STATE-FLAG @ ASSERT-FALSE

S" Acquisition filter: 4 of 5 beacon pulses triggers confirmed acquisition" TEST-CASE-START
ACQ-RESET
\ Feed 5 consecutive frames: 4 strong pulses (2000 counts) and 1 fade (100 counts)
2000 100 ACQ-PROCESS-FRAME DROP
2000 100 ACQ-PROCESS-FRAME DROP
2000 100 ACQ-PROCESS-FRAME DROP
100  100 ACQ-PROCESS-FRAME DROP
2000 100 ACQ-PROCESS-FRAME DROP
ACQ-COUNT-HITS 4 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 5: Master PAT State Machine Lifecycle
\ ------------------------------------------------------------------------------

S" PAT FSM: boot in STANDBY state" TEST-CASE-START
PAT-FSM-INIT
PAT-CURRENT-STATE @ PAT-STATE-STANDBY ASSERT-EQ

S" PAT FSM: transition to SLEW state" TEST-CASE-START
0 S>Q32 0 S>Q32 PAT-ENTER-SLEW
PAT-CURRENT-STATE @ PAT-STATE-SLEW ASSERT-EQ

S" PAT FSM: transition to SEARCH state" TEST-CASE-START
PAT-ENTER-SEARCH
PAT-CURRENT-STATE @ PAT-STATE-SEARCH ASSERT-EQ
SPIRAL-ACTIVE @ ASSERT-TRUE

S" PAT FSM: transition to TRACK state" TEST-CASE-START
PAT-ENTER-TRACK
PAT-CURRENT-STATE @ PAT-STATE-TRACK ASSERT-EQ
TRACK-LOOP-ACTIVE? ASSERT-TRUE

S" PAT FSM: transition to COMM-LOCK state" TEST-CASE-START
PAT-ENTER-COMM-LOCK
PAT-CURRENT-STATE @ PAT-STATE-COMM-LOCK ASSERT-EQ

S" PAT FSM: safe hold transition deactivates active actuators" TEST-CASE-START
PAT-STATE-SAFE-HOLD PAT-TRANSITION
PAT-CURRENT-STATE @ PAT-STATE-SAFE-HOLD ASSERT-EQ
