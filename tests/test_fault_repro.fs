\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_fault_repro.fs
\ Description: Edge-Case Regression and Bug Reproduction Verification Suite
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Regression and Boundary Edge-Case Verification Suite" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Math Engine Singularities & Division by Zero
\ ------------------------------------------------------------------------------

S" Fixed-point division by zero safety" TEST-CASE-START
\ Q32/ with zero denominator must return 0 without trapping
100 S>Q32 0 Q32/ 0 ASSERT-EQ
-500 S>Q32 0 Q32/ 0 ASSERT-EQ
\ Q16/ with zero denominator must return 0
50 S>Q16 0 Q16/ 0 ASSERT-EQ

S" CORDIC ATAN2: negative X axis branch and quadrants" TEST-CASE-START
\ (-1, 0) -> angle = +pi
0 -1 S>Q32 CORDIC-ATAN2
Q32-PI 1000000 ASSERT-Q32-CLOSE
\ (0, -1) -> angle = -pi/2
-1 S>Q32 0 CORDIC-ATAN2
Q32-HALF-PI NEGATE 1000000 ASSERT-Q32-CLOSE
\ (0, 0) -> angle = 0
0 0 CORDIC-ATAN2 0 ASSERT-EQ

S" CORDIC SIN-COS: quadrant sign symmetry and stack convention" TEST-CASE-START
\ CORDIC-SIN-COS returns ( sin cos ) with cos on TOS
0 CORDIC-SIN-COS
\ cos(0) = 1, sin(0) = 0
Q32-ONE 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE

Q32-HALF-PI CORDIC-SIN-COS
\ cos(pi/2) = 0, sin(pi/2) = 1
0 1000000 ASSERT-Q32-CLOSE
Q32-ONE 1000000 ASSERT-Q32-CLOSE

Q32-PI CORDIC-SIN-COS
\ cos(pi) = -1, sin(pi) = 0
Q32-ONE NEGATE 1000000 ASSERT-Q32-CLOSE
0 1000000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 2: Signed Q16 Arithmetic and Negative Filtering
\ ------------------------------------------------------------------------------

S" Signed Q16: arithmetic right shift on negative products" TEST-CASE-START
\ Multiply -10.0 by +2.0 in Q16.16 -> must yield -20.0 (not positive 2^48)
-10 S>Q16 2 S>Q16 Q16*
-20 S>Q16 ASSERT-EQ

\ Convert negative Q16 to integer
-25 S>Q16 Q16>S
-25 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 3: Kinematics Shortest Arc and Wrap-Around
\ ------------------------------------------------------------------------------

S" CPA target unwrapping: +/- 180 deg boundary wrap-around" TEST-CASE-START
\ S-curve unwrapper around +/- pi
\ Target: -170 deg, Current: +170 deg: shortest path is +20 deg forward through 180
170 S>Q32 DEG>RAD-Q32
-170 S>Q32 DEG>RAD-Q32
CPA-SHORTEST-ARC
20 S>Q32 DEG>RAD-Q32 10000 ASSERT-Q32-CLOSE

\ Target: +170 deg, Current: -170 deg: shortest delta is -20 deg backward through 180
-170 S>Q32 DEG>RAD-Q32
170 S>Q32 DEG>RAD-Q32
CPA-SHORTEST-ARC
-20 S>Q32 DEG>RAD-Q32 10000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 4: Ring Buffer Full and Wrap Boundaries
\ ------------------------------------------------------------------------------

CREATE REPRO-RB 128 ALLOT

: FILL-REPRO-RING ( -- )
    7 0 DO
        I REPRO-RB RINGBUF-PUSH-CELL DROP
    LOOP
;

: DRAIN-REPRO-RING ( -- )
    7 0 DO
        REPRO-RB RINGBUF-POP-CELL
        ASSERT-TRUE
        I ASSERT-EQ
    LOOP
;

S" Ring buffer: full capacity and oldest item overwrite protection" TEST-CASE-START
REPRO-RB 8 8 RINGBUF-INIT
REPRO-RB RINGBUF-EMPTY? ASSERT-TRUE

FILL-REPRO-RING
REPRO-RB RINGBUF-FULL? ASSERT-TRUE
REPRO-RB RINGBUF-COUNT 7 ASSERT-EQ

\ 8th write should be rejected (returns false)
999 REPRO-RB RINGBUF-PUSH-CELL ASSERT-FALSE

\ Read all 7 items in FIFO order
DRAIN-REPRO-RING
REPRO-RB RINGBUF-EMPTY? ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 5: String Length and Prefix Differences
\ ------------------------------------------------------------------------------

S" String comparison: exact match, length mismatch, and prefix order" TEST-CASE-START
S" TRUNNION" S" TRUNNION" STR-CMP 0 ASSERT-EQ
S" TRUNN" S" TRUNNION" STR-CMP 0< ASSERT-TRUE
S" TRUNNION" S" TRUNN" STR-CMP 0> ASSERT-TRUE
S" ALPHA" S" BRAVO" STR-CMP 0< ASSERT-TRUE
S" BRAVO" S" ALPHA" STR-CMP 0> ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 6: CCSDS 14-Bit Sequence Counter Overflow Wrap
\ ------------------------------------------------------------------------------

S" Space packet sequence: 14-bit counter wraps from 16383 to 0" TEST-CASE-START
\ Force sequence counter to 16383 ($3FFF)
16383 APID-PAYLOAD-HEALTH CELLS SPP-TX-SEQ-COUNTERS + !
APID-PAYLOAD-HEALTH SPP-GET-NEXT-SEQ 16383 ASSERT-EQ
\ Next sequence must wrap to 0 (14-bit mask)
APID-PAYLOAD-HEALTH SPP-GET-NEXT-SEQ 0 ASSERT-EQ
APID-PAYLOAD-HEALTH SPP-GET-NEXT-SEQ 1 ASSERT-EQ

\ ------------------------------------------------------------------------------
\ Test Group 7: Fault Posting and Recovery Stability
\ ------------------------------------------------------------------------------

S" Fault manager: critical safe-hold preserves stack and locks emission" TEST-CASE-START
FAULT-INIT
SAFETY-INIT
PAT-FSM-INIT

\ Post critical watchdog stall
FAULT-WATCHDOG-STALL SEV-CRITICAL FAULT-POST
\ Verify active severity is CRITICAL
FAULT-ACTIVE-SEVERITY @ SEV-CRITICAL ASSERT-EQ
\ Verify ESTOP asserted
SAFETY-EMERGENCY-STOP @ 1 ASSERT-EQ
\ Verify PAT entered SAFE-HOLD state
PAT-CURRENT-STATE @ PAT-STATE-SAFE-HOLD ASSERT-EQ
\ Verify laser emission disabled
SAFETY-PERMIT-EMISSION? ASSERT-FALSE
