\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_orbit.fs
\ Description: Test Suite for Astrodynamics, Kepler Propagation, LOS & Occultation
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

S" Astrodynamics, Kepler Propagator, J2 Secular Drift, LOS, and Occultation" TEST-SUITE-START

\ ------------------------------------------------------------------------------
\ Test Group 1: Keplerian Orbit Elements Setup
\ ------------------------------------------------------------------------------

S" Kepler setup: 7000 km circular LEO orbit" TEST-CASE-START
7000 S>Q32 0 S>Q32 45 S>Q32 DEG>RAD-Q32 0 S>Q32 0 S>Q32 0 S>Q32 ORB-SET-ELEMENTS
ORB-SMA-KM-Q32 @ 7000 S>Q32 100 ASSERT-Q32-CLOSE
ORB-ECC-Q32 @ 0 ASSERT-EQ
ORB-SEMI-LATUS-KM-Q32 @ 7000 S>Q32 100 ASSERT-Q32-CLOSE
ORB-MEAN-MOTION-Q32 @ 0> ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 2: Newton-Raphson Kepler Equation Solver
\ ------------------------------------------------------------------------------

S" Kepler equation solver: zero eccentricity yields E = M" TEST-CASE-START
1 S>Q32 0 S>Q32 KEPLER-SOLVE
1 S>Q32 100 ASSERT-Q32-CLOSE

S" Kepler equation solver: e=0.1 eccentric anomaly convergence" TEST-CASE-START
\ M = 0.5 rad, e = 0.1
Q32-ONE 2/ Q32-ONE 10 / KEPLER-SOLVE
\ E should satisfy E - 0.1*sin(E) = 0.5
DUP DUP CORDIC-SIN Q32-ONE 10 / Q32* -
Q32-ONE 2/ 100000 ASSERT-Q32-CLOSE
DROP

\ ------------------------------------------------------------------------------
\ Test Group 3: Orbit Propagation and Position Invariance
\ ------------------------------------------------------------------------------

S" Kepler orbit propagation: radius magnitude remains constant in circular orbit" TEST-CASE-START
7000 S>Q32 0 S>Q32 0 S>Q32 0 S>Q32 0 S>Q32 0 S>Q32 ORB-SET-ELEMENTS
0 S>Q32 ORB-PROPAGATE
ORB-GET-ECI-POS V3-NORM
7000 S>Q32 1 S>Q32 ASSERT-Q32-CLOSE

\ Propagate 60 seconds
60 S>Q32 ORB-PROPAGATE
ORB-GET-ECI-POS V3-NORM
7000 S>Q32 1 S>Q32 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 4: J2 Oblateness Secular Drift Rates
\ ------------------------------------------------------------------------------

S" J2 secular drift: 45 deg inclination produces nodal regression (dOmega/dt < 0)" TEST-CASE-START
7000 S>Q32 0 S>Q32 45 S>Q32 DEG>RAD-Q32 0 S>Q32 0 S>Q32 0 S>Q32 ORB-SET-ELEMENTS
J2-UPDATE-RATES
J2-RAAN-RATE-RAD-S-Q32 @ 0< ASSERT-TRUE

\ ------------------------------------------------------------------------------
\ Test Group 5: Inter-Satellite LOS Geometry and Range
\ ------------------------------------------------------------------------------

S" LOS geometry: Tx at (7000, 0, 0) and Rx at (7000, 3000, 0) gives range = 3000 km" TEST-CASE-START
7000 S>Q32 0 S>Q32 0 S>Q32 \ Tx
7000 S>Q32 3000 S>Q32 0 S>Q32 \ Rx
LOS-COMPUTE-RELATIVE
LOS-RANGE-KM-Q32 @ 3000 S>Q32 1000 ASSERT-Q32-CLOSE
\ Transit time: 3000 km / 299792 km/s ~ 0.010007 s (~10 ms)
LOS-TRANSIT-TIME-SEC-Q32 @
10 S>Q32 1000 S>Q32 Q32/ \ 0.010 s
50000 ASSERT-Q32-CLOSE
\ LOS unit vector along +Y: (0, 1, 0)
LOS-U-X-Q32 @ 0 S>Q32 1000 ASSERT-Q32-CLOSE
LOS-U-Y-Q32 @ Q32-ONE 1000 ASSERT-Q32-CLOSE
LOS-U-Z-Q32 @ 0 S>Q32 1000 ASSERT-Q32-CLOSE

\ ------------------------------------------------------------------------------
\ Test Group 6: Earth Limb Occultation and Solar Keep-Out
\ ------------------------------------------------------------------------------

S" Earth occultation: cross-link through Earth core is blocked" TEST-CASE-START
7000 S>Q32 0 S>Q32 0 S>Q32 \ Tx on +X side of Earth
-7000 S>Q32 0 S>Q32 0 S>Q32 \ Rx on -X side of Earth
OCC-CHECK-EARTH 1 ASSERT-EQ
OCC-EARTH-BLOCKED @ 1 ASSERT-EQ

S" Earth occultation: grazing line-of-sight above 100 km atmosphere is clear" TEST-CASE-START
7000 S>Q32 0 S>Q32 0 S>Q32 \ Tx at (7000, 0, 0)
7000 S>Q32 3000 S>Q32 0 S>Q32 \ Rx at (7000, 3000, 0)
OCC-CHECK-EARTH 0 ASSERT-EQ
OCC-EARTH-BLOCKED @ 0 ASSERT-EQ

S" Solar exclusion cone: optical line-of-sight directly toward Sun is blocked" TEST-CASE-START
\ Setup LOS along +Y axis
7000 S>Q32 0 S>Q32 0 S>Q32 7000 S>Q32 3000 S>Q32 0 S>Q32 LOS-COMPUTE-RELATIVE
\ Set Sun unit vector along +Y axis (collinear with LOS)
0 S>Q32 Q32-ONE 0 S>Q32 OCC-SET-SUN-VECTOR
OCC-CHECK-SUN 1 ASSERT-EQ

S" Solar exclusion cone: optical line-of-sight orthogonal to Sun is clear" TEST-CASE-START
\ Set Sun unit vector along +X axis (orthogonal to +Y LOS)
Q32-ONE 0 S>Q32 0 S>Q32 OCC-SET-SUN-VECTOR
OCC-CHECK-SUN 0 ASSERT-EQ
