\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/orbit/constants.fs
\ Description: Astrodynamic constants, WGS-84 reference ellipsoid parameters,
\              Earth gravitational harmonics, speed of light, and optical link
\              geometry constants.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Fundamental Physical and Astrodynamical Constants
\ ------------------------------------------------------------------------------

\ Speed of light in vacuum: c = 299,792,458 m/s
299792458 CONSTANT ORB-C-M-S

\ Earth standard gravitational parameter: mu = 3.986004418e14 m^3/s^2
\ Represented in km^3/s^2 for scaled fixed-point computation: mu = 398600.4418
\ In Q32.32 format: 398600.4418 * 2^32 = 1711977793666358
1711977793666358 CONSTANT ORB-MU-KM3-S2-Q32

\ Earth equatorial radius (WGS-84): R_earth = 6,378.137 km = 6,378,137 m
6378137 CONSTANT ORB-EARTH-RADIUS-M
6378 S>Q32 CONSTANT ORB-EARTH-RADIUS-KM-Q32

\ Earth atmospheric limb grazing buffer: 100 km altitude above geoid
\ Effective optical occultation radius: R_earth + 100 km = 6,478.137 km
6478137 CONSTANT ORB-OCCULT-RADIUS-M
6478 S>Q32 CONSTANT ORB-OCCULT-RADIUS-KM-Q32

\ Earth J2 zonal harmonic oblateness coefficient: J2 = 1.08262668e-3
\ In Q32.32 format: 0.00108262668 * 2^32 = 4649866
4649866 CONSTANT ORB-J2-Q32

\ Solar Keep-Out Half-Cone Angle: 15.0 degrees to protect detector arrays
\ 15.0 deg in Q32.32 radians: (15.0 * pi / 180) * 2^32 = 1124434235
1124434235 CONSTANT ORB-SUN-EXCLUSION-RAD-Q32

\ Lunar Keep-Out Half-Cone Angle: 5.0 degrees to mitigate optical noise floor
\ 5.0 deg in Q32.32 radians: (5.0 * pi / 180) * 2^32 = 374811411
374811411 CONSTANT ORB-MOON-EXCLUSION-RAD-Q32

\ Optical carrier nominal frequency: 1550 nm (C-band)
\ Frequency f0 = c / lambda = 299792458 / 1.55e-6 = 1.934145e14 Hz (193.41 THz)
\ Wavelength in meters (Q32.32): 1.55e-6 * 2^32 = 6657
6657 CONSTANT ORB-LAMBDA-1550NM-Q32
