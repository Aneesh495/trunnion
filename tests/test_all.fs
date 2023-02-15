\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_all.fs
\ Description: Master Test Runner for Comprehensive Unit, System & Regression Suites
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

INCLUDE test_harness.fs

INCLUDE test_core_math.fs
INCLUDE test_core_utils.fs
INCLUDE test_kinematics.fs
INCLUDE test_optics.fs
INCLUDE test_pat.fs
INCLUDE test_orbit.fs
INCLUDE test_comm.fs
INCLUDE test_fdir.fs
INCLUDE test_kernel.fs
INCLUDE test_fault_repro.fs

TEST-REPORT-SUMMARY
