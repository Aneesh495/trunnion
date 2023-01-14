\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/test_harness.fs
\ Description: Lightweight Automated Unit Testing Framework for Forth
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

VARIABLE TEST-TOTAL-COUNT
VARIABLE TEST-PASS-COUNT
VARIABLE TEST-FAIL-COUNT
VARIABLE SUITE-TOTAL-COUNT
VARIABLE SUITE-FAIL-COUNT

: TEST-HARNESS-INIT ( -- )
    0 TEST-TOTAL-COUNT !
    0 TEST-PASS-COUNT !
    0 TEST-FAIL-COUNT !
    0 SUITE-TOTAL-COUNT !
    0 SUITE-FAIL-COUNT !
;

TEST-HARNESS-INIT

: TEST-SUITE-START ( c-addr u -- )
    CR ." ========================================================" CR
    ." TEST SUITE: " TYPE CR
    ." ========================================================" CR
    1 SUITE-TOTAL-COUNT +!
;

: TEST-CASE-START ( c-addr u -- )
    ."   [TEST] " TYPE ."  ... "
    1 TEST-TOTAL-COUNT +!
;

: ASSERT-TRUE ( flag -- )
    IF
        ." PASS" CR
        1 TEST-PASS-COUNT +!
    ELSE
        ." FAIL" CR
        1 TEST-FAIL-COUNT +!
        1 SUITE-FAIL-COUNT +!
    THEN
;

: ASSERT-FALSE ( flag -- )
    0= ASSERT-TRUE
;

: ASSERT-EQ ( actual expected -- )
    = ASSERT-TRUE
;

: ASSERT-NEQ ( actual expected -- )
    <> ASSERT-TRUE
;

: ASSERT-Q32-CLOSE ( actual expected tol -- )
    >R - ABS R> <= ASSERT-TRUE
;

: ASSERT-CLOSE ( actual expected tol -- )
    >R - ABS R> <= ASSERT-TRUE
;

: TEST-REPORT-SUMMARY ( -- )
    CR ." --------------------------------------------------------" CR
    ." TEST EXECUTION SUMMARY:" CR
    ."   Suites Executed: " SUITE-TOTAL-COUNT @ . CR
    ."   Total Assertions: " TEST-TOTAL-COUNT @ . CR
    ."   Passed:           " TEST-PASS-COUNT @ . CR
    ."   Failed:           " TEST-FAIL-COUNT @ . CR
    ." --------------------------------------------------------" CR
    TEST-FAIL-COUNT @ 0= IF
        ." ALL TESTS PASSED SUCCESSFULLY (100% PASS RATE)." CR
    ELSE
        ." TEST FAILURES DETECTED!" CR
    THEN
    ." --------------------------------------------------------" CR
;
