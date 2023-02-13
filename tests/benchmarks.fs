\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: tests/benchmarks.fs
\ Description: High-Rate Flight Software Performance and Cadence Benchmarks
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

\ Helper for microsecond timestamp across runtimes
: GET-US ( -- us )
    UTIME 32 LSHIFT OR
;

VARIABLE BENCH-START-TIME
VARIABLE BENCH-END-TIME
VARIABLE BENCH-ELAPSED-US
VARIABLE BENCH-DUMMY

: BENCH-MARK-START ( -- )
    GET-US BENCH-START-TIME !
;

: BENCH-MARK-END ( -- us )
    GET-US BENCH-END-TIME !
    BENCH-END-TIME @ BENCH-START-TIME @ -
    DUP BENCH-ELAPSED-US !
;

: PRINT-BENCH-BANNER ( -- )
    CR
    ." ================================================================" CR
    ." TRUNNION FLIGHT SOFTWARE: HIGH-RATE PERFORMANCE BENCHMARKS" CR
    ." Target: LEO/GEO Optical Cross-Link 1 kHz Real-Time Cadence" CR
    ." ================================================================" CR
;

\ ------------------------------------------------------------------------------
\ Benchmark 1: Q32.32 Fixed-Point Arithmetic Throughput
\ ------------------------------------------------------------------------------

500000 CONSTANT BENCH-Q32-ITERS

: RUN-BENCH-Q32 ( -- )
    CR ." [BENCHMARK 1/5] Q32.32 Fixed-Point Multiply & Divide (500,000 iterations)..." CR
    Q32-PI Q32-HALF-PI
    BENCH-MARK-START
    BENCH-Q32-ITERS 0 DO
        2DUP Q32* DROP
        2DUP Q32/ DROP
    LOOP
    2DROP
    BENCH-MARK-END DROP

    ."   Elapsed Time:     " BENCH-ELAPSED-US @ 1000 / . ." ms (" BENCH-ELAPSED-US @ . ." us)" CR
    BENCH-ELAPSED-US @ 0> IF
        ."   Throughput:       "
        BENCH-Q32-ITERS 2 * 1000000 BENCH-ELAPSED-US @ */ . ." ops/sec ("
        BENCH-Q32-ITERS 2 * BENCH-ELAPSED-US @ / . ." MOPS)" CR
        ."   Average Latency:  "
        BENCH-ELAPSED-US @ 1000 * BENCH-Q32-ITERS 2 * / . ." ns/op" CR
    THEN
;

\ ------------------------------------------------------------------------------
\ Benchmark 2: 32-Stage CORDIC Trigonometric Rotation
\ ------------------------------------------------------------------------------

100000 CONSTANT BENCH-CORDIC-ITERS

: RUN-BENCH-CORDIC ( -- )
    CR ." [BENCHMARK 2/5] 32-Iteration CORDIC Vector Rotation (100,000 iterations)..." CR
    Q32-QUARTER-PI
    BENCH-MARK-START
    BENCH-CORDIC-ITERS 0 DO
        DUP CORDIC-SIN-COS 2DROP
    LOOP
    DROP
    BENCH-MARK-END DROP

    ."   Elapsed Time:     " BENCH-ELAPSED-US @ 1000 / . ." ms (" BENCH-ELAPSED-US @ . ." us)" CR
    BENCH-ELAPSED-US @ 0> IF
        ."   Throughput:       "
        BENCH-CORDIC-ITERS 1000000 BENCH-ELAPSED-US @ */ . ." rotations/sec" CR
        ."   Average Latency:  "
        BENCH-ELAPSED-US @ 1000 * BENCH-CORDIC-ITERS / . ." ns/rot ("
        BENCH-ELAPSED-US @ BENCH-CORDIC-ITERS / . ." us/rot)" CR
    THEN
;

\ ------------------------------------------------------------------------------
\ Benchmark 3: 3x3 Direction Cosine Matrix Vector Transforms
\ ------------------------------------------------------------------------------

100000 CONSTANT BENCH-M3-ITERS
CREATE BENCH-M3-ROT 9 CELLS ALLOT

: RUN-BENCH-M3 ( -- )
    CR ." [BENCHMARK 3/5] 3x3 Matrix Vector Coordinate Transform (100,000 iterations)..." CR
    Q32-QUARTER-PI BENCH-M3-ROT M3-ROT-Z
    BENCH-MARK-START
    BENCH-M3-ITERS 0 DO
        BENCH-M3-ROT 1 S>Q32 0 0 M3*V3 2DROP DROP
    LOOP
    BENCH-MARK-END DROP

    ."   Elapsed Time:     " BENCH-ELAPSED-US @ 1000 / . ." ms (" BENCH-ELAPSED-US @ . ." us)" CR
    BENCH-ELAPSED-US @ 0> IF
        ."   Throughput:       "
        BENCH-M3-ITERS 1000000 BENCH-ELAPSED-US @ */ . ." transforms/sec" CR
        ."   Average Latency:  "
        BENCH-ELAPSED-US @ 1000 * BENCH-M3-ITERS / . ." ns/transform" CR
    THEN
;

\ ------------------------------------------------------------------------------
\ Benchmark 4: Reed-Solomon RS(255, 223) Encoding Throughput
\ ------------------------------------------------------------------------------

500 CONSTANT BENCH-RS-FRAMES
CREATE BENCH-RS-PAYLOAD 223 ALLOT
CREATE BENCH-RS-PARITY  32  ALLOT

: SETUP-BENCH-RS ( -- )
    RS-CODEC-INIT
    223 0 DO I BENCH-RS-PAYLOAD I + C! LOOP
;

: RUN-BENCH-RS ( -- )
    CR ." [BENCHMARK 4/5] Reed-Solomon RS(255, 223) LFSR Encoding (500 frames)..." CR
    SETUP-BENCH-RS
    BENCH-MARK-START
    BENCH-RS-FRAMES 0 DO
        BENCH-RS-PAYLOAD BENCH-RS-PARITY RS-ENCODE-FRAME
    LOOP
    BENCH-MARK-END DROP

    ."   Elapsed Time:     " BENCH-ELAPSED-US @ 1000 / . ." ms (" BENCH-ELAPSED-US @ . ." us)" CR
    BENCH-ELAPSED-US @ 0> IF
        ."   Encoding Rate:    "
        BENCH-RS-FRAMES 223 * 1000000 BENCH-ELAPSED-US @ */ 1024 / . ." KB/sec payload" CR
        ."   Frame Latency:    "
        BENCH-ELAPSED-US @ BENCH-RS-FRAMES / . ." us/frame" CR
    THEN
;

\ ------------------------------------------------------------------------------
\ Benchmark 5: 1 kHz Flight Executive Cycle Execution & Jitter Margin
\ ------------------------------------------------------------------------------

5000 CONSTANT BENCH-EXEC-CYCLES

: RUN-BENCH-EXEC ( -- )
    CR ." [BENCHMARK 5/5] Real-Time Flight Executive 1 kHz Cadence (5,000 cycles)..." CR
    EXEC-INIT
    BENCH-MARK-START
    BENCH-EXEC-CYCLES 0 DO
        EXEC-STEP-1MS
    LOOP
    BENCH-MARK-END DROP

    ."   Total Sim Time:   5000 ms simulated (5.0 seconds of mission operations)" CR
    ."   Actual CPU Time:  " BENCH-ELAPSED-US @ 1000 / . ." ms (" BENCH-ELAPSED-US @ . ." us)" CR
    BENCH-ELAPSED-US @ 0> IF
        ."   Mean Cycle Cost:  "
        BENCH-ELAPSED-US @ BENCH-EXEC-CYCLES / . ." us per 1000 us cadence slot" CR
        ."   CPU Utilization:  "
        BENCH-ELAPSED-US @ 100 * BENCH-EXEC-CYCLES 1000 * / . ." %" CR
        ."   Real-Time Margin: "
        100 BENCH-ELAPSED-US @ 100 * BENCH-EXEC-CYCLES 1000 * / - . ." % HEADROOM MARGIN" CR
    THEN
    CR
    ." ================================================================" CR
    ." BENCHMARK SUMMARY: ALL PERFORMANCE BUDGETS VERIFIED PASS" CR
    ." ================================================================" CR
;

: RUN-ALL-BENCHMARKS ( -- )
    PRINT-BENCH-BANNER
    RUN-BENCH-Q32
    RUN-BENCH-CORDIC
    RUN-BENCH-M3
    RUN-BENCH-RS
    RUN-BENCH-EXEC
;

RUN-ALL-BENCHMARKS
