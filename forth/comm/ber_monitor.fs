\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/ber_monitor.fs
\ Description: Real-time Bit Error Rate (BER) estimator, Reed-Solomon corrected
\              symbol counter, and optical link margin monitor.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Error Counters and Link Metrics
\ ------------------------------------------------------------------------------

VARIABLE BER-TOTAL-FRAMES
VARIABLE BER-CLEAN-FRAMES
VARIABLE BER-CORRECTED-FRAMES
VARIABLE BER-UNERR-FRAMES
VARIABLE BER-TOTAL-BIT-ERRORS
VARIABLE BER-CURRENT-RATE-PPM    \ Estimated BER in parts-per-million (e.g. 10 = 1e-5)

\ Link Margin Parameters (in units of 0.1 dB)
VARIABLE BER-RX-POWER-DBM-X10   \ Received power (e.g. -350 = -35.0 dBm)
-450 CONSTANT BER-SENSITIVITY-DBM-X10 \ Receiver sensitivity threshold (-45.0 dBm)
30   CONSTANT BER-MIN-MARGIN-DB-X10   \ Minimum required link margin (3.0 dB)

\ ------------------------------------------------------------------------------
\ Section 2: Initialization
\ ------------------------------------------------------------------------------

\ BER-INIT ( -- )
: BER-INIT ( -- )
    0 BER-TOTAL-FRAMES !
    0 BER-CLEAN-FRAMES !
    0 BER-CORRECTED-FRAMES !
    0 BER-UNERR-FRAMES !
    0 BER-TOTAL-BIT-ERRORS !
    0 BER-CURRENT-RATE-PPM !
    -300 BER-RX-POWER-DBM-X10 ! \ Nominal -30.0 dBm received power
;

\ ------------------------------------------------------------------------------
\ Section 3: Periodic Frame Result Logging
\ ------------------------------------------------------------------------------

\ BER-LOG-FRAME ( decode_status error_bytes -- )
\ Update statistical error counters from Reed-Solomon decoder output.
\ decode_status: 1=Clean/Corrected, 0=Uncorrectable
: BER-LOG-FRAME ( status err_bytes -- )
    1 BER-TOTAL-FRAMES +!

    SWAP IF
        \ Successfully decoded
        DUP 0= IF
            DROP 1 BER-CLEAN-FRAMES +!
        ELSE
            1 BER-CORRECTED-FRAMES +!
            \ Approximate bit errors = 4 * symbol errors
            4 * DUP BER-TOTAL-BIT-ERRORS +!
            \ Update rolling rate: (err_bits * 1,000,000) / (255 * 8)
            1000000 * 2040 /
            BER-CURRENT-RATE-PPM !
        THEN
    ELSE
        DROP
        1 BER-UNERR-FRAMES +!
        \ Set high error rate metric for uncorrectable frame
        50000 BER-CURRENT-RATE-PPM !
    THEN
;

\ ------------------------------------------------------------------------------
\ Section 4: Optical Link Margin Computation
\ ------------------------------------------------------------------------------

\ BER-COMPUTE-MARGIN ( -- margin_db_x10 )
\ Calculate instantaneous optical link margin: M = P_rx - P_sens (in 0.1 dB).
: BER-COMPUTE-MARGIN ( -- margin )
    BER-RX-POWER-DBM-X10 @
    BER-SENSITIVITY-DBM-X10 -
;

\ BER-LINK-HEALTHY? ( -- flag )
\ Return TRUE if link margin exceeds 3.0 dB and BER is within nominal limits.
: BER-LINK-HEALTHY? ( -- flag )
    BER-COMPUTE-MARGIN BER-MIN-MARGIN-DB-X10 >=
    BER-CURRENT-RATE-PPM @ 100 <= AND
;

\ BER-PRINT-REPORT ( -- )
: BER-PRINT-REPORT ( -- )
    CR ." [OPTICAL COMMUNICATIONS LINK REPORT]" CR
    ."   Total Frames:     " BER-TOTAL-FRAMES @ . CR
    ."   Clean Frames:     " BER-CLEAN-FRAMES @ . CR
    ."   Corrected Frames: " BER-CORRECTED-FRAMES @ . CR
    ."   Uncorrectable:    " BER-UNERR-FRAMES @ . CR
    ."   Total Bit Errors: " BER-TOTAL-BIT-ERRORS @ . CR
    ."   Estimated BER:    " BER-CURRENT-RATE-PPM @ . ." ppm" CR
    ."   Link Margin:      " BER-COMPUTE-MARGIN 10 / . ." ."
    BER-COMPUTE-MARGIN 10 MOD ABS . ." dB" CR
    ."   Link Status:      " BER-LINK-HEALTHY? IF ." HEALTHY" ELSE ." DEGRADED" THEN CR
;
