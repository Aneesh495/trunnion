\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/qpd_registers.fs
\ Description: Four-quadrant photodiode (QPD) detector ADC interface,
\              transimpedance gain selection, and optical power telemetry.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ 16-bit ADC parameters:
\ Dynamic range: 0..65535 counts
\ Saturation threshold: >= 64000 counts
\ Dark noise floor: ~150 counts
64000 CONSTANT QPD-SATURATION-THRESHOLD
150 CONSTANT QPD-DARK-NOISE-COUNTS

\ Transimpedance Gain Modes:
\ Gain 0: Low (10 kOhm)   - High optical power tracking (> 1 uW)
\ Gain 1: Med (100 kOhm)  - Nominal tracking (100 nW - 1 uW)
\ Gain 2: High (1 MOhm)   - Weak signal acquisition (10 nW - 100 nW)
\ Gain 3: Ultra (10 MOhm) - Deep space faint beacon detection (< 10 nW)
0 CONSTANT QPD-GAIN-LOW
1 CONSTANT QPD-GAIN-MED
2 CONSTANT QPD-GAIN-HIGH
3 CONSTANT QPD-GAIN-ULTRA

\ ------------------------------------------------------------------------------
\ Section 1: Raw Quadrant Acquisition
\ ------------------------------------------------------------------------------

\ QPD-READ-ALL ( -- a b c d )
\ Read instantaneous ADC counts for all four quadrants:
\   A = Top-Left,      B = Top-Right
\   C = Bottom-Left,   D = Bottom-Right
: QPD-READ-ALL ( -- a b c d )
    REG-QPD-ADC-A HW@
    REG-QPD-ADC-B HW@
    REG-QPD-ADC-C HW@
    REG-QPD-ADC-D HW@
;

\ QPD-SUM@ ( -- sum )
\ Retrieve total optical power sum = A + B + C + D.
: QPD-SUM@ ( -- sum )
    REG-QPD-SUM HW@
    DUP 0= IF
        \ If hardware register not cached, compute manually
        DROP
        QPD-READ-ALL + + +
    THEN
;

\ ------------------------------------------------------------------------------
\ Section 2: Transimpedance Amplifier Gain Control
\ ------------------------------------------------------------------------------

\ QPD-GAIN-SET ( gain_idx -- )
\ Configure transimpedance gain (0..3).
: QPD-GAIN-SET ( gain -- )
    0 3 CLAMP
    REG-QPD-GAIN-CTRL HW!
;

\ QPD-GAIN@ ( -- gain_idx )
: QPD-GAIN@ ( -- gain )
    REG-QPD-GAIN-CTRL HW@
;

\ ------------------------------------------------------------------------------
\ Section 3: Diagnostic Checks and Solar Blinding Detection
\ ------------------------------------------------------------------------------

\ QPD-STATUS@ ( -- stat )
: QPD-STATUS@ ( -- stat )
    REG-QPD-STATUS HW@
;

\ QPD-SATURATED? ( -- flag )
\ Return TRUE if any quadrant ADC exceeds the saturation limit.
: QPD-SATURATED? ( -- flag )
    QPD-STATUS@ QPD-STAT-ADC-SATURATED AND 0<>
    IF TRUE EXIT THEN
    QPD-READ-ALL
    QPD-SATURATION-THRESHOLD >= >R
    QPD-SATURATION-THRESHOLD >= R> OR >R
    QPD-SATURATION-THRESHOLD >= R> OR >R
    QPD-SATURATION-THRESHOLD >= R> OR
;

\ QPD-BEACON-DETECTED? ( -- flag )
\ Return TRUE if total sum exceeds beacon acquisition threshold.
: QPD-BEACON-DETECTED? ( -- flag )
    QPD-STATUS@ QPD-STAT-BEACON-DETECTED AND 0<>
;

\ QPD-AMBIENT@ ( -- ambient_counts )
\ Read filtered background ambient DC light estimate.
: QPD-AMBIENT@ ( -- ambient )
    REG-QPD-AMBIENT HW@
;
