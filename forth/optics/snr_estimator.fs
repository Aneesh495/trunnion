\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/optics/snr_estimator.fs
\ Description: Optical signal-to-noise ratio (SNR) estimator, Poisson shot
\              noise modeling, link margin calculator, and AGC policy.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Thermal noise equivalent power (NEP): 5.0 nanowatts
5 CONSTANT OPT-NEP-NW

\ Minimum optical link margin required for error-free comms: +3.0 dB
3 S>Q32 CONSTANT COMM-MIN-MARGIN-DB

VARIABLE SNR-CALC-SIG
VARIABLE SNR-CALC-BG

\ ------------------------------------------------------------------------------
\ Section 1: Optical SNR Estimation
\ ------------------------------------------------------------------------------

\ LOG10-Q32 ( x_q32 -- log10_x_q32 )
\ Compute base-10 logarithm in Q32.32 using identity: log10(x) = ln(x) / ln(10)
\ ln(10) in Q32.32 = 2.302585092994 * 2^32 = 9889524023
9889524023 CONSTANT LN-10-Q32

: LOG10-Q32 ( x -- log10_x )
    DUP 0<= IF DROP 0 EXIT THEN
    S>F FLN F>S S>Q32
    LN-10-Q32 Q32/
;

\ SNR-ESTIMATE-DB ( sig_nw bg_nw -- snr_db_q32 )
\ Compute optical SNR in decibels: 10 * log10( P_sig / (P_bg + NEP) ).
: SNR-ESTIMATE-DB ( sig bg -- snr_db )
    OPT-NEP-NW + \ Noise denominator
    DUP 0= IF 2DROP 0 EXIT THEN
    SWAP S>Q32 SWAP S>Q32 Q32/ \ Linear SNR
    LOG10-Q32
    10 S>Q32 Q32*
;

\ LINK-MARGIN-DB ( rx_power_nw min_sens_nw -- margin_db_q32 )
\ Compute optical communication link margin above detection threshold in dB.
: LINK-MARGIN-DB ( rx min_sens -- margin_db )
    DUP 0= IF 2DROP 0 EXIT THEN
    SWAP S>Q32 SWAP S>Q32 Q32/
    LOG10-Q32
    10 S>Q32 Q32*
;

\ ------------------------------------------------------------------------------
\ Section 2: Automatic Gain Control (AGC) Decision Logic
\ ------------------------------------------------------------------------------

\ AGC-RECOMMENDED-GAIN ( rx_power_nw -- gain_idx )
\ Determine optimal transimpedance gain setting based on instantaneous optical power:
\   > 2000 nW:  Gain 0 (Low, 10 kOhm)
\   200..2000:  Gain 1 (Med, 100 kOhm)
\   20..200:    Gain 2 (High, 1 MOhm)
\   < 20 nW:    Gain 3 (Ultra, 10 MOhm)
: AGC-RECOMMENDED-GAIN ( rx_power -- gain )
    DUP 2000 > IF DROP QPD-GAIN-LOW EXIT THEN
    DUP 200 > IF DROP QPD-GAIN-MED EXIT THEN
    20 > IF QPD-GAIN-HIGH ELSE QPD-GAIN-ULTRA THEN
;
