\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/kernel/telemetry.fs
\ Description: Flight software binary engineering telemetry packet formatter,
\              housekeeping serializer, and downlink frame assembler.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Telemetry Buffer Layout
\ ------------------------------------------------------------------------------

32 CONSTANT TM-HOUSEKEEPING-PAYLOAD-BYTES
CREATE TM-PAYLOAD-BUF TM-HOUSEKEEPING-PAYLOAD-BYTES ALLOT
CREATE TM-PACKET-BUF 128 ALLOT

VARIABLE TM-PACKET-COUNT

\ ------------------------------------------------------------------------------
\ Section 2: Housekeeping Telemetry Serialization
\ ------------------------------------------------------------------------------

\ TM-PACK-HOUSEKEEPING ( -- buf_addr len )
\ Serialize real-time avionics telemetry into 32-byte binary payload:
\ Offset 0..3:  Uptime seconds
\ Offset 4:     PAT current state
\ Offset 5:     FDIR active severity
\ Offset 6..7:  Laser pump current (mA)
\ Offset 8..11: Laser temperature (mK)
\ Offset 12..15: Bench temperature (mK)
\ Offset 16..19: CPA Azimuth encoder (ticks)
\ Offset 20..23: CPA Elevation encoder (ticks)
\ Offset 24..25: FSM RMS error (urad)
\ Offset 26..27: Link Margin (0.1 dB)
\ Offset 28..31: CRC-32 checksum
: TM-PACK-HOUSEKEEPING ( -- buf len )
    TM-PAYLOAD-BUF TM-HOUSEKEEPING-PAYLOAD-BYTES ERASE

    \ Uptime seconds
    TIME-MILLIS 1000 / DUP >R
    $FF AND TM-PAYLOAD-BUF 3 + C!
    R@ 8 RSHIFT $FF AND TM-PAYLOAD-BUF 2 + C!
    R@ 16 RSHIFT $FF AND TM-PAYLOAD-BUF 1 + C!
    R> 24 RSHIFT $FF AND TM-PAYLOAD-BUF 0 + C!

    \ State and Severity
    PAT-CURRENT-STATE @ TM-PAYLOAD-BUF 4 + C!
    FAULT-ACTIVE-SEVERITY @ TM-PAYLOAD-BUF 5 + C!

    \ Laser pump current
    REG-LASER-PUMP-CUR HW@ DUP >R
    $FF AND TM-PAYLOAD-BUF 7 + C!
    R> 8 RSHIFT $FF AND TM-PAYLOAD-BUF 6 + C!

    \ Temperatures
    THERM-TEMP-LASER-MK @ DUP >R
    $FF AND TM-PAYLOAD-BUF 11 + C!
    R@ 8 RSHIFT $FF AND TM-PAYLOAD-BUF 10 + C!
    R@ 16 RSHIFT $FF AND TM-PAYLOAD-BUF 9 + C!
    R> 24 RSHIFT $FF AND TM-PAYLOAD-BUF 8 + C!

    THERM-TEMP-BENCH-MK @ DUP >R
    $FF AND TM-PAYLOAD-BUF 15 + C!
    R@ 8 RSHIFT $FF AND TM-PAYLOAD-BUF 14 + C!
    R@ 16 RSHIFT $FF AND TM-PAYLOAD-BUF 13 + C!
    R> 24 RSHIFT $FF AND TM-PAYLOAD-BUF 12 + C!

    \ CPA Encoders
    CPA-AZ-TICKS@ DUP >R
    $FF AND TM-PAYLOAD-BUF 19 + C!
    R@ 8 RSHIFT $FF AND TM-PAYLOAD-BUF 18 + C!
    R@ 16 RSHIFT $FF AND TM-PAYLOAD-BUF 17 + C!
    R> 24 RSHIFT $FF AND TM-PAYLOAD-BUF 16 + C!

    CPA-EL-TICKS@ DUP >R
    $FF AND TM-PAYLOAD-BUF 23 + C!
    R@ 8 RSHIFT $FF AND TM-PAYLOAD-BUF 22 + C!
    R@ 16 RSHIFT $FF AND TM-PAYLOAD-BUF 21 + C!
    R> 24 RSHIFT $FF AND TM-PAYLOAD-BUF 20 + C!

    \ FSM RMS Error
    TRACK-RMS-ERROR-URAD @ DUP >R
    $FF AND TM-PAYLOAD-BUF 25 + C!
    R> 8 RSHIFT $FF AND TM-PAYLOAD-BUF 24 + C!

    \ Link margin
    BER-COMPUTE-MARGIN DUP >R
    $FF AND TM-PAYLOAD-BUF 27 + C!
    R> 8 RSHIFT $FF AND TM-PAYLOAD-BUF 26 + C!

    TM-PAYLOAD-BUF TM-HOUSEKEEPING-PAYLOAD-BYTES
;

\ TM-GENERATE-SPACE-PACKET ( -- pkt_addr total_len )
\ Build complete Space Packet with primary header and CRC.
: TM-GENERATE-SPACE-PACKET ( -- addr len )
    TM-PACK-HOUSEKEEPING
    \ Create Space Packet: ( apid is_tc payload len out )
    APID-PAYLOAD-HEALTH 0 2SWAP TM-PACKET-BUF
    SPP-CREATE-PACKET
    1 TM-PACKET-COUNT +!
    TM-PACKET-BUF SWAP
;

\ TM-INIT ( -- )
: TM-INIT ( -- )
    0 TM-PACKET-COUNT !
    TM-PAYLOAD-BUF TM-HOUSEKEEPING-PAYLOAD-BYTES ERASE
;
