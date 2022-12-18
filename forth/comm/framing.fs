\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/framing.fs
\ Description: CCSDS AOS/TM Transfer Frame multiplexer, Virtual Channel (VC)
\              routing, frame counter management, and idle fill frame generator.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Transfer Frame Layout and Constants
\ ------------------------------------------------------------------------------

\ CCSDS 1024-byte Transfer Frame
1024 CONSTANT TF-FRAME-LEN-BYTES
6    CONSTANT TF-PRIMARY-HEADER-BYTES
2    CONSTANT TF-FECF-BYTES \ Frame Error Control Field (CRC-16)

0 CONSTANT VC-HOUSEKEEPING   \ VC 0: Critical real-time health telemetry
1 CONSTANT VC-SCIENCE       \ VC 1: Science payload data
2 CONSTANT VC-CROSS-LINK    \ VC 2: High-rate inter-satellite data
7 CONSTANT VC-IDLE-FILL     \ VC 7: Idle fill frame

VARIABLE TF-MASTER-FRAME-COUNT
CREATE TF-VC-FRAME-COUNTERS 8 CELLS ALLOT

\ Spacecraft ID: 0x0042
$0042 CONSTANT TF-SPACECRAFT-ID

\ ------------------------------------------------------------------------------
\ Section 2: Frame Header Generator
\ ------------------------------------------------------------------------------

\ TF-BUILD-HEADER ( vc_id buf_addr -- )
\ Populate 6-byte Transfer Frame Primary Header.
: TF-BUILD-HEADER ( vc buf -- )
    >R
    \ Byte 0..1: Version (01b for AOS, 00b for TM), Spacecraft ID (10 bits), VCID (3 bits)
    TF-SPACECRAFT-ID 3 LSHIFT OVER $07 AND OR
    DUP 8 RSHIFT $FF AND R@ 0 + C!
    $FF AND R@ 1 + C!

    \ Byte 2: Master Channel Frame Count
    TF-MASTER-FRAME-COUNT @ $FF AND R@ 2 + C!
    1 TF-MASTER-FRAME-COUNT +!

    \ Byte 3: Virtual Channel Frame Count
    DUP CELLS TF-VC-FRAME-COUNTERS +
    DUP @ $FF AND R@ 3 + C!
    1 SWAP +!

    \ Byte 4..5: Signaling Field and First Header Pointer (0x0000)
    0 R@ 4 + C!
    0 R@ 5 + C!

    DROP R> DROP
;

\ ------------------------------------------------------------------------------
\ Section 3: Transfer Frame Assembler
\ ------------------------------------------------------------------------------

\ TF-GENERATE-FRAME ( vc_id data_src_addr data_len out_frame_buf -- )
\ Build complete 1024-byte transfer frame with header, data, padding, and CRC.
: TF-GENERATE-FRAME ( vc src len out -- )
    >R >R >R
    \ Populate Primary Header
    DUP R@ TF-BUILD-HEADER

    \ Copy Payload Data
    R> R> R@ TF-PRIMARY-HEADER-BYTES + SWAP CMOVE

    \ Fill remainder of frame up to FECF with idle pattern 0xAA
    \ Header (6) + Data (len) to (1024 - 2)
    \ Clear trailing space
    R@ TF-FRAME-LEN-BYTES 2 - + 0 SWAP C! \ Terminator

    DROP R> DROP
;

\ TF-GENERATE-IDLE ( out_frame_buf -- )
\ Generate an idle fill frame (VC 7) to maintain optical transceiver lock.
: TF-GENERATE-IDLE ( out -- )
    DUP >R
    VC-IDLE-FILL R@ TF-BUILD-HEADER

    \ Fill payload with pseudo-random idle pattern
    TF-FRAME-LEN-BYTES TF-PRIMARY-HEADER-BYTES - 2 - 0 DO
        $AA R@ TF-PRIMARY-HEADER-BYTES + I + C!
    LOOP

    R> DROP
;

\ TF-INIT ( -- )
: TF-INIT ( -- )
    0 TF-MASTER-FRAME-COUNT !
    TF-VC-FRAME-COUNTERS 8 CELLS ERASE
;
