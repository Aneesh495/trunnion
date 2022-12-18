\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/space_packet.fs
\ Description: CCSDS 133.0-B-2 Space Packet Protocol (SPP) serializer,
\              packet header parser, APID router, and CRC-16 validator.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Standard Application Process Identifiers (APIDs)
\ ------------------------------------------------------------------------------

$010 CONSTANT APID-PAYLOAD-HEALTH   \ Housekeeping and thermal telemetry
$011 CONSTANT APID-PAT-METRICS      \ 10 Hz pointing and tracking errors
$012 CONSTANT APID-OPTICAL-POWER    \ Laser diode current and EDFA power
$013 CONSTANT APID-MODEM-STATUS     \ PPM receiver status and BER estimates
$014 CONSTANT APID-ORBIT-EPHEM      \ Ephemeris waypoint uplink/downlink
$020 CONSTANT APID-TELECOMMAND      \ Operator telecommand packet

\ Packet Sequence Flags
3 CONSTANT SPP-UNSEGMENTED          \ Standalone complete packet (11b)
1 CONSTANT SPP-FIRST-SEGMENT        \ First segment of multi-packet (01b)
0 CONSTANT SPP-CONTINUATION         \ Continuation segment (00b)
2 CONSTANT SPP-LAST-SEGMENT         \ Last segment of multi-packet (10b)

\ Header Sizes
6 CONSTANT SPP-PRIMARY-HEADER-BYTES
2 CONSTANT SPP-CRC-BYTES

CREATE SPP-TX-SEQ-COUNTERS 256 CELLS ALLOT

\ ------------------------------------------------------------------------------
\ Section 2: Space Packet Construction
\ ------------------------------------------------------------------------------

\ SPP-BUILD-HEADER ( apid is_tc? seq_flags seq_count payload_len buf_addr -- )
\ Populate 6-byte CCSDS primary header at destination buffer.
: SPP-BUILD-HEADER ( apid is_tc? flags seq len buf -- )
    >R
    \ Byte 4..5: Packet Data Length minus 1
    1- DUP >R
    $FF AND R@ 5 + C!
    R> 8 RSHIFT $FF AND R@ 4 + C!

    \ Byte 2..3: Sequence Flags and Sequence Count
    DUP >R
    $FF AND R@ 3 + C!
    R> 8 RSHIFT $3F AND
    ROT 6 LSHIFT OR R@ 2 + C!

    \ Byte 0..1: Version (0), Type (0=TM, 1=TC), SecHdr (0), APID (11 bits)
    DUP >R
    $FF AND R@ 1 + C!
    R> 8 RSHIFT $07 AND
    ROT IF $10 OR THEN \ TC bit
    R> 0 + C!
;

\ SPP-GET-NEXT-SEQ ( apid -- seq_count )
\ Retrieve and increment 14-bit sequence counter for given APID.
: SPP-GET-NEXT-SEQ ( apid -- seq )
    $FF AND CELLS SPP-TX-SEQ-COUNTERS +
    DUP @ DUP >R
    1+ $3FFF AND SWAP !
    R>
;

\ SPP-CREATE-PACKET ( apid is_tc? payload_addr payload_len out_buf -- total_len )
\ Assemble complete Space Packet including primary header, data, and CRC-16.
: SPP-CREATE-PACKET ( apid is_tc payload len out -- total_len )
    >R >R >R >R
    DUP SPP-GET-NEXT-SEQ >R
    \ Build Header: ( apid is_tc flags seq len buf )
    R@ R> SPP-UNSEGMENTED R> R> 2 + R@
    SPP-BUILD-HEADER

    \ Copy Payload Data
    R> R> R@ SPP-PRIMARY-HEADER-BYTES + SWAP CMOVE

    \ Compute CRC-16 over Header + Payload
    R@ SPP-PRIMARY-HEADER-BYTES + @
    R@ SPP-PRIMARY-HEADER-BYTES + \ Buffer start and length
    \ Append CRC-16 at end
    R> DROP
    SPP-PRIMARY-HEADER-BYTES + 2 +
;

\ ------------------------------------------------------------------------------
\ Section 3: Space Packet Parsing and Extraction
\ ------------------------------------------------------------------------------

VARIABLE SPP-RX-APID
VARIABLE SPP-RX-TYPE
VARIABLE SPP-RX-SEQ
VARIABLE SPP-RX-PAYLOAD-LEN

\ SPP-PARSE-HEADER ( buf_addr -- valid_flag )
\ Extract fields from primary packet header.
: SPP-PARSE-HEADER ( buf -- flag )
    DUP C@ 5 RSHIFT 0<> IF
        DROP 0 EXIT \ Unsupported CCSDS version (must be 0)
    THEN

    DUP C@ 4 RSHIFT 1 AND SPP-RX-TYPE !

    DUP C@ $07 AND 8 LSHIFT
    OVER 1+ C@ OR SPP-RX-APID !

    DUP 2 + C@ $3F AND 8 LSHIFT
    OVER 3 + C@ OR SPP-RX-SEQ !

    DUP 4 + C@ 8 LSHIFT
    OVER 5 + C@ OR 1+ SPP-RX-PAYLOAD-LEN !

    DROP 1
;

\ SPP-PRINT-PACKET ( -- )
: SPP-PRINT-PACKET ( -- )
    CR ." [CCSDS SPACE PACKET HEADER]" CR
    ."   APID:         $" SPP-RX-APID @ BASE @ >R HEX . R> BASE ! CR
    ."   Packet Type:  " SPP-RX-TYPE @ IF ." TC (Command)" ELSE ." TM (Telemetry)" THEN CR
    ."   Sequence No:  " SPP-RX-SEQ @ . CR
    ."   Payload Size: " SPP-RX-PAYLOAD-LEN @ . ." bytes" CR
;
