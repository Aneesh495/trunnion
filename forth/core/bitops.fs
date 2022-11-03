\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/core/bitops.fs
\ Description: Bit manipulation primitives, Hamming weight, Gray coding,
\              CRC-16-CCITT, and CRC-32 for CCSDS optical telemetry frames.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Population Count and Parity
\ ------------------------------------------------------------------------------

\ POPCOUNT ( u -- count )
\ Compute Hamming weight (number of set 1-bits in 64-bit integer) using parallel bitwise tree.
: POPCOUNT ( u -- count )
    DUP 1 RSHIFT $5555555555555555 AND -
    DUP $3333333333333333 AND SWAP 2 RSHIFT $3333333333333333 AND +
    DUP 4 RSHIFT + $0F0F0F0F0F0F0F0F AND
    $0101010101010101 * 56 RSHIFT
;

\ PARITY ( u -- flag )
\ Return 1 if number of set bits is odd, 0 if even.
: PARITY ( u -- flag )
    POPCOUNT 1 AND
;

\ HAMMING-DISTANCE ( u1 u2 -- dist )
\ Compute bitwise Hamming distance between two cells.
: HAMMING-DISTANCE ( u1 u2 -- dist )
    XOR POPCOUNT
;

\ BIT-REVERSE32 ( u -- rev )
\ Reverse the bit order of a 32-bit integer.
: BIT-REVERSE32 ( u -- rev )
    $FFFFFFFF AND
    DUP 16 RSHIFT SWAP 16 LSHIFT OR $FFFFFFFF AND
    DUP 8 RSHIFT $00FF00FF AND SWAP 8 LSHIFT $FF00FF00 AND OR
    DUP 4 RSHIFT $0F0F0F0F AND SWAP 4 LSHIFT $F0F0F0F0 AND OR
    DUP 2 RSHIFT $33333333 AND SWAP 2 LSHIFT $CCCCCCCC AND OR
    DUP 1 RSHIFT $55555555 AND SWAP 1 LSHIFT $AAAAAAAA AND OR
;

\ ------------------------------------------------------------------------------
\ Section 2: Gray Code Encoders and Decoders
\ ------------------------------------------------------------------------------

\ BIN>GRAY ( bin -- gray )
\ Convert binary integer to reflected Gray code.
: BIN>GRAY ( bin -- gray )
    DUP 1 RSHIFT XOR
;

\ GRAY>BIN ( gray -- bin )
\ Convert reflected Gray code to binary integer.
: GRAY>BIN ( gray -- bin )
    DUP 1 RSHIFT XOR
    DUP 2 RSHIFT XOR
    DUP 4 RSHIFT XOR
    DUP 8 RSHIFT XOR
    DUP 16 RSHIFT XOR
    DUP 32 RSHIFT XOR
;

\ ------------------------------------------------------------------------------
\ Section 3: CRC-16-CCITT Generator (Polynomial 0x1021)
\ ------------------------------------------------------------------------------

\ CRC-16-CCITT Lookup Table (256 precomputed entries for 0x1021, init 0xFFFF)
CREATE CRC16-TABLE
    $0000 , $1021 , $2042 , $3063 , $4084 , $50A5 , $60C6 , $70E7 ,
    $8108 , $9129 , $A14A , $B16B , $C18C , $D1AD , $E1CE , $F1EF ,
    $1231 , $0210 , $3273 , $2252 , $52B5 , $4294 , $72F7 , $62D6 ,
    $9339 , $8318 , $B37B , $A35A , $D3BD , $C39C , $F3FF , $E3DE ,
    $2462 , $3443 , $0420 , $1401 , $64E6 , $74C7 , $44A4 , $5485 ,
    $A56A , $B54B , $8528 , $9509 , $E5EE , $F5CF , $C5AC , $D58D ,
    $3653 , $2672 , $1611 , $0630 , $76D7 , $66F6 , $5695 , $46B4 ,
    $B75B , $A77A , $9719 , $8738 , $F7DF , $E7FE , $D79D , $C7BC ,
    $48C4 , $58E5 , $6886 , $78A7 , $0840 , $1861 , $2802 , $3823 ,
    $C9CC , $D9ED , $E98E , $F9AF , $8948 , $9969 , $A90A , $B92B ,
    $5AF5 , $4AD4 , $7AB7 , $6A96 , $1A71 , $0A50 , $3A33 , $2A12 ,
    $DBFD , $CBDC , $FBBF , $EB9E , $9B79 , $8B58 , $BB3B , $AB1A ,
    $6CA6 , $7C87 , $4CE4 , $5CC5 , $2C22 , $3C03 , $0C60 , $1C41 ,
    $EDAE , $FD8F , $CDEC , $DDCD , $AD2A , $BD0B , $8D68 , $9D49 ,
    $7E97 , $6EB6 , $5ED5 , $4EF4 , $3E13 , $2E32 , $1E51 , $0E70 ,
    $FF9F , $EFBE , $DFDD , $CFFC , $BF1B , $AF3A , $9F59 , $8F78 ,
    $9188 , $81A9 , $B1CA , $A1EB , $D10C , $C12D , $F14E , $E16F ,
    $1080 , $00A1 , $30C2 , $20E3 , $5004 , $4025 , $7046 , $6067 ,
    $83B9 , $9398 , $A3FB , $B3DA , $C33D , $D31C , $E37F , $F35E ,
    $02B1 , $1290 , $22F3 , $32D2 , $4235 , $5214 , $6277 , $7256 ,
    $B5EA , $A5CB , $95A8 , $8589 , $F56E , $E54F , $D52C , $C50D ,
    $34E2 , $24C3 , $14A0 , $0481 , $7466 , $6447 , $5424 , $4405 ,
    $A7DB , $B7FA , $8799 , $97B8 , $E75F , $F77E , $C71D , $D73C ,
    $26D3 , $36F2 , $0691 , $16B0 , $6657 , $7676 , $4615 , $5634 ,
    $D94C , $C96D , $F90E , $E92F , $99C8 , $89E9 , $B98A , $A9AB ,
    $5844 , $4865 , $7806 , $6827 , $18C0 , $08E1 , $3882 , $28A3 ,
    $CB7D , $DB5C , $EB3F , $FB1E , $8BF9 , $9BD8 , $ABBB , $BB9A ,
    $4A75 , $5A54 , $6A37 , $7A16 , $0AF1 , $1AD0 , $2AB3 , $3A92 ,
    $FD2E , $ED0F , $DD6C , $CD4D , $BDAA , $AD8B , $9DE8 , $8DC9 ,
    $7C26 , $6C07 , $5C64 , $4C45 , $3CA2 , $2C83 , $1CE0 , $0CC1 ,
    $EF1F , $FF3E , $CF5D , $DF7C , $AF9B , $BFBA , $8FD9 , $9FF8 ,
    $6E17 , $7E36 , $4E55 , $5E74 , $2E93 , $3EB2 , $0ED1 , $1EF0 ,

\ CRC16-CCITT-UPDATE ( crc byte -- crc' )
\ Process single octet into 16-bit CRC state.
: CRC16-CCITT-UPDATE ( crc byte -- crc' )
    SWAP DUP 8 RSHIFT $FF AND \ ( byte crc_hi )
    ROT XOR $FF AND \ Table index
    CELLS CRC16-TABLE + @ \ Table value
    SWAP 8 LSHIFT $FFFF AND XOR
;

\ CRC16-CCITT-BUFFER ( addr len -- crc16 )
\ Compute CRC-16-CCITT checksum over buffer memory.
: CRC16-CCITT-BUFFER ( addr len -- crc )
    $FFFF -ROT \ Initial seed 0xFFFF
    0 DO
        OVER I + C@
        CRC16-CCITT-UPDATE
    LOOP
    NIP
;

\ ------------------------------------------------------------------------------
\ Section 4: CRC-32 Generator (Polynomial 0xEDB88320)
\ ------------------------------------------------------------------------------

CREATE CRC32-TABLE
    $00000000 , $77073096 , $EE0E612C , $990951BA ,
    $076DC419 , $706AF48F , $E963A535 , $9E6495A3 ,
    $0EDB8832 , $79DCB8A4 , $E0D5E91E , $97D2D988 ,
    $09B64C2B , $7EB17CBD , $E7B82D07 , $90BF1D91 ,
    $1DB71064 , $6AB020F2 , $F3B97148 , $84BE41DE ,
    $1ADAD47D , $6DDDE4EB , $F4D4B551 , $83D385C7 ,
    $136C9856 , $646BA8C0 , $FD62F97A , $8A65C9EC ,
    $14015C4F , $63066CD9 , $FA0F3D63 , $8D080DF5 ,
    $3B6E20C8 , $4C69105E , $D56041E4 , $A2677172 ,
    $3C03E4D1 , $4B04D447 , $D20D85FD , $A50AB56B ,
    $35B5A8FA , $42B2986C , $DBBBC9D6 , $ACBCF940 ,
    $32D86CE3 , $45DF5C75 , $DCD60DCF , $ABD13D59 ,
    $26D930AC , $51DE003A , $C8D75180 , $BFD06116 ,
    $21B4F4B5 , $56B3C423 , $CFBA9599 , $B8BDA50F ,
    $2802B89E , $5F058808 , $C60CD9B2 , $B10BE924 ,
    $2F6F7C87 , $58684C11 , $C1611DAB , $B6662D3D ,

\ CRC32-UPDATE ( crc byte -- crc' )
: CRC32-UPDATE ( crc byte -- crc' )
    OVER XOR $0F AND \ Fast 4-bit nibble indexing
    CELLS CRC32-TABLE + @
    SWAP 4 RSHIFT XOR
;

\ CRC32-BUFFER ( addr len -- crc32 )
: CRC32-BUFFER ( addr len -- crc )
    $FFFFFFFF -ROT
    0 DO
        OVER I + C@
        CRC32-UPDATE
    LOOP
    NIP
    $FFFFFFFF XOR
;
