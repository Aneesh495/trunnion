\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/comm/ppm_modem.fs
\ Description: Multi-order Pulse Position Modulation (M-PPM) modulator, slot
\              timing clock, guard slot inserter, and maximum-likelihood
\              photon-counting soft/hard decision slicer (CCSDS 141.0-B-1).
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Modulation Parameters and State
\ ------------------------------------------------------------------------------

VARIABLE PPM-ORDER              \ M = 4, 8, 16, 32, 64, 128, 256
VARIABLE PPM-BITS-PER-SYMBOL    \ log2(M)
VARIABLE PPM-SLOT-WIDTH-PS      \ Slot duration in picoseconds (e.g. 500 ps = 2 Gslot/s)
VARIABLE PPM-GUARD-SLOTS        \ Trailing dead-time slots between symbols (e.g. 4 slots)
VARIABLE PPM-TOTAL-SLOTS        \ M + guard_slots

\ Symbol Statistics
VARIABLE PPM-TX-SYMBOL-COUNT
VARIABLE PPM-RX-SYMBOL-COUNT
VARIABLE PPM-RX-PEAK-PHOTONS
VARIABLE PPM-RX-NOISE-FLOOR

\ Slot Photon Count Accumulator (Max order 256 + 32 guard = 288 slots)
300 CONSTANT PPM-MAX-SLOTS
CREATE PPM-SLOT-BUFFER PPM-MAX-SLOTS CELLS ALLOT

\ ------------------------------------------------------------------------------
\ Section 2: Modem Initialization and Mode Selection
\ ------------------------------------------------------------------------------

\ PPM-SET-ORDER ( m_order -- success_flag )
\ Configure modulation order M (must be power of 2 between 4 and 256).
: PPM-SET-ORDER ( m -- flag )
    DUP CASE
        4   OF 2 PPM-BITS-PER-SYMBOL ! ENDOF
        8   OF 3 PPM-BITS-PER-SYMBOL ! ENDOF
        16  OF 4 PPM-BITS-PER-SYMBOL ! ENDOF
        32  OF 5 PPM-BITS-PER-SYMBOL ! ENDOF
        64  OF 6 PPM-BITS-PER-SYMBOL ! ENDOF
        128 OF 7 PPM-BITS-PER-SYMBOL ! ENDOF
        256 OF 8 PPM-BITS-PER-SYMBOL ! ENDOF
        DROP 0 EXIT
    ENDCASE

    DUP PPM-ORDER !
    4 PPM-GUARD-SLOTS ! \ Default 4 guard slots for laser recovery
    4 + PPM-TOTAL-SLOTS !
    500 PPM-SLOT-WIDTH-PS ! \ 500 ps nominal
    0 PPM-TX-SYMBOL-COUNT !
    0 PPM-RX-SYMBOL-COUNT !
    1
;

\ PPM-INIT ( -- )
\ Initialize modem with default 16-PPM order (4 bits/symbol).
: PPM-INIT ( -- )
    16 PPM-SET-ORDER DROP
    PPM-SLOT-BUFFER PPM-MAX-SLOTS CELLS ERASE
;

\ ------------------------------------------------------------------------------
\ Section 3: PPM Symbol Encoding (TX Modulator)
\ ------------------------------------------------------------------------------

\ PPM-ENCODE-SYMBOL ( bits -- slot_index )
\ Map input bitfield into pulse slot index [0, M-1].
: PPM-ENCODE-SYMBOL ( bits -- slot )
    PPM-ORDER @ 1- AND
    1 PPM-TX-SYMBOL-COUNT +!
;

\ PPM-ENCODE-BYTE ( byte -- slot0 slot1 )
\ For 16-PPM (4 bits/symbol), encode 8-bit byte into two consecutive PPM slots.
: PPM-ENCODE-BYTE ( byte -- s0 s1 )
    DUP 4 RSHIFT PPM-ENCODE-SYMBOL
    SWAP $0F AND PPM-ENCODE-SYMBOL
;

\ ------------------------------------------------------------------------------
\ Section 4: Maximum-Likelihood Soft/Hard Decision Slicer (RX Demodulator)
\ ------------------------------------------------------------------------------

VARIABLE SLICE-BEST-SLOT
VARIABLE SLICE-MAX-COUNT
VARIABLE SLICE-SECOND-MAX

\ PPM-SLICE-FRAME ( buf_addr -- symbol_bits snr_metric )
\ Find slot with maximum photon counts using maximum-likelihood decision rule.
: PPM-SLICE-FRAME ( buf -- symbol snr )
    >R
    0 SLICE-MAX-COUNT !
    0 SLICE-SECOND-MAX !
    0 SLICE-BEST-SLOT !

    PPM-ORDER @ 0 DO
        R@ I CELLS + @ \ Photon count in slot I
        DUP SLICE-MAX-COUNT @ > IF
            SLICE-MAX-COUNT @ SLICE-SECOND-MAX !
            SLICE-MAX-COUNT !
            I SLICE-BEST-SLOT !
        ELSE
            DUP SLICE-SECOND-MAX @ > IF
                SLICE-SECOND-MAX !
            ELSE
                DROP
            THEN
        THEN
    LOOP

    R> DROP
    1 PPM-RX-SYMBOL-COUNT +!

    \ Return best slot (symbol) and extinction ratio metric
    SLICE-BEST-SLOT @
    SLICE-MAX-COUNT @ SLICE-SECOND-MAX @ -
;

\ PPM-PRINT-STATUS ( -- )
: PPM-PRINT-STATUS ( -- )
    CR ." [PPM OPTICAL MODEM STATUS]" CR
    ."   Order (M):       " PPM-ORDER @ . CR
    ."   Bits/Symbol:     " PPM-BITS-PER-SYMBOL @ . CR
    ."   Slot Width:      " PPM-SLOT-WIDTH-PS @ . ." ps" CR
    ."   Total Slots:     " PPM-TOTAL-SLOTS @ . CR
    ."   TX Symbol Count: " PPM-TX-SYMBOL-COUNT @ . CR
    ."   RX Symbol Count: " PPM-RX-SYMBOL-COUNT @ . CR
;
