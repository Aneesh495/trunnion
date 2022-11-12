\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/comm_registers.fs
\ Description: Optical communications physical layer receiver telemetry,
\              slot clock lock status, and high-speed packet FIFO registers.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ Receiver Sensitivity Thresholds:
\ Minimum received power for 1 Gbps PPM lock: 50 nanowatts
\ Nominal link margin threshold: 200 nanowatts
50 CONSTANT COMM-MIN-RX-POWER-NW
200 CONSTANT COMM-NOMINAL-RX-POWER-NW

\ ------------------------------------------------------------------------------
\ Section 1: Link Status and Power Telemetry
\ ------------------------------------------------------------------------------

\ COMM-STATUS@ ( -- stat )
: COMM-STATUS@ ( -- stat )
    REG-COMM-STATUS HW@
;

\ COMM-RX-POWER@ ( -- nW )
\ Read instantaneous optical power delivered to avalanche photodiode (APD).
: COMM-RX-POWER@ ( -- nW )
    REG-COMM-RX-POWER HW@
;

\ COMM-SLOT-LOCK? ( -- flag )
\ Return TRUE if slot clock recovery PLL has locked to PPM symbol boundaries.
: COMM-SLOT-LOCK? ( -- flag )
    COMM-STATUS@ COMM-STAT-SLOT-CLOCK-LOCK AND 0<>
;

\ COMM-FRAME-SYNC? ( -- flag )
\ Return TRUE if Attached Synchronization Marker (ASM) is verified.
: COMM-FRAME-SYNC? ( -- flag )
    COMM-STATUS@ COMM-STAT-ASM-SYNCHRONIZED AND 0<>
;

\ COMM-LINK-ESTABLISHED? ( -- flag )
\ Return TRUE if end-to-end optical communication link is active.
: COMM-LINK-ESTABLISHED? ( -- flag )
    COMM-STATUS@ COMM-STAT-COMM-LINK-ESTABLISHED AND 0<>
;

\ COMM-BER-EST@ ( -- ber_scaled )
\ Estimated Bit Error Rate scaled by 10^7 (e.g. 100 = 10^-5).
: COMM-BER-EST@ ( -- ber )
    REG-COMM-BER-EST HW@
;

\ ------------------------------------------------------------------------------
\ Section 2: Transfer Frame Statistics Counters
\ ------------------------------------------------------------------------------

\ COMM-TX-COUNT@ ( -- count )
: COMM-TX-COUNT@ ( -- count )
    REG-COMM-FRAMES-TX HW@
;

\ COMM-RX-COUNT@ ( -- count )
: COMM-RX-COUNT@ ( -- count )
    REG-COMM-FRAMES-RX HW@
;

\ COMM-CRC-ERRS@ ( -- count )
: COMM-CRC-ERRS@ ( -- count )
    REG-COMM-CRC-ERRS HW@
;

\ ------------------------------------------------------------------------------
\ Section 3: High-Speed Packet FIFO DMA
\ ------------------------------------------------------------------------------

\ COMM-FIFO-READ ( dst len -- read_len )
\ Read incoming optical frame payload from hardware FIFO.
: COMM-FIFO-READ ( dst len -- read_len )
    DUP 0 DO
        REG-COMM-FIFO-DATA HW@
        OVER I + C!
    LOOP
    NIP
;

\ COMM-FIFO-WRITE ( src len -- written_len )
\ Push outgoing space packet into optical modem transmission FIFO.
: COMM-FIFO-WRITE ( src len -- written_len )
    DUP 0 DO
        OVER I + C@
        REG-COMM-FIFO-DATA HW!
    LOOP
    NIP
;
