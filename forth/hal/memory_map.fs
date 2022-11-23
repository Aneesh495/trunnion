\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/memory_map.fs
\ Description: Memory-mapped register offsets, control bitmasks, and hardware
\              interfaces for optical communication payload avionics.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Subsystem Base Offsets and Portability Stubs
\ ------------------------------------------------------------------------------

[UNDEFINED] HW@ [IF]
\ Simulated memory-mapped hardware array for host Forth environments (e.g. Gforth)
CREATE SIM-HW-REGS 4096 CELLS ALLOT
: HW@ ( reg_offset -- val )
    CELLS SIM-HW-REGS + @
;
: HW! ( val reg_offset -- )
    CELLS SIM-HW-REGS + !
;
: HW-STEP ( dt_us -- )
    DROP
;
: HW-CLOCK ( -- time_lo time_hi )
    0 0
;
[THEN]

$0000 CONSTANT SYS-REG-BASE     \ System management and clock
$0100 CONSTANT CPA-REG-BASE     \ Coarse Pointing Assembly (gimbal motors & encoders)
$0200 CONSTANT FSM-REG-BASE     \ Fast Steering Mirror (piezo tip/tilt & capacitive sensors)
$0300 CONSTANT QPD-REG-BASE     \ Quadrant Photodiode & Acquisition Sensor
$0400 CONSTANT LASER-REG-BASE   \ 1550nm Master Oscillator Power Amplifier (MOPA) & Shutter
$0500 CONSTANT COMM-REG-BASE    \ Optical Modem (PPM serializer, slot CDR, sync marker)
$0600 CONSTANT TIMER-REG-BASE   \ Monotonic hardware clock & watchdog timer

\ ------------------------------------------------------------------------------
\ Section 2: System Management Registers (Base: 0x0000)
\ ------------------------------------------------------------------------------

SYS-REG-BASE $00 + CONSTANT REG-SYS-MAGIC        \ Magic signature: 0x5452554E ('TRUN')
SYS-REG-BASE $01 + CONSTANT REG-SYS-STATUS       \ System status & mode flags
SYS-REG-BASE $02 + CONSTANT REG-SYS-CLOCK-LO     \ Monotonic microsecond clock (low 32-bit)
SYS-REG-BASE $03 + CONSTANT REG-SYS-CLOCK-HI     \ Monotonic microsecond clock (high 32-bit)
SYS-REG-BASE $04 + CONSTANT REG-SYS-RESET        \ Software reset & subsystem reload
SYS-REG-BASE $05 + CONSTANT REG-SYS-FAULT-MASK   \ Fault interrupt enable mask
SYS-REG-BASE $06 + CONSTANT REG-SYS-TEMP-BOARD   \ Avionics backplane temperature (milli-Kelvin)
SYS-REG-BASE $07 + CONSTANT REG-SYS-VOLT-BUS     \ 28V spacecraft power bus telemetry (millivolts)

\ System Status Bitmasks
1 0 LSHIFT CONSTANT SYS-STAT-POWER-ON
1 1 LSHIFT CONSTANT SYS-STAT-BOOT-COMPLETE
1 2 LSHIFT CONSTANT SYS-STAT-SAFE-MODE
1 3 LSHIFT CONSTANT SYS-STAT-CALIBRATED
1 4 LSHIFT CONSTANT SYS-STAT-FAULT-ACTIVE

\ ------------------------------------------------------------------------------
\ Section 3: Coarse Pointing Assembly (CPA) Registers (Base: 0x0100)
\ ------------------------------------------------------------------------------

CPA-REG-BASE $00 + CONSTANT REG-CPA-STATUS       \ CPA status & interlock register
CPA-REG-BASE $01 + CONSTANT REG-CPA-CTRL         \ CPA servo control & mode select
CPA-REG-BASE $02 + CONSTANT REG-CPA-AZ-POS       \ Azimuth absolute resolver position (ticks)
CPA-REG-BASE $03 + CONSTANT REG-CPA-EL-POS       \ Elevation absolute resolver position (ticks)
CPA-REG-BASE $04 + CONSTANT REG-CPA-AZ-VEL       \ Azimuth estimated angular velocity (urad/s)
CPA-REG-BASE $05 + CONSTANT REG-CPA-EL-VEL       \ Elevation estimated angular velocity (urad/s)
CPA-REG-BASE $06 + CONSTANT REG-CPA-AZ-CMD-RATE  \ Azimuth commanded velocity DAC (mA / ticks/s)
CPA-REG-BASE $07 + CONSTANT REG-CPA-EL-CMD-RATE  \ Elevation commanded velocity DAC (mA / ticks/s)
CPA-REG-BASE $08 + CONSTANT REG-CPA-AZ-CURRENT   \ Azimuth motor winding current (milliamperes)
CPA-REG-BASE $09 + CONSTANT REG-CPA-EL-CURRENT   \ Elevation motor winding current (milliamperes)
CPA-REG-BASE $0A + CONSTANT REG-CPA-TEMP-AZ      \ Azimuth motor core temperature (milli-Kelvin)
CPA-REG-BASE $0B + CONSTANT REG-CPA-TEMP-EL      \ Elevation motor core temperature (milli-Kelvin)

\ CPA Status Bitmasks
1 0 LSHIFT CONSTANT CPA-STAT-ENABLED
1 1 LSHIFT CONSTANT CPA-STAT-AZ-HOMED
1 2 LSHIFT CONSTANT CPA-STAT-EL-HOMED
1 3 LSHIFT CONSTANT CPA-STAT-AZ-LIMIT-TRIP
1 4 LSHIFT CONSTANT CPA-STAT-EL-LIMIT-TRIP
1 5 LSHIFT CONSTANT CPA-STAT-OVERCURRENT
1 6 LSHIFT CONSTANT CPA-STAT-OVERTEMP

\ ------------------------------------------------------------------------------
\ Section 4: Fast Steering Mirror (FSM) Registers (Base: 0x0200)
\ ------------------------------------------------------------------------------

FSM-REG-BASE $00 + CONSTANT REG-FSM-STATUS       \ FSM piezo status & calibration register
FSM-REG-BASE $01 + CONSTANT REG-FSM-CTRL         \ FSM loop mode (open-loop, closed-loop, offload)
FSM-REG-BASE $02 + CONSTANT REG-FSM-TIP-CMD      \ Commanded tip angle (nanoradians)
FSM-REG-BASE $03 + CONSTANT REG-FSM-TILT-CMD     \ Commanded tilt angle (nanoradians)
FSM-REG-BASE $04 + CONSTANT REG-FSM-TIP-FB       \ Capacitive sensor tip feedback (nanoradians)
FSM-REG-BASE $05 + CONSTANT REG-FSM-TILT-FB      \ Capacitive sensor tilt feedback (nanoradians)
FSM-REG-BASE $06 + CONSTANT REG-FSM-ACT1-DAC     \ Piezo actuator 1 high-voltage DAC
FSM-REG-BASE $07 + CONSTANT REG-FSM-ACT2-DAC     \ Piezo actuator 2 high-voltage DAC
FSM-REG-BASE $08 + CONSTANT REG-FSM-ACT3-DAC     \ Piezo actuator 3 high-voltage DAC
FSM-REG-BASE $09 + CONSTANT REG-FSM-OFFLOAD-X    \ Accumulated low-frequency tip offload to CPA
FSM-REG-BASE $0A + CONSTANT REG-FSM-OFFLOAD-Y    \ Accumulated low-frequency tilt offload to CPA
FSM-REG-BASE $0B + CONSTANT REG-FSM-TEMP-BENCH   \ Carbon-composite optical bench temperature

\ FSM Status Bitmasks
1 0 LSHIFT CONSTANT FSM-STAT-ENABLED
1 1 LSHIFT CONSTANT FSM-STAT-CLOSED-LOOP
1 2 LSHIFT CONSTANT FSM-STAT-TIP-SATURATED
1 3 LSHIFT CONSTANT FSM-STAT-TILT-SATURATED
1 4 LSHIFT CONSTANT FSM-STAT-SENSOR-VALID

\ ------------------------------------------------------------------------------
\ Section 5: Quadrant Photodiode (QPD) Registers (Base: 0x0300)
\ ------------------------------------------------------------------------------

QPD-REG-BASE $00 + CONSTANT REG-QPD-STATUS       \ Acquisition detector status
QPD-REG-BASE $01 + CONSTANT REG-QPD-GAIN-CTRL    \ Transimpedance amplifier (TIA) gain select
QPD-REG-BASE $02 + CONSTANT REG-QPD-ADC-A        \ Quadrant A instantaneous ADC counts (0..65535)
QPD-REG-BASE $03 + CONSTANT REG-QPD-ADC-B        \ Quadrant B instantaneous ADC counts
QPD-REG-BASE $04 + CONSTANT REG-QPD-ADC-C        \ Quadrant C instantaneous ADC counts
QPD-REG-BASE $05 + CONSTANT REG-QPD-ADC-D        \ Quadrant D instantaneous ADC counts
QPD-REG-BASE $06 + CONSTANT REG-QPD-SUM          \ Total optical power sum (A + B + C + D)
QPD-REG-BASE $07 + CONSTANT REG-QPD-POS-X        \ Estimated normalized horizontal position dx/sum
QPD-REG-BASE $08 + CONSTANT REG-QPD-POS-Y        \ Estimated normalized vertical position dy/sum
QPD-REG-BASE $09 + CONSTANT REG-QPD-AMBIENT      \ Estimated background solar/ambient flux

\ QPD Status Bitmasks
1 0 LSHIFT CONSTANT QPD-STAT-POWER-GOOD
1 1 LSHIFT CONSTANT QPD-STAT-BEACON-DETECTED
1 2 LSHIFT CONSTANT QPD-STAT-ADC-SATURATED
1 3 LSHIFT CONSTANT QPD-STAT-SOLAR-BLINDING

\ ------------------------------------------------------------------------------
\ Section 6: Laser Transmitter & Optical Shutter Registers (Base: 0x0400)
\ ------------------------------------------------------------------------------

LASER-REG-BASE $00 + CONSTANT REG-LASER-STATUS   \ Laser safety interlock & emission status
LASER-REG-BASE $01 + CONSTANT REG-LASER-CTRL     \ Laser enable, mode (CW beacon, PPM data)
LASER-REG-BASE $02 + CONSTANT REG-LASER-SEED-CUR \ 1550nm DFB seed laser diode current (mA)
LASER-REG-BASE $03 + CONSTANT REG-LASER-PUMP-CUR \ 980nm EDFA pump diode current (mA)
LASER-REG-BASE $04 + CONSTANT REG-LASER-TX-POWER \ Monitored optical output power (milliwatts)
LASER-REG-BASE $05 + CONSTANT REG-LASER-TEMP     \ Laser bench temperature (milli-Kelvin)
LASER-REG-BASE $06 + CONSTANT REG-LASER-SHUTTER  \ Beam dump safety shutter (0=closed, 1=open)

\ Laser Status Bitmasks
1 0 LSHIFT CONSTANT LASER-STAT-INTERLOCK-CLOSED
1 1 LSHIFT CONSTANT LASER-STAT-SEED-READY
1 2 LSHIFT CONSTANT LASER-STAT-PUMP-READY
1 3 LSHIFT CONSTANT LASER-STAT-EMISSION-ACTIVE
1 4 LSHIFT CONSTANT LASER-STAT-SHUTTER-OPEN
1 5 LSHIFT CONSTANT LASER-STAT-OVERTEMP-TRIP

\ ------------------------------------------------------------------------------
\ Section 7: Optical Modem & CCSDS Comm Registers (Base: 0x0500)
\ ------------------------------------------------------------------------------

COMM-REG-BASE $00 + CONSTANT REG-COMM-STATUS     \ Modem link status & synchronization flags
COMM-REG-BASE $01 + CONSTANT REG-COMM-CTRL       \ Modulation order (4, 8, 16, 32, 64, 128, 256 PPM)
COMM-REG-BASE $02 + CONSTANT REG-COMM-SLOT-CLK   \ Slot clock frequency synthesizer (Hz)
COMM-REG-BASE $03 + CONSTANT REG-COMM-RX-POWER   \ Received optical signal power (nanowatts)
COMM-REG-BASE $04 + CONSTANT REG-COMM-BER-EST    \ Estimated bit error rate (scaled 10^7)
COMM-REG-BASE $05 + CONSTANT REG-COMM-FRAMES-TX  \ Transmitted valid CCSDS transfer frames
COMM-REG-BASE $06 + CONSTANT REG-COMM-FRAMES-RX  \ Received and verified CCSDS transfer frames
COMM-REG-BASE $07 + CONSTANT REG-COMM-CRC-ERRS   \ Frame CRC validation error counter
COMM-REG-BASE $08 + CONSTANT REG-COMM-FIFO-DATA  \ High-speed frame packet FIFO data port

\ Comm Status Bitmasks
1 0 LSHIFT CONSTANT COMM-STAT-RX-POWER-LOCK
1 1 LSHIFT CONSTANT COMM-STAT-SLOT-CLOCK-LOCK
1 2 LSHIFT CONSTANT COMM-STAT-ASM-SYNCHRONIZED
1 3 LSHIFT CONSTANT COMM-STAT-FEC-DECODE-PASS
1 4 LSHIFT CONSTANT COMM-STAT-COMM-LINK-ESTABLISHED

\ ------------------------------------------------------------------------------
\ Section 8: Watchdog and Cadence Timer Registers (Base: 0x0600)
\ ------------------------------------------------------------------------------

TIMER-REG-BASE $00 + CONSTANT REG-WD-FEED        \ Watchdog heart-beat kick register (write 0xA55A)
TIMER-REG-BASE $01 + CONSTANT REG-WD-TIMEOUT     \ Watchdog expiration period (milliseconds)
TIMER-REG-BASE $02 + CONSTANT REG-CADENCE-CTRL   \ 1 kHz real-time interrupt timer rate
TIMER-REG-BASE $03 + CONSTANT REG-TICK-COUNT     \ Accumulated real-time interrupt ticks

$A55A CONSTANT WD-MAGIC-FEED
