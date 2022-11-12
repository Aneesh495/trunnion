\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: forth/hal/timer_registers.fs
\ Description: Monotonic microsecond hardware timer, watchdog feed register,
\              and real-time execution cadence pacing.
\
\ Copyright (c) 2026 Aneesh Krishna Parthasarathy
\ Licensed under the ISC License.
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Monotonic Time Registers
\ ------------------------------------------------------------------------------

\ TIME-MICROS ( -- d_us )
\ Retrieve 64-bit monotonic microsecond counter from payload timer hardware.
: TIME-MICROS ( -- d_us )
    REG-SYS-CLOCK-LO HW@
    REG-SYS-CLOCK-HI HW@
;

\ TIME-MILLIS ( -- ms )
\ Retrieve monotonic time converted to 32-bit milliseconds.
: TIME-MILLIS ( -- ms )
    TIME-MICROS 1000 S>D D/ D>S
;

\ TICKS@ ( -- ticks )
\ Read accumulated 1 kHz real-time interrupt ticks.
: TICKS@ ( -- ticks )
    REG-TICK-COUNT HW@
;

\ ------------------------------------------------------------------------------
\ Section 2: Watchdog Supervisor Feed
\ ------------------------------------------------------------------------------

\ WD-FEED! ( -- )
\ Feed payload watchdog timer by writing the magic pattern 0xA55A.
: WD-FEED! ( -- )
    WD-MAGIC-FEED REG-WD-FEED HW!
;

\ ------------------------------------------------------------------------------
\ Section 3: Cadence and Delay Helpers
\ ------------------------------------------------------------------------------

\ DELAY-US ( us -- )
\ Cooperative busy-wait loop for microsecond interval timing.
: DELAY-US ( us -- )
    HW-STEP
;

\ DELAY-MS ( ms -- )
\ Delay execution by specified milliseconds.
: DELAY-MS ( ms -- )
    1000 * DELAY-US
;
