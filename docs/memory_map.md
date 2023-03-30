# Avionics Memory Map & Hardware Abstraction Layer

The Trunnion flight software interfaces with physical optomechanical mechanisms, photonics hardware, and payload avionics via a unified 64-bit cell memory map.

The virtual machine and hardware abstraction layer (HAL) expose standard memory-mapped input/output (MMIO) registers at designated physical offsets, accessible using standard Forth `@` (fetch) and `!` (store) operators.

---

## 1. Physical Memory Architecture

The 64-bit address space is partitioned into execution code, static Forth dictionary data, transient string/frame buffers, and memory-mapped peripheral registers:

```
+-------------------------------------------------------------------------+
| Address Range               | Size        | Usage                       |
+-----------------------------+-------------+-----------------------------+
| 0x00000000 - 0x000001FF     | 512 B       | Boot Vectors & CPU Flags    |
| 0x00000200 - 0x00000FFF     | 3.5 KB      | Transient Literal Buffers   |
| 0x00001000 - 0x00001FFF     | 4.0 KB      | Peripheral MMIO Space       |
| 0x00002000 - 0x000FFFFF     | 1.0 MB      | Forth Dictionary & Heap     |
| 0x00100000 - 0x001FFFFF     | 1.0 MB      | Telemetry Ring Buffers      |
| 0x00200000 - 0x002FFFFF     | 1.0 MB      | CCSDS Interleaver Matrices  |
| 0x00300000 - 0x003FFFFF     | 1.0 MB      | Return & Parameter Stacks   |
+-------------------------------------------------------------------------+
```

---

## 2. Memory-Mapped I/O (MMIO) Peripheral Register Map

All peripheral hardware registers reside in the `0x1000 - 0x1FFF` region. Registers are 32-bit aligned unless specified as 64-bit timestamp words.

### 2.1 Coarse Pointing Assembly (CPA) Base: `0x1000`

The CPA controller interfaces to the azimuth and elevation brushless DC servo motors and optical encoders.

| Offset | Register Name | R/W | Type | Description |
|:-------|:--------------|:---:|:----:|:------------|
| `0x1000` | `CPA_CTRL_REG` | R/W | uint32 | Motor enable (bit 0), brake release (bit 1), auto-homing (bit 2) |
| `0x1004` | `CPA_STAT_REG` | R | uint32 | Ready (bit 0), moving (bit 1), error (bit 2), index locked (bit 3) |
| `0x1008` | `CPA_AZ_CMD` | R/W | int32 | Target azimuth angle in Q16.16 degrees ($[-180.0, +180.0]$) |
| `0x100C` | `CPA_EL_CMD` | R/W | int32 | Target elevation angle in Q16.16 degrees ($[-20.0, +95.0]$) |
| `0x1010` | `CPA_AZ_POS` | R | int32 | Measured azimuth encoder angle in Q16.16 degrees |
| `0x1014` | `CPA_EL_POS` | R | int32 | Measured elevation encoder angle in Q16.16 degrees |
| `0x1018` | `CPA_AZ_VEL` | R | int32 | Measured azimuth angular velocity in Q16.16 deg/s |
| `0x101C` | `CPA_EL_VEL` | R | int32 | Measured elevation angular velocity in Q16.16 deg/s |

### 2.2 Fine Steering Mirror (FSM) Base: `0x1100`

The FSM controller drives the three high-voltage piezo actuators and reads strain-gauge position sensors (SGS).

| Offset | Register Name | R/W | Type | Description |
|:-------|:--------------|:---:|:----:|:------------|
| `0x1100` | `FSM_CTRL_REG` | R/W | uint32 | Piezo driver power (bit 0), closed-loop mode (bit 1) |
| `0x1104` | `FSM_STAT_REG` | R | uint32 | Amplifier ready (bit 0), thermal fault (bit 1), saturation (bit 2) |
| `0x1108` | `FSM_TIP_CMD` | R/W | int32 | Command tip deflection in Q16.16 microradians |
| `0x110C` | `FSM_TILT_CMD` | R/W | int32 | Command tilt deflection in Q16.16 microradians |
| `0x1110` | `FSM_TIP_POS` | R | int32 | Measured SGS tip deflection in Q16.16 microradians |
| `0x1114` | `FSM_TILT_POS` | R | int32 | Measured SGS tilt deflection in Q16.16 microradians |

### 2.3 Quadrant Photodiode (QPD) Optical Receiver Base: `0x1200`

The QPD optical sensor samples spot intensity distributions for coarse acquisition tracking.

| Offset | Register Name | R/W | Type | Description |
|:-------|:--------------|:---:|:----:|:------------|
| `0x1200` | `QPD_CTRL_REG` | R/W | uint32 | Transimpedance amplifier gain (bits 0..2), integration gate (bit 3) |
| `0x1204` | `QPD_STAT_REG` | R | uint32 | Signal detected flag (bit 0), ADC saturation flag (bit 1) |
| `0x1208` | `QPD_QUAD_A` | R | uint32 | Upper-left quadrant photocurrent count |
| `0x120C` | `QPD_QUAD_B` | R | uint32 | Upper-right quadrant photocurrent count |
| `0x1210` | `QPD_QUAD_C` | R | uint32 | Lower-right quadrant photocurrent count |
| `0x1214` | `QPD_QUAD_D` | R | uint32 | Lower-left quadrant photocurrent count |
| `0x1218` | `QPD_SUM_VAL` | R | uint32 | Hardware accumulated total optical energy ($A + B + C + D$) |

### 2.4 Laser Subsystem & Modem Base: `0x1300`

Controls the 1550 nm continuous-wave/pulsed master oscillator power amplifier (MOPA) and PPM modulator.

| Offset | Register Name | R/W | Type | Description |
|:-------|:--------------|:---:|:----:|:------------|
| `0x1300` | `LASER_CTRL_REG` | R/W | uint32 | Laser emission enable (bit 0), pilot beacon enable (bit 1) |
| `0x1304` | `LASER_STAT_REG` | R | uint32 | Interlock state (bit 0), emission active (bit 1), over-temp (bit 2) |
| `0x1308` | `LASER_PWR_CMD` | R/W | uint32 | Optical output power setpoint in milliwatts |
| `0x130C` | `COMM_TX_REG` | W | uint32 | M-PPM transmit FIFO write port |
| `0x1310` | `COMM_RX_REG` | R | uint32 | M-PPM receive FIFO read port |
| `0x1314` | `COMM_STAT_REG` | R | uint32 | TX FIFO empty (bit 0), RX FIFO ready (bit 1), frame lock (bit 2) |

### 2.5 Real-Time Timers & Watchdog Base: `0x1400`

Provides microsecond system timing, hardware heartbeat supervision, and periodic interrupt triggers.

| Offset | Register Name | R/W | Type | Description |
|:-------|:--------------|:---:|:----:|:------------|
| `0x1400` | `TIMER_CTRL_REG` | R/W | uint32 | Timer enable (bit 0), 1 kHz periodic interrupt enable (bit 1) |
| `0x1404` | `TIMER_COUNT_LO` | R | uint32 | Lower 32 bits of 64-bit microsecond counter |
| `0x1408` | `TIMER_COUNT_HI` | R | uint32 | Upper 32 bits of 64-bit microsecond counter |
| `0x140C` | `WATCHDOG_REG` | W | uint32 | Hardware watchdog kick register (write 0xA55A1234) |

---

## 3. Housekeeping Telemetry Binary Packet Format

The flight software streams periodic engineering telemetry formatted as 32-byte binary structures at 10 Hz cadence:

```
 Byte Offset:
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                        Mission Timestamp                      |  (Offset 0)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 | PAT State | CPA Stat  | FSM Stat  | Laser Stat|                  (Offset 4)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                       CPA Azimuth Position                    |  (Offset 8)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                      CPA Elevation Position                   |  (Offset 12)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |        FSM Tip Position       |       FSM Tilt Position       |  (Offset 16)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |       QPD Total Energy        |        Optical SNR Metric     |  (Offset 20)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |    Optical Bench Temp (0.01C) |    Laser Diode Temp (0.01C)   |  (Offset 24)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |     Laser Output Power (mW)   |         CRC-16 Checksum       |  (Offset 28)
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### Telemetry Field Breakdown

1. `Timestamp` (`uint32`, Bytes 0..3): Milliseconds since system initialization.
2. `PAT State` (`uint8`, Byte 4): 0=STANDBY, 1=SLEW, 2=SPIRAL, 3=ACQUIRE, 4=COARSE_TRACK, 5=FINE_TRACK, 6=POINT_AHEAD, 7=FAULT.
3. `CPA Status` (`uint8`, Byte 5): Bitfield indicating gimbal motor power, homing status, and limits.
4. `FSM Status` (`uint8`, Byte 6): Bitfield indicating piezo driver health and offload status.
5. `Laser Status` (`uint8`, Byte 7): Bitfield indicating interlock status, emission status, and TEC lock.
6. `CPA Azimuth` (`int32`, Bytes 8..11): Signed gimbal azimuth in Q16.16 degrees.
7. `CPA Elevation` (`int32`, Bytes 12..15): Signed gimbal elevation in Q16.16 degrees.
8. `FSM Tip` (`int16`, Bytes 16..17): Signed tip deflection in microradians.
9. `FSM Tilt` (`int16`, Bytes 18..19): Signed tilt deflection in microradians.
10. `QPD Energy` (`uint16`, Bytes 20..21): Total raw photocurrent sum counts.
11. `Optical SNR` (`uint16`, Bytes 22..23): Estimated channel SNR in Q8.8 decibels.
12. `Bench Temp` (`int16`, Bytes 24..25): Telescope chassis temperature in hundredths of a degree Celsius.
13. `Laser Temp` (`int16`, Bytes 26..27): Laser seed diode temperature in hundredths of a degree Celsius.
14. `Laser Power` (`uint16`, Bytes 28..29): Optical transmit power in milliwatts.
15. `CRC-16` (`uint16`, Bytes 30..31): CRC-16-CCITT integrity checksum across preceding 30 bytes.
