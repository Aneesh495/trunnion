# CCSDS Optical Communications Subsystem

The Trunnion communications pipeline implements the physical and data-link layers defined in the Consultative Committee for Space Data Systems (CCSDS) Blue Book standard **CCSDS 141.0-B-1 (Optical Communications Physical Layer)** and **CCSDS 133.0-B-1 (Space Packet Protocol)**.

Designed for free-space optical (FSO) satellite cross-links and deep-space downlinks, the modem employs photon-efficient Pulse Position Modulation combined with Reed-Solomon forward error correction and deep matrix block interleaving to overcome deep optical fading bursts.

---

## 1. M-ary Pulse Position Modulation (M-PPM)

In peak-power-limited laser cross-links, intensity modulation with direct detection (IM/DD) using Pulse Position Modulation (PPM) maximizes photon information efficiency (bits per photon) compared to conventional On-Off Keying (OOK).

### 1.1 Signal Structure

An $M$-PPM symbol frame of duration $T_{frame}$ is partitioned into $M$ equal time slots of duration $T_{slot} = T_{frame} / M$. Exactly one laser optical pulse is emitted in the slot corresponding to the integer symbol value $S \in \{0, 1, \dots, M-1\}$:

$$s(t) = \sum_{k=-\infty}^{\infty} P_{peak} \cdot p(t - k T_{frame} - S_k T_{slot})$$

where each symbol conveys $k = \log_2(M)$ bits of information. Trunnion supports $M \in \{4, 8, 16, 32, 64\}$, with 16-PPM as the nominal cross-link baseline (4 bits per optical pulse).

```
Slot Index:   0     1     2     3     4     5     6     7     ...    15
            +-----+-----+-----+-----+-----+-----+-----+-----+       +-----+
Pulse:      |     |     |     |     |#####|     |     |     |       |     |  (Symbol = 4)
            +-----+-----+-----+-----+-----+-----+-----+-----+       +-----+
            |<-- T_slot ------------>|
            |<--------------------- T_frame = 16 * T_slot --------------->|
```

### 1.2 Maximum-Likelihood (ML) Decision Slicer

At the receiver, a photon-counting detector (e.g. SNSPD or APD in Geiger mode) records arrival counts $\{k_0, k_1, \dots, k_{M-1}\}$ in each slot. Assuming Poisson counting statistics:

$$\hat{S} = \arg\max_{0 \le i < M} k_i$$

In `forth/comm/ppm_modem.fs`, the decision rule tracks both the peak count and the second-highest count to compute the real-time extinction ratio and signal-to-noise metric:

$$\Delta k = k_{max} - k_{second}$$

---

## 2. Attached Sync Marker & Frame Synchronization

Frame boundaries are identified by correlating the demodulated bitstream against a 32-bit Attached Sync Marker (ASM).

### 2.1 Standard 32-Bit Sync Marker

Trunnion utilizes the official CCSDS optical cross-link sync marker:

$$\text{ASM}_{32} = \text{0x1ACFFC1D} = 0001\text{ }1010\text{ }1100\text{ }1111\text{ }1111\text{ }1100\text{ }0001\text{ }1101_2$$

The cyclic autocorrelation function of $\text{ASM}_{32}$ exhibits ideal sharp side-lobes bounded by $\le 0.125$, preventing false acquisition during symbol slip events.

### 2.2 Sync Correlator State Machine

The frame correlator in `forth/comm/sync_marker.fs` operates a three-state synchronization supervisor:

1. **SEARCH State**: Computes bitwise XOR and Hamming weight (`POPCOUNT`) between a sliding 32-bit window and $\text{ASM}_{32}$. If $d_H \le 2$ bit errors, advances to VERIFY.
2. **VERIFY State**: Validates that successive sync markers occur at exact frame length intervals. If 3 consecutive frames match, advances to LOCK.
3. **LOCK State**: Passes synchronized codewords to the Reed-Solomon decoder. Tolerates up to 3 corrupted sync markers before dropping lock back to SEARCH.

---

## 3. Reed-Solomon RS(255, 223) Error Correction Codec

Optical cross-links are subject to burst errors from pointing jitter and satellite body nutation. Trunnion applies the CCSDS standard Reed-Solomon $(N=255, K=223)$ code over the Galois Field $GF(2^8)$.

### 3.1 Galois Field $GF(2^8)$ Arithmetic

The field $GF(2^8)$ is constructed using the CCSDS primitive polynomial:

$$p(x) = x^8 + x^4 + x^3 + x^2 + 1 \quad (\text{0x11D})$$

Precomputed antilog ($GF\text{-}EXP$) and logarithm ($GF\text{-}LOG$) tables in `forth/comm/reed_solomon.fs` reduce Galois Field multiplication to simple addition modulo 255:

$$a \cdot b = \alpha^{(\log_\alpha(a) + \log_\alpha(b)) \bmod 255}$$

### 3.2 Generator Polynomial & Encoder

The code generator polynomial is the product of 32 consecutive roots in $GF(2^8)$:

$$g(x) = \prod_{i=112}^{143} (x - \alpha^{11 \cdot i}) = \sum_{j=0}^{32} g_j x^j$$

The encoder computes 32 parity symbols via linear-feedback shift register (LFSR) polynomial division of the 223-byte message polynomial $m(x) \cdot x^{32} \pmod{g(x)}$, achieving maximum distance separable (MDS) performance capable of correcting up to $t = 16$ symbol errors per 255-byte codeword:

$$d_{min} = N - K + 1 = 33 \implies t = \left\lfloor \frac{d_{min} - 1}{2} \right\rfloor = 16\text{ bytes}$$

---

## 4. Matrix Block Interleaving

To mitigate long optical fading events (e.g. when pointing jitter exceeds the divergence cone for several milliseconds), Trunnion sandwiches the RS codewords inside a configurable **Matrix Block Interleaver** of depth $I \in \{1, 2, 4, 8\}$.

```
           Codeword 0: [ c0,0   c0,1   c0,2   ...  c0,254 ]
           Codeword 1: [ c1,0   c1,1   c1,2   ...  c1,254 ]
Input:     Codeword 2: [ c2,0   c2,1   c2,2   ...  c2,254 ]
(Row-by-Row)  ...
           Codeword I-1:[ cI-1,0 cI-1,1 ...        cI-1,254 ]

Transmission Order: c0,0, c1,0, ..., cI-1,0, c0,1, c1,1, ..., cI-1,254 (Column-by-Column)
```

With an interleaving depth of $I = 8$, a contiguous optical burst fade destroying up to $16 \times 8 = 128$ consecutive transmitted bytes is distributed across 8 separate codewords, allowing each RS decoder instance to correct its local slice without frame loss.

---

## 5. CCSDS Space Packet Protocol & AOS Framing

At the network layer, flight software encapsulates payload engineering telemetry and commands into standard **CCSDS Space Packets (CCSDS 133.0-B-1)**:

```
+------------------------------------------------------------------------+
|                     CCSDS PACKET PRIMARY HEADER (6 Bytes)              |
+------------------------------------+-----------------+-----------------+
| Version (3b) | Type (1b) | Sec (1b)| APID (11b)      | Flags (2b)      |
+------------------------------------+-----------------+-----------------+
| Packet Sequence Count (14b)        | Packet Data Length minus 1 (16b)  |
+------------------------------------+-----------------+-----------------+
|                     PACKET DATA FIELD (Variable)                       |
|   Telemetry Structures, Sensor Floats, Command Payload                 |
+------------------------------------------------------------------------+
|                     PACKET ERROR CONTROL (2 Bytes)                     |
|   CRC-16-CCITT Checksum (Polynomial: 0x1021)                           |
+------------------------------------------------------------------------+
```

Multi-packet telemetry is multiplexed into fixed-length **Advanced Orbiting Systems (AOS) Transfer Frames (CCSDS 732.0-B-3)** with Virtual Channel Identifiers (VCID) dedicating separate streams for real-time attitude data, optical science telemetry, and operator engineering logs.
