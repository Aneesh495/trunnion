# Precision Pointing, Acquisition, and Tracking (PAT) Theory

Optical Inter-Satellite Links (OISLs) operate with beam divergence angles on the order of 10 to 25 microradians across inter-satellite ranges from 1,000 km to over 45,000 km. To establish and maintain a high-bandwidth communication cross-link, the satellite must perform three sequential phases:

1. **Pointing (Open-Loop)**: Slew the optical head toward the predicted ephemeris line-of-sight.
2. **Acquisition (Spatial Search)**: Scan the uncertainty volume until the partner terminal's optical beacon is captured.
3. **Tracking (Closed-Loop)**: Continuously reject spacecraft base vibrations and maintain sub-microradian optical alignment.

---

## 1. Spatial Acquisition and Scanning Patterns

When establishing cross-links, uncertainty in satellite orbit determination (GNSS ephemeris) and spacecraft attitude determination (star trackers, IMUs) defines an **Uncertainty Cone** of semi-angle $\Theta_{unc} \approx 1\text{ to }5\text{ mrad}$, which vastly exceeds the narrow beam divergence $\theta_{div} \approx 20\text{ }\mu\text{rad}$.

### 1.1 Archimedean Spiral Scan

Trunnion implements an Archimedean spiral search profile parameterized in polar coordinates $(r, \theta)$:

$$r(\theta) = a \cdot \theta$$

where:
- $a = \frac{\Delta r}{2\pi}$ is the radial growth rate.
- $\Delta r \le 0.75 \cdot \theta_{div}$ is the radial overlap pitch between successive spiral arms to ensure zero blind spots across the scan cone.

To maintain a constant spatial search rate and uniform photon integration time on the acquisition sensor, the angular velocity $\omega(t) = \dot{\theta}(t)$ must decay inversely with radius:

$$v = r \cdot \dot{\theta} = \text{const} \implies \dot{\theta}(t) = \frac{v}{a \cdot \theta(t)}$$

Integrating over time yields the explicit discrete update sequence executed in `forth/pat/spiral_scan.fs`:

$$\theta(t) = \sqrt{\frac{2 v t}{a} + \theta_0^2}, \quad r(t) = \sqrt{2 a v t + r_0^2}$$

### 1.2 Hexagonal Lissajous Search

For terminals with rectangular optical detector geometries or asymmetric attitude uncertainty ellipses, Trunnion provides orthogonal harmonic scanning:

$$x(t) = A_x \sin(\omega_x t + \phi_x), \quad y(t) = A_y \sin(\omega_y t + \phi_y)$$

Choosing prime harmonic ratios (e.g. $f_x / f_y = 11 / 13$) synthesizes a dense, non-repeating Lissajous lattice that covers the rectangular search area with guaranteed revisit intervals.

---

## 2. Optical Centroiding and Error Sensing

Once the optical beacon is acquired, the flight software measures angular misalignment using two complementary sensor topologies.

### 2.1 Quadrant Photodiode (QPD) Normalized Error

A quadrant photodiode segments the incoming Gaussian beam spot across four photosensitive sectors ($A, B, C, D$). The normalized error transfer function yields dimensionless pointing error coordinates:

$$\epsilon_x = \frac{(A + D) - (B + C)}{A + B + C + D}, \quad \epsilon_y = \frac{(A + B) - (C + D)}{A + B + C + D}$$

For small angular errors ($|\epsilon| < 0.5$), the relation to physical angular displacement is linear:

$$\Delta\theta_x \approx k_{qpd} \cdot \epsilon_x, \quad \Delta\theta_y \approx k_{qpd} \cdot \epsilon_y$$

where the responsivity slope $k_{qpd}$ is proportional to the beam spot radius $w_0$ divided by the optical system effective focal length $f_{eff}$:

$$k_{qpd} = \frac{\sqrt{2\pi} \cdot w_0}{4 f_{eff}}$$

### 2.2 Sub-Pixel Center-of-Gravity (CoG) on Focal Plane Arrays

For wide-field tracking sensors, Trunnion computes intensity-weighted first moments across an $N \times N$ pixel region-of-interest (ROI):

$$x_{cog} = \frac{\sum_{i,j} (i - i_0) \cdot I(i, j)}{\sum_{i,j} I(i, j)}, \quad y_{cog} = \frac{\sum_{i,j} (j - j_0) \cdot I(i, j)}{\sum_{i,j} I(i, j)}$$

Fixed-point calculations in `forth/optics/nfov_detector.fs` accumulate integer intensities into double-precision registers before computing the Q32.32 division, guaranteeing sub-pixel resolution down to 0.05 pixels without floating-point overhead.

---

## 3. Relativistic Point-Ahead Angle (PAA)

Because light travels at a finite velocity ($c = 299,792\text{ km/s}$), photons transmitted from satellite A to satellite B require a transit time:

$$\tau = \frac{R}{c}$$

Across a typical LEO-to-LEO cross-link range $R = 5,000\text{ km}$, $\tau \approx 16.68\text{ ms}$. Over this transit window, satellite B travels along its orbit by a displacement $\Delta \vec{r}_B = \vec{v}_B \cdot \tau$.

Transmitting directly along the perceived Line-of-Sight (LOS) would cause the narrow 20-microradian laser beam to completely miss the target satellite. The optical terminal must therefore point ahead of the apparent target position by the **Point-Ahead Angle**:

$$\vec{\theta}_{PA} = \frac{2 \vec{v}_\perp}{c}$$

where $\vec{v}_\perp = \vec{v}_{rel} - (\vec{v}_{rel} \cdot \hat{u}_{los})\hat{u}_{los}$ is the transverse component of relative velocity orthogonal to the line of sight.

For typical low-Earth orbits ($v_{rel} \approx 7.5\text{ km/s}$), the point-ahead angle reaches:

$$\theta_{PA} \approx \frac{2 \times 7.5\text{ km/s}}{300,000\text{ km/s}} = 50.0\text{ }\mu\text{rad}$$

which is more than double the beam divergence half-angle. Trunnion computes $\vec{\theta}_{PA}$ dynamically in `forth/pat/point_ahead.fs` using high-rate orbit propagation vectors.

---

## 4. Dual-Stage Actuation Topology

Sub-microradian tracking requires simultaneously handling wide angular ranges ($\pm 180^\circ$ azimuth) and high-frequency disturbance rejection (up to 200 Hz). Trunnion accomplishes this via a dual-stage cooperative control architecture:

```
Disturbance Theta_d
        |
        v
       (+) <------------------------------------+
        |                                        |
        v Error e                                |
     [ QPD / NFOV Sensor ]                       |
        |                                        |
        v                                        |
+-------------------+                            |
| FSM Lead-Lag / PID| ---> [ Fast Piezo Mirror ]-+ (High Bandwidth: 0 - 500 Hz)
+-------------------+            |
        |                        |
        v u_fsm                  |
+-------------------+            |
| Offload Filter    |            |
| H_off(s) = K / s  |            |
+-------------------+            |
        |                        |
        v u_cpa                  |
+-------------------+            |
| CPA Gimbal Servo  | ---> [ Stepper / BLDC ]----+ (Wide Range: +/- 180 deg)
+-------------------+
```

1. **Fine Steering Mirror (FSM)**:
   - High-rate piezoelectric actuators operating at 1 kHz cadence.
   - Rejects high-frequency micro-vibrations (60 Hz RWA unbalances, 120 Hz solar panel torsion).
   - Dynamic stroke range: $\pm 350\text{ }\mu\text{rad}$.

2. **Coarse Pointing Assembly (CPA)**:
   - Dual-axis azimuth/elevation motorized gimbal operating at 100 Hz cadence.
   - S-curve trajectory profiler limits actuator jerk and acceleration.
   - LuGre friction observer compensates for nonlinear Coulomb and Stribeck breakaway friction.

3. **Offload Mechanism**:
   - Integrates steady-state FSM deflection. When the piezo deflection exceeds $150\text{ }\mu\text{rad}$, the offload supervisor commands the CPA gimbal to slew, desaturating the piezo mirror back to its central optical zero.
