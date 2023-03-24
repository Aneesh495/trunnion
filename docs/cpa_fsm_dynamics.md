# Mechanism Dynamics & Servo Control

The Trunnion pointing subsystem achieves sub-microradian beam stabilization across astronomical cross-link distances by orchestrating a hierarchical, dual-stage optomechanical pointing assembly:

1. **Coarse Pointing Assembly (CPA)**: A wide-angle, elevation-over-azimuth two-axis gimbal delivering hemispherical coverage ($\pm 180^\circ$ azimuth, $-20^\circ$ to $+95^\circ$ elevation) with moderate bandwidth (closed-loop cutoff $\approx 5\text{ Hz}$).
2. **Fine Steering Mirror (FSM)**: A high-bandwidth tip-tilt piezoelectrically actuated mirror mounted in the optical relay path delivering micro-radian precision over a restricted deflection window ($\pm 5\text{ mrad}$) with closed-loop bandwidth exceeding $200\text{ Hz}$.

---

## 1. Coarse Pointing Assembly (CPA) Kinematics

The CPA mounts the primary optical telescope on an elevation-over-azimuth gimbal assembly attached to the satellite body.

```
       Zenith (+Z_b)
             ^
             |       Elevation Axis (\theta)
             |          /
             |         /
             |     +--/---+
             |     |  O   |  Optical Telescope Bore-Sight
             |     +------+
             |        |
             |        +------- Azimuth Axis (\psi)
             |       /
             |      /
             +-----+----------------> East (+Y_b)
            /
           /
          v
      North (+X_b)
```

### 1.1 Forward Kinematics

Given gimbal azimuth angle $\psi$ and elevation angle $\theta$, the unit line-of-sight vector in spacecraft body coordinates $\mathbf{u}_b = [x_b, y_b, z_b]^T$ is evaluated via the composite rotation matrix:

$$\mathbf{u}_b = \mathbf{R}_z(\psi) \mathbf{R}_y(-\theta) \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} \cos(\psi) \cos(\theta) \\ \sin(\psi) \cos(\theta) \\ \sin(\theta) \end{bmatrix}$$

### 1.2 Inverse Kinematics

Given a desired line-of-sight unit vector $\mathbf{u}_b = [x, y, z]^T$, the target gimbal angles are computed using four-quadrant arctangents:

$$\psi = \text{atan2}(y, x)$$

$$\theta = \text{atan2}\left(z, \sqrt{x^2 + y^2}\right)$$

#### Gimbal Singularity Handling

At near-zenith elevation ($\theta \to +90^\circ$), $\sqrt{x^2 + y^2} \to 0$, creating a kinematic singularity where the azimuth rate $\dot{\psi} \to \infty$. The solver in `forth/kinematics/cpa_kinematics.fs` applies singularity regularization:

$$\text{If } \sqrt{x^2 + y^2} < \epsilon_{singularity} \quad (10^{-4}\text{ rad}), \quad \psi_{target} \leftarrow \psi_{current}$$

#### Shortest-Arc Angular Unwinding

To avoid unnecessary $360^\circ$ gimbal rotations when crossing the branch cut at $\pm \pi$:

$$\Delta \psi = (\psi_{target} - \psi_{current}) \bmod 2\pi$$

$$\text{If } \Delta \psi > \pi, \quad \Delta \psi \leftarrow \Delta \psi - 2\pi$$

$$\text{If } \Delta \psi < -\pi, \quad \Delta \psi \leftarrow \Delta \psi + 2\pi$$

$$\psi_{unwound} = \psi_{current} + \Delta \psi$$

---

## 2. Jerk-Bounded S-Curve Trajectory Generation

Direct step commands to high-inertia gimbal mechanisms excite flexible spacecraft modes, solar array flapping, and optical bench resonances. Trunnion implements a 7-segment S-curve profile bounding position, velocity, acceleration, and jerk.

```
 Acceleration a(t)
       ^
 +a_max|       +-------+               Phase 1: Jerk ramp-up (+j_max)
       |      /         \              Phase 2: Constant acceleration
       |     /           \             Phase 3: Jerk ramp-down (-j_max)
      0+----+-------------+-------+--> Time t
       |                   \     /     Phase 4: Constant velocity (a=0)
       |                    \   /      Phase 5: Jerk ramp-down (-j_max)
 -a_max|                     +-+       Phase 6: Constant deceleration
                                       Phase 7: Jerk ramp-up (+j_max)
```

The profile guarantees $C^2$ continuity of gimbal position:

$$|j(t)| \le j_{max}, \quad |a(t)| \le a_{max}, \quad |v(t)| \le v_{max}$$

In discrete-time fixed-point execution (`forth/kinematics/trajectory.fs`), the interpolator computes state vectors $[s_k, v_k, a_k]^T$ at each $1\text{ kHz}$ scheduler cycle using symplectic Euler integration in Q32.32 format:

$$a_{k+1} = a_k + j_k \cdot \Delta t$$

$$v_{k+1} = v_k + a_{k+1} \cdot \Delta t$$

$$s_{k+1} = s_k + v_{k+1} \cdot \Delta t$$

---

## 3. LuGre Dynamic Friction Model

Bearing and slip-ring friction in vacuum environments exhibit severe non-linearities, including stick-slip phenomena, Stribeck effect (friction drop at low velocities), and pre-sliding displacement hysteresis.

Trunnion incorporates the LuGre friction model in `forth/kinematics/lugre_friction.fs` for feed-forward torque compensation.

### 3.1 Bristle State Dynamics

Friction interface asperities are modeled as elastic bristles with average deflection $z$:

$$\frac{dz}{dt} = v - \frac{\sigma_0 |v|}{g(v)} z$$

where $v$ is relative angular velocity, $\sigma_0$ is bristle stiffness, and $g(v)$ models the Stribeck velocity dependency:

$$g(v) = F_c + (F_s - F_c) e^{-(v / v_s)^2}$$

- $F_c$: Coulomb friction torque.
- $F_s$: Static friction torque ($F_s > F_c$).
- $v_s$: Stribeck velocity parameter.

### 3.2 Total Friction Torque

The resulting friction torque opposing gimbal motion is:

$$T_f = \sigma_0 z + \sigma_1 \frac{dz}{dt} + \sigma_2 v$$

- $\sigma_0$: Micro-stiffness ($N \cdot m / rad$).
- $\sigma_1$: Micro-damping ($N \cdot m \cdot s / rad$).
- $\sigma_2$: Viscous friction coefficient ($N \cdot m \cdot s / rad$).

By computing $T_f$ in real time, the servo controller injects feed-forward torque $\tau_{ff} = +T_f(v, z)$, linearizing the gimbal drive and eliminating limit cycles during slow sidereal tracking.

---

## 4. Fine Steering Mirror (FSM) Actuation

The FSM comprises a lightweight silicon carbide mirror supported by a three-actuator piezo flexure mount.

```
       +-----------------------+
       |   Dielectric Mirror   |
       +-----------------------+
        /          |          \
       /           |           \
   [Piezo A]   [Piezo B]   [Piezo C]   (120 deg apart)
        \          |          /
       +-----------------------+
       |   Base Mounting Flange|
       +-----------------------+
```

### 4.1 Tip-Tilt Kinematics

Mechanical deflection $[\alpha_m, \beta_m]^T$ produces optical beam deflection $[\theta_x, \theta_y]^T$ amplified by the optical law of reflection:

$$\theta_x = 2 \cdot \alpha_m, \quad \theta_y = 2 \cdot \beta_m$$

### 4.2 Lead-Lag Digital Filter

To achieve $200\text{ Hz}$ bandwidth while suppressing the structural mirror resonance at $850\text{ Hz}$, `forth/kinematics/fsm_servo.fs` implements a cascaded biquad lead-lag filter:

$$H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}$$

Direct Form II transposition minimizes coefficient truncation noise in fixed-point registers:

$$w[k] = e[k] - a_1 w[k-1] - a_2 w[k-2]$$

$$u[k] = b_0 w[k] + b_1 w[k-1] + b_2 w[k-2]$$

---

## 5. Dual-Stage Offload Architecture

The coarse and fine pointing stages cooperate in a closed-loop frequency-split configuration:

```
 Target LOS  +   Optical Error   +------------+  u_FSM   +----------+
 ---------->(+)----------------->| FSM Biquad |--------->| FSM DAC  |---+
             ^ -                 | Controller |    |     +----------+   |
             |                   +------------+    |                    |
             |                                     v                    |
             |                              +-------------+             |
             |                              | Low-Pass    |             |
             |                              | Filter      |             |
             |                              +-------------+             |
             |                                     | u_offload          |
             |                                     v                    |
             |                               +------------+             |
             |                               | CPA Servo  |             |
             |                               | Trajectory |             |
             |                               +------------+             |
             |                                     |                    |
             |                                     v                    |
             |                               +------------+             |
             |                               | CPA Motors |             |
             |                               +------------+             |
             |                                     |                    |
             |   Total Optical Pointing Vector     v                    v
             +------------------------------------(+)-------------------+
```

### 5.1 Frequency Splitting Principle

1. High-frequency line-of-sight disturbances (reaction wheel imbalance, solar array vibrations from 5 Hz to 150 Hz) are immediately rejected by the ultra-fast FSM.
2. Low-frequency orbital trajectory changes and drift accumulate in the FSM deflection angle $\theta_{FSM}$.
3. When $|\theta_{FSM}|$ exceeds the offload deadband ($\approx 1.5\text{ mrad}$), the low-pass filtered offload loop commands CPA slews.
4. As the CPA slews to follow the target, optical feedback automatically drives $\theta_{FSM}$ back to zero, maintaining maximum dynamic headroom for transient rejection.
