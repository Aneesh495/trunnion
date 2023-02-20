#!/usr/bin/env python3
"""
TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
File: sim/orbital_sim.py
Description: High-fidelity orbital mechanics simulator for inter-satellite laser links.
             Models two-body Keplerian dynamics, J2 Earth oblateness secular drift,
             cross-link line-of-sight geometry, range rate, and relativistic point-ahead.

Author: Aneesh Krishna Parthasarathy
License: ISC License
"""

import math
import sys
from typing import Dict, Tuple, List, Optional

# Physical Constants (WGS-84 / EGM96)
MU_EARTH = 398600.4418      # Earth gravitational parameter [km^3 / s^2]
EARTH_RADIUS = 6378.137     # Earth equatorial radius [km]
J2_EARTH = 1.08262668e-3    # Earth second zonal harmonic (J2 oblateness)
SPEED_OF_LIGHT = 299792.458 # Speed of light [km / s]
ATMOSPHERE_BUFFER = 100.0   # Atmospheric limb occultation margin [km]


class OrbitState:
    """Represents Keplerian orbital elements and secular drift rates."""

    def __init__(
        self,
        semi_major_axis_km: float,
        eccentricity: float,
        inclination_deg: float,
        raan_deg: float,
        arg_perigee_deg: float,
        mean_anomaly_deg: float,
        name: str = "SAT",
    ):
        self.name = name
        self.a = semi_major_axis_km
        self.e = eccentricity
        self.inc = math.radians(inclination_deg)
        self.raan = math.radians(raan_deg)
        self.omega = math.radians(arg_perigee_deg)
        self.m0 = math.radians(mean_anomaly_deg)

        # Derived parameters
        self.p = self.a * (1.0 - self.e**2) # Semi-latus rectum [km]
        self.n0 = math.sqrt(MU_EARTH / (self.a**3)) # Unperturbed mean motion [rad/s]

        # J2 secular drift rates
        self._compute_j2_rates()

    def _compute_j2_rates(self) -> None:
        """Compute secular drift rates for RAAN, argument of perigee, and mean motion."""
        factor = (3.0 / 2.0) * J2_EARTH * ((EARTH_RADIUS / self.p) ** 2) * self.n0
        # Nodal precession: d(RAAN)/dt = -3/2 J2 (R/p)^2 n cos(i)
        self.raan_rate = -factor * math.cos(self.inc)
        # Apsidal rotation: d(omega)/dt = 3/4 J2 (R/p)^2 n (5 cos^2(i) - 1)
        self.omega_rate = 0.5 * factor * (5.0 * (math.cos(self.inc) ** 2) - 1.0)
        # Secular mean motion correction: n_bar = n0 [1 + 3/4 J2 (R/p)^2 sqrt(1 - e^2) (3 cos^2(i) - 1)]
        eta = math.sqrt(1.0 - self.e**2)
        self.n_bar = self.n0 * (1.0 + 0.5 * factor * eta * (3.0 * (math.cos(self.inc) ** 2) - 1.0))

    def solve_kepler(self, mean_anomaly: float) -> float:
        """Solve Kepler's equation M = E - e*sin(E) using Newton-Raphson iteration."""
        m_norm = mean_anomaly % (2.0 * math.pi)
        e_curr = m_norm
        for _ in range(15):
            f = e_curr - self.e * math.sin(e_curr) - m_norm
            f_prime = 1.0 - self.e * math.cos(e_curr)
            delta = f / f_prime
            e_curr -= delta
            if abs(delta) < 1e-12:
                break
        return e_curr

    def propagate(self, elapsed_sec: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Propagate orbit state by elapsed time in seconds.
        Returns position (x, y, z) [km] and velocity (vx, vy, vz) [km/s] in ECI frame.
        """
        # Update secular elements
        current_raan = self.raan + self.raan_rate * elapsed_sec
        current_omega = self.omega + self.omega_rate * elapsed_sec
        current_m = self.m0 + self.n_bar * elapsed_sec

        # Solve eccentric anomaly
        e_anom = self.solve_kepler(current_m)

        # True anomaly nu
        sin_e = math.sin(e_anom)
        cos_e = math.cos(e_anom)
        nu = math.atan2(math.sqrt(1.0 - self.e**2) * sin_e, cos_e - self.e)

        # Radius r
        r = self.a * (1.0 - self.e * cos_e)

        # Perifocal coordinates (P, Q, W)
        x_p = r * math.cos(nu)
        y_p = r * math.sin(nu)

        # Perifocal velocities
        sqrt_mu_p = math.sqrt(MU_EARTH / self.p)
        vx_p = -sqrt_mu_p * math.sin(nu)
        vy_p = sqrt_mu_p * (self.e + math.cos(nu))

        # Rotation matrix from perifocal to ECI frame
        cr = math.cos(current_raan)
        sr = math.sin(current_raan)
        co = math.cos(current_omega)
        so = math.sin(current_omega)
        ci = math.cos(self.inc)
        si = math.sin(self.inc)

        p_x = cr * co - sr * so * ci
        p_y = sr * co + cr * so * ci
        p_z = so * si

        q_x = -cr * so - sr * co * ci
        q_y = -sr * so + cr * co * ci
        q_z = co * si

        # Transform to ECI
        r_eci = (
            x_p * p_x + y_p * q_x,
            x_p * p_y + y_p * q_y,
            x_p * p_z + y_p * q_z,
        )

        v_eci = (
            vx_p * p_x + vy_p * q_x,
            vx_p * p_y + vy_p * q_y,
            vx_p * p_z + vy_p * q_z,
        )

        return r_eci, v_eci


class CrossLinkSimulator:
    """Models geometry, link dynamics, Doppler shift, and point-ahead for two satellites."""

    def __init__(self, sat_tx: OrbitState, sat_rx: OrbitState):
        self.sat_tx = sat_tx
        self.sat_rx = sat_rx

    def compute_link_geometry(self, elapsed_sec: float) -> Dict[str, object]:
        """Compute complete line-of-sight and cross-link geometry at elapsed time."""
        r_tx, v_tx = self.sat_tx.propagate(elapsed_sec)
        r_rx, v_rx = self.sat_rx.propagate(elapsed_sec)

        # Relative separation vector: rho = r_rx - r_tx
        rho = (
            r_rx[0] - r_tx[0],
            r_rx[1] - r_tx[1],
            r_rx[2] - r_tx[2],
        )
        range_km = math.sqrt(rho[0] ** 2 + rho[1] ** 2 + rho[2] ** 2)

        # Unit line-of-sight vector u_los
        u_los = (rho[0] / range_km, rho[1] / range_km, rho[2] / range_km)

        # Relative velocity: v_rel = v_rx - v_tx
        v_rel = (
            v_rx[0] - v_tx[0],
            v_rx[1] - v_tx[1],
            v_rx[2] - v_tx[2],
        )

        # Range rate (line-of-sight Doppler velocity) [km/s]
        range_rate_kms = u_los[0] * v_rel[0] + u_los[1] * v_rel[1] + u_los[2] * v_rel[2]

        # Relativistic Doppler factor: (1 - v/c)
        beta_los = range_rate_kms / SPEED_OF_LIGHT
        doppler_shift_ppm = beta_los * 1e6

        # Transverse velocity components (orthogonal to LOS)
        v_perp = (
            v_rel[0] - range_rate_kms * u_los[0],
            v_rel[1] - range_rate_kms * u_los[1],
            v_rel[2] - range_rate_kms * u_los[2],
        )
        v_perp_mag = math.sqrt(v_perp[0] ** 2 + v_perp[1] ** 2 + v_perp[2] ** 2)

        # Point-ahead angle: theta_PA = 2 * v_perp / c [urad]
        point_ahead_urad = (2.0 * v_perp_mag / SPEED_OF_LIGHT) * 1e6

        # Atmospheric occultation check
        is_occulted, min_altitude_km = self._check_earth_occultation(r_tx, r_rx)

        return {
            "elapsed_sec": elapsed_sec,
            "range_km": range_km,
            "range_rate_kms": range_rate_kms,
            "doppler_shift_ppm": doppler_shift_ppm,
            "v_perp_mag_kms": v_perp_mag,
            "point_ahead_urad": point_ahead_urad,
            "is_occulted": is_occulted,
            "min_altitude_km": min_altitude_km,
            "u_los": u_los,
        }

    def _check_earth_occultation(
        self, r1: Tuple[float, float, float], r2: Tuple[float, float, float]
    ) -> Tuple[bool, float]:
        """
        Determine if line-of-sight between r1 and r2 intersects Earth limb.
        Returns (is_occulted_flag, closest_approach_altitude_km).
        """
        # Direction vector d = r2 - r1
        d = (r2[0] - r1[0], r2[1] - r1[1], r2[2] - r1[2])
        d_len_sq = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
        d_len = math.sqrt(d_len_sq)

        # Projection parameter t = -(r1 . d) / |d|^2
        r1_dot_d = r1[0] * d[0] + r1[1] * d[1] + r1[2] * d[2]
        t = -r1_dot_d / d_len_sq

        # Closest point of approach along line segment [0, 1]
        t_clamped = max(0.0, min(1.0, t))
        p_close = (
            r1[0] + t_clamped * d[0],
            r1[1] + t_clamped * d[1],
            r1[2] + t_clamped * d[2],
        )
        min_radius = math.sqrt(p_close[0] ** 2 + p_close[1] ** 2 + p_close[2] ** 2)
        min_altitude = min_radius - EARTH_RADIUS

        is_blocked = min_altitude < ATMOSPHERE_BUFFER
        return is_blocked, min_altitude


def main() -> None:
    """Execute sample orbital cross-link simulation between two LEO satellites."""
    print("================================================================")
    print("TRUNNION: Optical Cross-Link Orbital Simulation")
    print("================================================================")

    # Satellite A (Leader): 7000 km altitude, circular 53-deg inclination
    sat_a = OrbitState(
        semi_major_axis_km=7000.0,
        eccentricity=0.001,
        inclination_deg=53.0,
        raan_deg=0.0,
        arg_perigee_deg=0.0,
        mean_anomaly_deg=0.0,
        name="SAT-LEO-A",
    )

    # Satellite B (Follower): Same orbit plane, 5 deg true anomaly phase lag
    sat_b = OrbitState(
        semi_major_axis_km=7000.0,
        eccentricity=0.001,
        inclination_deg=53.0,
        raan_deg=0.0,
        arg_perigee_deg=0.0,
        mean_anomaly_deg=5.0,
        name="SAT-LEO-B",
    )

    sim = CrossLinkSimulator(sat_a, sat_b)

    print(f"Propagating cross-link geometry over 6000 seconds (1 full orbit)...")
    print(f"{'Time [s]':>8} {'Range [km]':>12} {'Rate [km/s]':>12} {'Doppler [ppm]':>14} {'Pt-Ahead [urad]':>16} {'Occulted':>10}")
    print("-" * 76)

    for t_sec in range(0, 6001, 600):
        geo = sim.compute_link_geometry(float(t_sec))
        occ_str = "YES" if geo["is_occulted"] else "NO"
        print(
            f"{geo['elapsed_sec']:8.0f} "
            f"{geo['range_km']:12.2f} "
            f"{geo['range_rate_kms']:12.4f} "
            f"{geo['doppler_shift_ppm']:14.2f} "
            f"{geo['point_ahead_urad']:16.2f} "
            f"{occ_str:>10}"
        )

    print("================================================================")
    print("Orbital simulation completed successfully.")


if __name__ == "__main__":
    main()
