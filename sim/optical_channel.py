#!/usr/bin/env python3
"""
TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
File: sim/optical_channel.py
Description: Free-space optical communication channel and link budget model.
             Simulates Gaussian beam divergence, geometric path loss, receiver
             aperture coupling, pointing jitter induced burst fading, and link margin.

Author: Aneesh Krishna Parthasarathy
License: ISC License
"""

import math
import sys
from typing import Dict, Tuple, List, Optional

# Physical and Optical Constants
SPEED_OF_LIGHT = 299792458.0  # [m/s]
PLANCK_CONSTANT = 6.62607015e-34 # [J*s]
BOLTZMANN_CONST = 1.380649e-23  # [J/K]


class OpticalLinkBudget:
    """
    Computes rigorous radiometric link budgets for free-space laser cross-links
    following CCSDS 141.0-B-1 and ISO/DIS 15459 standards.
    """

    def __init__(
        self,
        wavelength_nm: float = 1550.0,
        tx_power_watts: float = 2.0,
        tx_aperture_diam_m: float = 0.10,
        rx_aperture_diam_m: float = 0.10,
        tx_optics_eff: float = 0.80,
        rx_optics_eff: float = 0.70,
        detector_eff: float = 0.85,
        target_bitrate_mbps: float = 100.0,
        ppm_order: int = 16,
    ):
        self.wavelength = wavelength_nm * 1e-9 # [m]
        self.tx_power = tx_power_watts        # [W]
        self.tx_diam = tx_aperture_diam_m     # [m]
        self.rx_diam = rx_aperture_diam_m     # [m]
        self.tx_eff = tx_optics_eff
        self.rx_eff = rx_optics_eff
        self.detector_eff = detector_eff
        self.bitrate = target_bitrate_mbps * 1e6 # [bps]
        self.ppm_order = ppm_order

        # Photon energy h * c / lambda [J]
        self.photon_energy = (PLANCK_CONSTANT * SPEED_OF_LIGHT) / self.wavelength

        # Diffraction-limited beam divergence half-angle theta = lambda / (pi * w0)
        # Using aperture waist w0 = 0.4 * D_tx
        self.waist_radius = 0.4 * self.tx_diam
        self.divergence_half_angle_rad = self.wavelength / (math.pi * self.waist_radius)
        self.tx_gain_db = 10.0 * math.log10(32.0 / (self.divergence_half_angle_rad ** 2))

        # Receiver aperture area and optical gain
        self.rx_area = math.pi * ((self.rx_diam / 2.0) ** 2)
        self.rx_gain_db = 10.0 * math.log10(
            (math.pi * self.rx_diam / self.wavelength) ** 2
        )

    def compute_free_space_loss_db(self, range_km: float) -> float:
        """Free-space path loss L_FS = (lambda / (4 * pi * R))^2 in dB."""
        range_m = range_km * 1e3
        loss_ratio = (self.wavelength / (4.0 * math.pi * range_m)) ** 2
        return 10.0 * math.log10(loss_ratio)

    def compute_pointing_loss_db(self, radial_jitter_urad: float) -> float:
        """
        Pointing jitter attenuation based on Gaussian beam profile:
        L_point = exp(-2 * (theta_jitter / theta_div)^2)
        """
        jitter_rad = radial_jitter_urad * 1e-6
        normalized_error = jitter_rad / self.divergence_half_angle_rad
        attenuation_ratio = math.exp(-2.0 * (normalized_error ** 2))
        return 10.0 * math.log10(max(1e-12, attenuation_ratio))

    def evaluate_link(
        self, range_km: float, radial_jitter_urad: float = 2.0
    ) -> Dict[str, float]:
        """
        Evaluate full link budget at specified inter-satellite range.
        Returns received power, photon flux, required sensitivity, and link margin.
        """
        tx_power_dbm = 10.0 * math.log10(self.tx_power * 1e3)
        tx_eff_db = 10.0 * math.log10(self.tx_eff)
        rx_eff_db = 10.0 * math.log10(self.rx_eff)

        # Path loss & pointing loss
        fs_loss_db = self.compute_free_space_loss_db(range_km)
        point_loss_db = self.compute_pointing_loss_db(radial_jitter_urad)

        # Received optical power at aperture [dBm]
        rx_power_dbm = (
            tx_power_dbm
            + self.tx_gain_db
            + tx_eff_db
            + fs_loss_db
            + self.rx_gain_db
            + rx_eff_db
            + point_loss_db
        )
        rx_power_watts = 1e-3 * (10.0 ** (rx_power_dbm / 10.0))

        # Received photon rate [photons / second]
        photons_per_sec = rx_power_watts / self.photon_energy

        # Photons per bit = photons_per_sec / bitrate
        photons_per_bit = photons_per_sec / self.bitrate

        # Required sensitivity for M-PPM (nominal 5.0 photons/bit for RS-coded 16-PPM at BER 1e-9)
        required_photons_per_bit = 5.0
        req_power_watts = required_photons_per_bit * self.bitrate * self.photon_energy
        req_power_dbm = 10.0 * math.log10(req_power_watts * 1e3)

        # Link Margin [dB]
        link_margin_db = rx_power_dbm - req_power_dbm

        return {
            "range_km": range_km,
            "radial_jitter_urad": radial_jitter_urad,
            "tx_power_dbm": tx_power_dbm,
            "tx_gain_db": self.tx_gain_db,
            "rx_gain_db": self.rx_gain_db,
            "fs_loss_db": fs_loss_db,
            "point_loss_db": point_loss_db,
            "rx_power_dbm": rx_power_dbm,
            "rx_power_watts": rx_power_watts,
            "photons_per_bit": photons_per_bit,
            "link_margin_db": link_margin_db,
            "req_power_dbm": req_power_dbm,
        }


def main() -> None:
    """Run link budget analysis across cross-link ranges from 500 km to 5000 km."""
    print("================================================================")
    print("TRUNNION: Optical Cross-Link Radiometric Link Budget")
    print("Standard: CCSDS 141.0-B-1 1550nm 16-PPM (100 Mbps)")
    print("================================================================")

    budget = OpticalLinkBudget(
        wavelength_nm=1550.0,
        tx_power_watts=2.0,
        tx_aperture_diam_m=0.10,
        rx_aperture_diam_m=0.10,
        target_bitrate_mbps=100.0,
        ppm_order=16,
    )

    print(f"Diffraction Divergence: {budget.divergence_half_angle_rad * 1e6:.1f} urad (half-angle)")
    print(f"Transmitter Gain:       {budget.tx_gain_db:.2f} dBi")
    print(f"Receiver Gain:          {budget.rx_gain_db:.2f} dBi")
    print()
    print(f"{'Range [km]':>10} {'FS Loss [dB]':>14} {'Pointing [dB]':>14} {'Rx Power [dBm]':>16} {'Photons/bit':>14} {'Margin [dB]':>12}")
    print("-" * 84)

    test_ranges = [500.0, 1000.0, 1500.0, 2000.0, 3000.0, 4000.0, 5000.0]
    for r in test_ranges:
        res = budget.evaluate_link(r, radial_jitter_urad=2.5)
        print(
            f"{res['range_km']:10.0f} "
            f"{res['fs_loss_db']:14.2f} "
            f"{res['point_loss_db']:14.2f} "
            f"{res['rx_power_dbm']:16.2f} "
            f"{res['photons_per_bit']:14.1f} "
            f"{res['link_margin_db']:12.2f}"
        )

    print("================================================================")
    print("Link budget verification completed.")


if __name__ == "__main__":
    main()
