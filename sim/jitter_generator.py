#!/usr/bin/env python3
"""
TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
File: sim/jitter_generator.py
Description: Spacecraft micro-vibration PSD and high-rate angular jitter generator.
             Models reaction wheel assembly (RWA) static/dynamic unbalance harmonics,
             structural resonance poles, and time-domain colored noise injection.

Author: Aneesh Krishna Parthasarathy
License: ISC License
"""

import math
import random
from typing import Dict, List, Tuple


class JitterHarmonic:
    """Represents a discrete micro-vibration harmonic tone."""

    def __init__(self, frequency_hz: float, amplitude_urad: float, phase_rad: float = 0.0):
        self.freq = frequency_hz
        self.amp = amplitude_urad
        self.phase = phase_rad

    def sample(self, t_sec: float) -> float:
        """Sample tone at time t."""
        return self.amp * math.sin(2.0 * math.pi * self.freq * t_sec + self.phase)


class SpacecraftJitterModel:
    """
    Synthesizes realistic low-Earth orbit spacecraft micro-vibration environment.
    Combines low-frequency base motion (solar array, gravity gradient),
    wheel unbalance speed-dependent harmonics, and wideband structural resonance.
    """

    def __init__(
        self,
        sample_rate_hz: float = 1000.0,
        rwa_speed_rpm: float = 3600.0,
        static_unbalance_gcm: float = 2.5,
        dynamic_unbalance_gcm2: float = 15.0,
    ):
        self.fs = sample_rate_hz
        self.dt = 1.0 / self.fs
        self.rwa_rpm = rwa_speed_rpm
        self.f0 = self.rwa_rpm / 60.0 # Fundamental spin frequency [Hz]

        # Initialize discrete RWA harmonics
        # 1x spin, 2x elliptic, 2.8x cage, 5.2x bearing track
        self.tones: List[JitterHarmonic] = [
            JitterHarmonic(frequency_hz=self.f0 * 1.0, amplitude_urad=1.8),
            JitterHarmonic(frequency_hz=self.f0 * 2.0, amplitude_urad=0.9),
            JitterHarmonic(frequency_hz=self.f0 * 2.8, amplitude_urad=0.6),
            JitterHarmonic(frequency_hz=self.f0 * 5.2, amplitude_urad=0.4),
            # Structural resonance mode at 120 Hz (solar panel torsion)
            JitterHarmonic(frequency_hz=120.0, amplitude_urad=0.75),
            # Optical bench mount resonance mode at 350 Hz
            JitterHarmonic(frequency_hz=350.0, amplitude_urad=0.35),
        ]

        # Random seeds for phase randomization
        for tone in self.tones:
            tone.phase = random.uniform(0.0, 2.0 * math.pi)

    def generate_jitter_sample(self, t_sec: float, broad_noise_std_urad: float = 0.20) -> Tuple[float, float]:
        """
        Generate two-axis (Azimuth, Elevation) jitter sample in microradians.
        Returns (az_urad, el_urad).
        """
        az_val = 0.0
        el_val = 0.0

        for tone in self.tones:
            az_val += tone.sample(t_sec)
            # Orthogonal axis has 90 deg phase lag and slight amplitude scaling
            el_val += tone.amp * 0.95 * math.cos(2.0 * math.pi * tone.freq * t_sec + tone.phase)

        # Add broadband Gaussian white noise (sensor shot noise + thermal baseline)
        az_val += random.gauss(0.0, broad_noise_std_urad)
        el_val += random.gauss(0.0, broad_noise_std_urad)

        return az_val, el_val

    def generate_time_history(self, duration_sec: float) -> Dict[str, List[float]]:
        """Synthesize continuous jitter time history."""
        num_samples = int(duration_sec * self.fs)
        t_arr: List[float] = []
        az_arr: List[float] = []
        el_arr: List[float] = []
        rad_arr: List[float] = []

        for i in range(num_samples):
            t = i * self.dt
            az, el = self.generate_jitter_sample(t)
            r = math.sqrt(az**2 + el**2)

            t_arr.append(t)
            az_arr.append(az)
            el_arr.append(el)
            rad_arr.append(r)

        return {
            "time_sec": t_arr,
            "az_urad": az_arr,
            "el_urad": el_arr,
            "radial_urad": rad_arr,
        }

    def compute_rms_jitter(self, series: List[float]) -> float:
        """Calculate Root-Mean-Square (RMS) jitter amplitude."""
        if not series:
            return 0.0
        mean_val = sum(series) / len(series)
        var = sum((x - mean_val) ** 2 for x in series) / len(series)
        return math.sqrt(var)


def main() -> None:
    """Synthesize 1 second of micro-vibration and evaluate statistical metrics."""
    print("================================================================")
    print("TRUNNION: Spacecraft Micro-Vibration & Jitter Simulation")
    print("RWA Spin: 3600 RPM (f0 = 60 Hz), Sample Rate: 1000 Hz")
    print("================================================================")

    random.seed(42) # Deterministic for verification
    model = SpacecraftJitterModel(sample_rate_hz=1000.0, rwa_speed_rpm=3600.0)

    print("Active Micro-Vibration Discrete Harmonics:")
    for tone in model.tones:
        print(f"  Tone: {tone.freq:6.1f} Hz  |  Peak Amplitude: {tone.amp:5.2f} urad")

    print("\nSynthesizing 1.0 second (1,000 samples) of micro-vibration...")
    hist = model.generate_time_history(duration_sec=1.0)

    az_rms = model.compute_rms_jitter(hist["az_urad"])
    el_rms = model.compute_rms_jitter(hist["el_urad"])
    rad_rms = model.compute_rms_jitter(hist["radial_urad"])
    rad_peak = max(hist["radial_urad"])

    print(f"Azimuth RMS Jitter:    {az_rms:6.3f} urad")
    print(f"Elevation RMS Jitter:  {el_rms:6.3f} urad")
    print(f"Radial 1-Sigma Jitter: {rad_rms:6.3f} urad")
    print(f"Radial 3-Sigma Peak:   {rad_peak:6.3f} urad")

    # FSM closed-loop rejection assessment (assuming 15 dB loop gain at 60 Hz)
    fsm_rejection_factor = 10.0 ** (-15.0 / 20.0) # approx 0.178
    residual_rms = rad_rms * fsm_rejection_factor
    print(f"FSM Closed-Loop Residual: {residual_rms:6.3f} urad (Target < 0.500 urad)")
    if residual_rms < 0.50:
        print("Status: PASS (Residual meets sub-microradian optical link requirement)")
    else:
        print("Status: WARN (Requires increased loop bandwidth)")

    print("================================================================")


if __name__ == "__main__":
    main()
