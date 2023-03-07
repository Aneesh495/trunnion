#!/usr/bin/env python3
"""
TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
File: tools/dashboard.py
Description: Real-time ANSI terminal engineering telemetry dashboard.
             Displays Pointing, Acquisition and Tracking (PAT) finite state machine,
             gimbal encoders, FSM piezo deflections, optical power, and link margin.

Author: Aneesh Krishna Parthasarathy
License: ISC License
"""

import sys
import time
import math
from typing import Dict, Any, Optional

# ANSI Color Codes
CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_CYAN = "\033[36m"
CLR_GREEN = "\033[32m"
CLR_YELLOW = "\033[33m"
CLR_RED = "\033[31m"
CLR_MAGENTA = "\033[35m"
CLR_BLUE = "\033[34m"
CLR_WHITE = "\033[37m"
CLR_BG_BLUE = "\033[44m"


def progress_bar(val: float, max_val: float, width: int = 20, unit: str = "") -> str:
    """Generate color-coded progress bar."""
    ratio = max(0.0, min(1.0, val / max_val))
    filled = int(ratio * width)
    bar = "=" * filled + "-" * (width - filled)
    if ratio > 0.8:
        color = CLR_GREEN
    elif ratio > 0.4:
        color = CLR_YELLOW
    else:
        color = CLR_RED
    return f"[{color}{bar}{CLR_RESET}] {val:5.1f}/{max_val:5.1f} {unit}"


def render_dashboard(data: Dict[str, Any]) -> str:
    """Render full formatted telemetry console screen."""
    lines = []
    w = 78

    # Title Banner
    lines.append(f"{CLR_BOLD}{CLR_CYAN}+{'=' * (w - 2)}+{CLR_RESET}")
    title = "TRUNNION: SATELLITE OPTICAL CROSS-LINK FLIGHT TELEMETRY"
    lines.append(f"{CLR_BOLD}{CLR_CYAN}| {title:^{w - 4}} |{CLR_RESET}")
    lines.append(f"{CLR_BOLD}{CLR_CYAN}+{'=' * (w - 2)}+{CLR_RESET}")

    # Subsystem Overview
    met = data.get("met_sec", 142.50)
    state = data.get("pat_state", "COMM_LOCK")
    state_color = CLR_GREEN if state in ("TRACK", "COMM_LOCK") else CLR_YELLOW
    fdir = data.get("fdir_severity", "NOMINAL (INFO)")
    fdir_color = CLR_GREEN if "NOMINAL" in fdir else CLR_RED

    lines.append(f"  {CLR_BOLD}Mission Elapsed Time:{CLR_RESET} {met:10.2f} s   "
                 f"{CLR_BOLD}PAT FSM State:{CLR_RESET} [{state_color}{state:^10}{CLR_RESET}]   "
                 f"{CLR_BOLD}FDIR Health:{CLR_RESET} {fdir_color}{fdir}{CLR_RESET}")
    lines.append(f"  {'-' * (w - 4)}")

    # CPA Gimbal Subsystem
    lines.append(f"  {CLR_BOLD}[1] COARSE POINTING ASSEMBLY (CPA GIMBAL)]{CLR_RESET}")
    az_deg = data.get("cpa_az_deg", 45.230)
    el_deg = data.get("cpa_el_deg", 12.180)
    az_rate = data.get("cpa_az_rate_dps", 0.052)
    el_rate = data.get("cpa_el_rate_dps", -0.014)
    az_temp = data.get("cpa_az_temp_c", 22.4)
    el_temp = data.get("cpa_el_temp_c", 21.8)

    lines.append(f"    Azimuth:    {az_deg:8.3f} deg  | Rate: {az_rate:7.4f} deg/s | Stator Temp: {az_temp:5.1f} C")
    lines.append(f"    Elevation:  {el_deg:8.3f} deg  | Rate: {el_rate:7.4f} deg/s | Stator Temp: {el_temp:5.1f} C")
    lines.append(f"    Friction Compensation: LuGre Observer Active (Bristle Stiff: 1.2e5 N*m/rad)")

    # FSM Fine Steering Mirror
    lines.append(f"\n  {CLR_BOLD}[2] FINE STEERING MIRROR (FSM PIEZO ACTUATION)]{CLR_RESET}")
    tip_urad = data.get("fsm_tip_urad", -1.45)
    tilt_urad = data.get("fsm_tilt_urad", 0.88)
    rms_urad = data.get("fsm_rms_urad", 0.142)
    offload = data.get("fsm_offload_pct", 18.5)

    lines.append(f"    Deflection: Tip = {tip_urad:+6.2f} urad, Tilt = {tilt_urad:+6.2f} urad  | Residual RMS: {CLR_GREEN}{rms_urad:5.3f} urad{CLR_RESET}")
    lines.append(f"    Piezo Rail: Z1 = {data.get('piezo_z1_v', 45.2):5.1f} V, Z2 = {data.get('piezo_z2_v', 52.8):5.1f} V, Z3 = {data.get('piezo_z3_v', 48.0):5.1f} V")
    lines.append(f"    CPA Offload Accumulator: {progress_bar(offload, 100.0, 18, '%')}")

    # Optical Detector & Centroiding
    lines.append(f"\n  {CLR_BOLD}[3] OPTICAL RECEIVER & BEAM CENTROIDING]{CLR_RESET}")
    qpd_sum = data.get("qpd_sum", 48250)
    snr_db = data.get("snr_db", 24.6)
    spot_x = data.get("spot_x_um", -0.36)
    spot_y = data.get("spot_y_um", 0.22)

    lines.append(f"    Quadrant Photodiode: Sum = {qpd_sum:5d} counts  | SNR: {CLR_GREEN}{snr_db:5.1f} dB{CLR_RESET} | Extinction: 14.8 dB")
    lines.append(f"    NFOV Beam Spot CoG:  X = {spot_x:+6.2f} um, Y = {spot_y:+6.2f} um (Diffraction Limit: 12.3 um)")
    lines.append(f"    Nutation Tracker:    Conical Scan Synchronized (Demod Phase: 42.1 deg)")

    # CCSDS Optical Communications Subsystem
    lines.append(f"\n  {CLR_BOLD}[4] CCSDS OPTICAL COMMUNICATIONS SUBSYSTEM]{CLR_RESET}")
    link_margin = data.get("link_margin_db", 53.7)
    ber = data.get("ber", 1.2e-9)
    rs_frames = data.get("rs_frames", 1420)
    rs_corrected = data.get("rs_corrected", 14)

    lines.append(f"    Modulation:  16-PPM @ 100.0 Mbps  | CCSDS Sync Marker (0x1ACFFC1D): {CLR_GREEN}LOCKED{CLR_RESET}")
    lines.append(f"    Codec:       Reed-Solomon RS(255, 223) + Depth-4 Block Interleaver")
    lines.append(f"    Frames:      {rs_frames:6d} Decoded, {rs_corrected:3d} Corrected | Channel BER: {ber:1.1e}")
    lines.append(f"    Link Margin: {progress_bar(link_margin, 60.0, 24, 'dB')} (Threshold: +3.0 dB)")

    # Thermal & Power Management
    lines.append(f"\n  {CLR_BOLD}[5] THERMAL & LASER SAFETY STATUS]{CLR_RESET}")
    laser_temp = data.get("laser_temp_c", 24.8)
    bench_temp = data.get("bench_temp_c", 25.1)
    heater_pwm = data.get("heater_pwm_pct", 32)
    tx_power_w = data.get("tx_power_w", 2.05)
    shutter = "OPEN" if data.get("shutter_open", True) else "CLOSED"

    lines.append(f"    Laser Submount: {laser_temp:5.1f} C (Limit 50.0 C) | Bench: {bench_temp:5.1f} C (Target 25.0 C)")
    lines.append(f"    Thermal PID:    Heater Output = {heater_pwm:2d}% PWM    | Laser Shutter: {CLR_GREEN}{shutter}{CLR_RESET}")
    lines.append(f"    Emission Power: {tx_power_w:5.2f} W @ 1550 nm       | Watchdog: {CLR_GREEN}PET NOMINAL{CLR_RESET}")

    lines.append(f"{CLR_BOLD}{CLR_CYAN}+{'=' * (w - 2)}+{CLR_RESET}")
    return "\n".join(lines)


def main() -> None:
    """Render telemetry dashboard."""
    sample_data = {
        "met_sec": 142.50,
        "pat_state": "COMM_LOCK",
        "fdir_severity": "NOMINAL (INFO)",
        "cpa_az_deg": 45.230,
        "cpa_el_deg": 12.180,
        "cpa_az_rate_dps": 0.052,
        "cpa_el_rate_dps": -0.014,
        "cpa_az_temp_c": 22.4,
        "cpa_el_temp_c": 21.8,
        "fsm_tip_urad": -1.45,
        "fsm_tilt_urad": 0.88,
        "fsm_rms_urad": 0.142,
        "fsm_offload_pct": 18.5,
        "piezo_z1_v": 45.2,
        "piezo_z2_v": 52.8,
        "piezo_z3_v": 48.0,
        "qpd_sum": 48250,
        "snr_db": 24.6,
        "spot_x_um": -0.36,
        "spot_y_um": 0.22,
        "link_margin_db": 53.7,
        "ber": 1.2e-9,
        "rs_frames": 1420,
        "rs_corrected": 14,
        "laser_temp_c": 24.8,
        "bench_temp_c": 25.1,
        "heater_pwm_pct": 32,
        "tx_power_w": 2.05,
        "shutter_open": True,
    }

    if "--live" in sys.argv:
        print("\033[2J\033[H", end="")
        for i in range(10):
            sample_data["met_sec"] += 0.5
            sample_data["cpa_az_deg"] += 0.01
            sample_data["fsm_rms_urad"] = 0.140 + 0.02 * math.sin(i)
            print("\033[H" + render_dashboard(sample_data))
            time.sleep(0.5)
    else:
        print(render_dashboard(sample_data))


if __name__ == "__main__":
    main()
