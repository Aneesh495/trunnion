\ ==============================================================================
\ TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
\ File: trunnion.fs
\ Description: Master Flight Software Root System Loader & Bootstrap Entry
\ Author: Aneesh Krishna Parthasarathy
\ License: ISC License
\ ==============================================================================

\ ------------------------------------------------------------------------------
\ Section 1: Core Mathematical & Algorithmic Foundations
\ ------------------------------------------------------------------------------
INCLUDE forth/core/preamble.fs
INCLUDE forth/core/double.fs
INCLUDE forth/core/fixed.fs
INCLUDE forth/core/cordic.fs
INCLUDE forth/core/vector3.fs
INCLUDE forth/core/matrix3.fs
INCLUDE forth/core/quaternion.fs
INCLUDE forth/core/ringbuf.fs
INCLUDE forth/core/bitops.fs
INCLUDE forth/core/string_util.fs

\ ------------------------------------------------------------------------------
\ Section 2: Hardware Abstraction Layer & Peripheral Registers
\ ------------------------------------------------------------------------------
INCLUDE forth/hal/memory_map.fs
INCLUDE forth/hal/cpa_registers.fs
INCLUDE forth/hal/fsm_registers.fs
INCLUDE forth/hal/qpd_registers.fs
INCLUDE forth/hal/laser_registers.fs
INCLUDE forth/hal/comm_registers.fs
INCLUDE forth/hal/timer_registers.fs

\ ------------------------------------------------------------------------------
\ Section 3: Mechanism Kinematics, Dynamics & Servos
\ ------------------------------------------------------------------------------
INCLUDE forth/kinematics/cpa_kinematics.fs
INCLUDE forth/kinematics/cpa_trajectory.fs
INCLUDE forth/kinematics/cpa_friction.fs
INCLUDE forth/kinematics/cpa_servo.fs
INCLUDE forth/kinematics/fsm_kinematics.fs
INCLUDE forth/kinematics/fsm_filters.fs
INCLUDE forth/kinematics/fsm_servo.fs
INCLUDE forth/kinematics/offload.fs

\ ------------------------------------------------------------------------------
\ Section 4: Optical Processing, Detectors & Centroiding
\ ------------------------------------------------------------------------------
INCLUDE forth/optics/qpd_centroid.fs
INCLUDE forth/optics/nfov_detector.fs
INCLUDE forth/optics/nutation.fs
INCLUDE forth/optics/snr_estimator.fs
INCLUDE forth/optics/calib_matrix.fs

\ ------------------------------------------------------------------------------
\ Section 5: Pointing, Acquisition, and Tracking (PAT) Engine
\ ------------------------------------------------------------------------------
INCLUDE forth/pat/spiral_search.fs
INCLUDE forth/pat/lissajous_search.fs
INCLUDE forth/pat/acquisition_fsm.fs
INCLUDE forth/pat/point_ahead.fs
INCLUDE forth/pat/tracking_loop.fs
INCLUDE forth/pat/loss_of_lock.fs
INCLUDE forth/pat/state_machine.fs

\ ------------------------------------------------------------------------------
\ Section 6: Orbital Mechanics & Ephemeris Geometry
\ ------------------------------------------------------------------------------
INCLUDE forth/orbit/constants.fs
INCLUDE forth/orbit/kepler.fs
INCLUDE forth/orbit/j2_perturbation.fs
INCLUDE forth/orbit/los_geometry.fs
INCLUDE forth/orbit/occultation.fs
INCLUDE forth/orbit/ephemeris_table.fs

\ ------------------------------------------------------------------------------
\ Section 7: Optical Communications Subsystem (CCSDS)
\ ------------------------------------------------------------------------------
INCLUDE forth/comm/ppm_modem.fs
INCLUDE forth/comm/sync_marker.fs
INCLUDE forth/comm/reed_solomon.fs
INCLUDE forth/comm/interleaver.fs
INCLUDE forth/comm/space_packet.fs
INCLUDE forth/comm/framing.fs
INCLUDE forth/comm/ber_monitor.fs

\ ------------------------------------------------------------------------------
\ Section 8: Fault Detection, Isolation, and Recovery (FDIR)
\ ------------------------------------------------------------------------------
INCLUDE forth/fdir/watchdog.fs
INCLUDE forth/fdir/thermal_monitor.fs
INCLUDE forth/fdir/optical_safety.fs
INCLUDE forth/fdir/fault_manager.fs

\ ------------------------------------------------------------------------------
\ Section 9: Real-Time Kernel & Multi-Rate Executive
\ ------------------------------------------------------------------------------
INCLUDE forth/kernel/task.fs
INCLUDE forth/kernel/event_flags.fs
INCLUDE forth/kernel/msg_queue.fs
INCLUDE forth/kernel/telemetry.fs
INCLUDE forth/kernel/executive.fs

\ ------------------------------------------------------------------------------
\ Section 10: Interactive Shell & Diagnostics
\ ------------------------------------------------------------------------------
INCLUDE forth/shell/cli_words.fs
INCLUDE forth/shell/diag_dump.fs
INCLUDE forth/shell/console.fs

\ ------------------------------------------------------------------------------
\ Section 11: System Boot & Initialization Sequence
\ ------------------------------------------------------------------------------

\ TRUNNION-BOOT ( -- )
\ Master flight software boot routine.
: TRUNNION-BOOT ( -- )
    EXEC-INIT
    CONSOLE-WELCOME
;

\ Execute boot routine
TRUNNION-BOOT
