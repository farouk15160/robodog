"""Reproducible sizing measurements at the backend and gait interfaces.

All efforts above the backend are joint-side. Thermal estimates use motor
output torque, before the external belt. Neither estimate validates cooling.
"""
from __future__ import annotations

import numpy as np


def operating_point_assessment(*, completed, stable, tracks_velocity,
                               within_continuous_rating,
                               safety_clamped_cycle_fraction,
                               max_clamped_cycle_fraction=.01):
    """Judge whether a run has usable margin, separately from basic stability."""
    reasons = []
    if not completed:
        reasons.append("run did not complete")
    if not stable:
        reasons.append("stability criteria failed")
    if not tracks_velocity:
        reasons.append("velocity tracking criteria failed")
    if not within_continuous_rating:
        reasons.append("motor RMS exceeded the continuous stall reference")
    if safety_clamped_cycle_fraction > max_clamped_cycle_fraction:
        reasons.append(
            f"safety limiting exceeded {100 * max_clamped_cycle_fraction:.1f}% of control cycles")
    return not reasons, reasons


def joint_metrics(*, names, torque, velocity, temperature, ratio, efficiency,
                  continuous, peak, kt, resistance, ambient, thermal_resistance):
    """Summarise signed time samples without mixing joint and motor ratings."""
    torque = np.asarray(torque, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    temperature = np.asarray(temperature, dtype=float)
    if (torque.ndim != 2 or torque.shape[0] == 0
            or torque.shape[1] != len(names)
            or velocity.shape != torque.shape or temperature.shape != torque.shape
            or not all(np.isfinite(x).all() for x in (torque, velocity, temperature))):
        raise ValueError("finite, nonempty joint samples with matching shapes required")
    motor = torque / (np.asarray(ratio) * np.asarray(efficiency))
    rms = np.sqrt(np.mean(motor ** 2, axis=0))
    steady = ambient + 3 * (rms / kt) ** 2 * resistance * thermal_resistance
    return [dict(
        name=name,
        joint_peak_nm=float(np.max(np.abs(torque[:, i]))),
        joint_rms_nm=float(np.sqrt(np.mean(torque[:, i] ** 2))),
        motor_peak_nm=float(np.max(np.abs(motor[:, i]))),
        motor_rms_nm=float(rms[i]),
        motor_peak_speed_rad_s=float(np.max(np.abs(velocity[:, i] * ratio[i]))),
        continuous_utilisation=float(rms[i] / continuous),
        peak_utilisation=float(np.max(np.abs(motor[:, i])) / peak),
        fraction_above_continuous=float(np.mean(np.abs(motor[:, i]) > continuous)),
        estimated_rms_current_a=float(rms[i] / kt),
        estimated_final_temperature_c=float(temperature[-1, i]),
        estimated_steady_temperature_c=float(steady[i]),
    ) for i, name in enumerate(names)]


def gait_command(output, gains):
    """Create a joint-side impedance command using the deployed gain config."""
    from robodog_hardware.types import ControlMode, JointCommand
    return JointCommand(
        mode=np.full(12, int(ControlMode.IMPEDANCE), dtype=np.uint8),
        position=output.q.copy(), velocity=output.qd.copy(), effort=output.tau_ff.copy(),
        kp=np.repeat(np.where(output.contact, gains["stance_kp"], gains["swing_kp"]), 3),
        kd=np.repeat(np.where(output.contact, gains["stance_kd"], gains["swing_kd"]), 3),
    )


def _enrich_physics_rows(rows, actuator, temperature, clamp_cycles, clamp_events, dt):
    electrical, thermal = actuator["electrical"], actuator["thermal"]
    continuous = actuator["operational_limits"]["continuous_torque_nm"]
    peak = actuator["operational_limits"]["peak_torque_nm"]
    kt = electrical["torque_constant_nm_per_arms"]
    return [{**row,
             "continuous_utilisation": row["motor_rms_nm"] / continuous,
             "peak_utilisation": row["motor_peak_nm"] / peak,
             "fraction_above_continuous": row["threshold_metrics"][f"{continuous:g}"]["total_time_s"] / row["duration_s"],
             "estimated_rms_current_a": row["motor_rms_nm"] / kt,
             "estimated_final_temperature_c": float(temperature[i]),
             "estimated_steady_temperature_c": thermal["ambient_temp_c"] + 3 *
                 (row["motor_rms_nm"] / kt) ** 2 * electrical["phase_resistance_ohm"] *
                 thermal["thermal_resistance_k_per_w"],
             "safety_torque_clipped_cycles": int(clamp_cycles[i]),
             "safety_torque_clip_events": int(clamp_events[i]),
             "safety_torque_clipped_duration_s": float(clamp_cycles[i] * dt),
             } for i, row in enumerate(rows)]


def run_case(params, actuator, gaits, controls, model_path, *, gait="stand",
             vx=0.0, vy=0.0, wz=0.0, seconds=8.0, warmup=1.0):
    """Measure actual applied torque at every physics substep, including impacts.

    Full-run maxima, threshold durations and clipping include startup and turns.
    Selected RMS excludes warmup; full-run RMS is also returned. A standing
    keyframe receives a constant body-velocity command at time zero. This is a
    simulation measurement, not a thermal or hardware qualification.
    """
    from robodog_control.balance import roll_pitch_from_quat, yaw_from_quat
    from robodog_control.gait import BodyFeedback, GaitGenerator, GaitParams
    from robodog_control.kinematics import LegGeometry
    from robodog_control.safety import SafetyLimits, SafetyMonitor
    from robodog_hardware.physics_metrics import PhysicsMetrics, body_planar_velocity
    from robodog_hardware.registry import create_backend
    from robodog_hardware.transmission import backend_config, transmission_arrays
    from robodog_hardware.types import JOINT_NAMES, Fault

    if not np.isfinite([seconds, warmup, vx, vy, wz]).all() or not 0 <= warmup < seconds:
        raise ValueError("require finite duration > warmup >= 0 and finite velocity")
    rate = float(controls["control_rate_hz"])
    if not np.isfinite(rate) or rate <= 0 or seconds * rate < 1:
        raise ValueError("positive control rate and at least one control cycle required")
    dt = 1 / rate
    backend = create_backend("mujoco", {
        **backend_config(actuator, JOINT_NAMES),
        "model_path": str(model_path), "keyframe": "stand",
    })
    backend.configure()
    try:
        backend.enable()
        ratio, efficiency = transmission_arrays(actuator, JOINT_NAMES)
        continuous = actuator["operational_limits"]["continuous_torque_nm"]
        peak = actuator["operational_limits"]["peak_torque_nm"]
        physics = PhysicsMetrics(
            JOINT_NAMES, ratio, efficiency,
            thresholds=tuple(sorted({8., 11., float(continuous), float(peak)})),
            warmup_s=warmup,
        )
        def observe_physics(torque, velocity, step_dt, **clipping):
            physics.update(
                torque, velocity, step_dt,
                world_position=backend.base_state().position,
                **clipping,
            )
        backend.set_physics_observer(observe_physics)
        pose = params["named_poses"]["stand"]
        gen = GaitGenerator(LegGeometry.from_params(params),
                            np.array([pose[k] for k in ("haa", "hfe", "kfe")]),
                            params["mass_budget"]["total_kg"])
        fields = {k: gaits[gait][k] for k in (
            "step_frequency_hz", "step_height_m", "duty_factor", "stance_height_m")}
        gen.set_params(GaitParams(gait=gait, vx=vx, vy=vy, wz=wz, **fields))
        safety = SafetyMonitor(SafetyLimits.from_config(params, actuator, list(JOINT_NAMES)),
                               list(JOINT_NAMES), rate)
        base, state = backend.base_state(), backend.read()
        last_temperature = state.temperature.copy()
        initial, last_position = base.position.copy(), base.position.copy()
        max_tilt, min_height, drift, faults, clamp_ticks, cycles = 0., float(initial[2]), 0., 0, 0, 0
        clamp_cycles, clamp_events, previous_clamp = np.zeros(12, int), np.zeros(12, int), np.zeros(12, bool)
        velocity_sum = np.zeros(3)
        strikes, previous_contact = np.zeros(4, int), base.foot_contact.copy()
        failure = None
        for _ in range(int(seconds * rate)):
            roll, pitch = roll_pitch_from_quat(base.orientation)
            feedback = BodyFeedback(
                height=float(base.position[2]), vz=float(base.linear_velocity[2]),
                roll=roll, pitch=pitch, yaw=yaw_from_quat(base.orientation),
                omega=tuple(base.angular_velocity),
                v_xy=tuple(body_planar_velocity(base.orientation, base.linear_velocity)))
            request = gait_command(gen.update(dt, feedback), controls["gains"])
            command, report = safety.apply(state, request)
            clamp_ticks += int(report.clamped > 0)
            clipped = (report.faults & int(Fault.TORQUE_LIMIT)) != 0
            clamp_cycles += clipped
            clamp_events += clipped & ~previous_clamp
            previous_clamp = clipped
            faults |= int(np.bitwise_or.reduce(report.faults))
            backend.write(command)
            before = float(backend.d.time)
            try:
                backend.step(dt)
                state, base = backend.read(), backend.base_state()
                if (not np.isfinite(np.r_[base.position, base.orientation, base.linear_velocity,
                                         base.angular_velocity, state.temperature]).all()
                        or backend.d.time <= before):
                    raise ValueError("nonfinite simulation state or reset simulation clock")
            except (ValueError, FloatingPointError) as error:
                failure = str(error)
                break
            cycles += 1
            last_temperature = state.temperature.copy()
            last_position = base.position.copy()
            tilt = float(np.degrees(2 * np.arcsin(np.clip(np.linalg.norm(base.orientation[:2]), 0, 1))))
            max_tilt, min_height = max(max_tilt, tilt), min(min_height, float(base.position[2]))
            drift = max(drift, abs(float(base.position[1] - initial[1])))
            velocity_sum += np.r_[body_planar_velocity(base.orientation, base.linear_velocity),
                                  base.angular_velocity[2]]
            strikes += base.foot_contact & ~previous_contact
            previous_contact = base.foot_contact.copy()
        duration = physics.elapsed_s
        metrics = _enrich_physics_rows(physics.summary(), actuator, last_temperature,
                                      clamp_cycles, clamp_events, dt) if physics.samples else []
        average_velocity = velocity_sum / max(cycles, 1)
        world_vx = float((last_position[0] - initial[0]) / max(duration, 1e-30))
        stable = failure is None and max_tilt < 15 and min_height > .25 and not safety.latched
        complete = failure is None and cycles == int(seconds * rate)
        rms_available = physics.rms_duration_s > 0
        clamped_fraction = clamp_ticks / max(cycles + int(failure is not None), 1)
        tracks_velocity = bool(
            complete and np.max(np.abs(average_velocity[:2] - [vx, vy])) < .07
            and abs(average_velocity[2] - wz) < .15)
        within_continuous = bool(
            complete and rms_available and metrics
            and max(j["continuous_utilisation"] for j in metrics) < 1)
        operating_point_pass, margin_failure_reasons = operating_point_assessment(
            completed=complete, stable=stable, tracks_velocity=tracks_velocity,
            within_continuous_rating=within_continuous,
            safety_clamped_cycle_fraction=clamped_fraction)
        return dict(
            gait=gait, commanded_vx_m_s=vx, commanded_vy_m_s=vy, commanded_wz_rad_s=wz,
            duration_s=duration, requested_duration_s=seconds, warmup_s=warmup,
            actual_vx_m_s=float(average_velocity[0]), actual_body_vx_m_s=float(average_velocity[0]),
            actual_vy_m_s=float(average_velocity[1]), world_average_vx_m_s=world_vx,
            actual_body_vy_m_s=float(average_velocity[1]), actual_wz_rad_s=float(average_velocity[2]),
            travel_m=float(last_position[0] - initial[0]), lateral_drift_m=drift,
            max_tilt_deg=max_tilt, min_height_m=min_height, final_height_m=float(last_position[2]),
            safety_enabled=True, safety_latched=safety.latched, safety_clamp_events=safety.clamp_events,
            safety_clamped_cycle_fraction=clamped_fraction, fault_flags=faults,
            stable=bool(stable), completed=complete, failure_reason=failure,
            stability_failure_reasons=(["excessive tilt"] if max_tilt >= 15 else []) +
                (["body height below limit"] if min_height <= .25 else []) +
                (["safety latched"] if safety.latched else []) + ([failure] if failure else []),
            tracks_velocity=tracks_velocity,
            within_continuous_rating=within_continuous,
            operating_point_pass=operating_point_pass,
            margin_failure_reasons=margin_failure_reasons,
            effort_source="MuJoCo qfrc_actuator at joint DOFs", physics_samples=physics.samples,
            physics_timestep_s=float(backend.m.opt.timestep),
            torque_velocity_pairing="Applied force with velocity at start of each implicitfast physics step",
            foot_strike_events_control_rate={leg: int(count) for leg, count in zip(("FL", "FR", "RL", "RR"), strikes)},
            joints=metrics,
        )
    finally:
        backend.shutdown()
