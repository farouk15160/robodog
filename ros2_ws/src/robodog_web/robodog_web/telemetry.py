"""Actuator references and bounded statistics for received browser telemetry."""
import math
from collections import deque

import numpy as np

from robodog_hardware.transmission import joint_limits, transmission_arrays


def joint_actuator_info(spec: dict, names: list[str]) -> dict:
    """Expose joint-side ratings and external belt gain without ROS dependencies."""
    ratios, efficiencies = transmission_arrays(spec, names)
    limits = joint_limits(spec, names)
    return {
        name: {
            "ratio": float(ratios[i]),
            "efficiency": float(efficiencies[i]),
            "torque_gain": float(ratios[i] * efficiencies[i]),
            "continuous_torque_nm": float(limits["continuous_torque_nm"][i]),
            "continuous_limit_basis": spec["performance"].get(
                "continuous_torque_basis", "configured limit"),
            "peak_torque_nm": float(limits["peak_torque_nm"][i]),
            "motor_continuous_torque_nm": float(spec["operational_limits"]["continuous_torque_nm"]),
            "motor_vendor_rotating_torque_nm": float(spec["performance"]["rated_torque_nm"]),
            "motor_peak_torque_nm": float(spec["operational_limits"]["peak_torque_nm"]),
        }
        for i, name in enumerate(names)
    }


# Statistics deliberately operate on received ROS telemetry. The browser cannot
# recover contact impacts between these samples (normally published at 50 Hz).
STATS_WINDOW_S = 20.0


def finite_json(value):
    """Keep invalid telemetry as JSON null instead of non-standard NaN/Infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    return value


class RollingTelemetry:
    """Bounded left-hold integration over a rolling observation window.

    Each received sample describes the interval until the next sample. Intervals
    longer than max_gap_s restart coverage, rather than inventing unobserved
    exposure. Small reversed stamps are discarded; a reversal greater than one
    second (or a clock change) starts a new timeline. Current samples contribute
    peaks immediately, but RMS needs at least two samples. The predecessor at a
    window boundary is retained only for its overlapping held interval.
    """

    def __init__(self, actuators: dict, window_s: float = STATS_WINDOW_S,
                 max_gap_s: float = 0.25, max_samples: int = 2048):
        if window_s <= 0 or max_gap_s <= 0 or max_samples < 2:
            raise ValueError("window, gap and sample capacity must be positive")
        self.window_s, self.max_gap_s = float(window_s), float(max_gap_s)
        self.names = tuple(actuators)
        self.gains = np.array([actuators[n]["torque_gain"] for n in self.names])
        self.ratios = np.array([actuators[n]["ratio"] for n in self.names])
        self.continuous = np.array([actuators[n]["motor_continuous_torque_nm"]
                                    for n in self.names])
        self.rotating = np.array([actuators[n]["motor_vendor_rotating_torque_nm"]
                                  for n in self.names])
        self._history = deque(maxlen=max_samples)
        self._clock = None
        self.resets = self.dropped_samples = self.gaps = 0

    def _accept(self, timestamp, clock, joints):
        by_name = {j["name"]: j for j in joints}
        try:
            values = np.array([[by_name[n][k] for k in ("eff", "vel", "temp", "err")]
                               for n in self.names], dtype=float)
            valid = (len(by_name) == len(self.names) == len(joints)
                     and math.isfinite(timestamp) and np.isfinite(values).all())
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.dropped_samples += 1
            self._history.clear()
            return
        if self._clock is not None and clock != self._clock:
            self.resets += 1
            self._history.clear()
        self._clock = clock
        if self._history:
            delta = timestamp - self._history[-1][0]
            if delta < -1.0:
                self.resets += 1
                self._history.clear()
            elif delta <= 0:
                self.dropped_samples += 1
                return
            elif delta > self.max_gap_s:
                self.gaps += 1
                self._history.clear()
        self._history.append((float(timestamp), values))
        cutoff = timestamp - self.window_s
        while len(self._history) > 1 and self._history[1][0] <= cutoff:
            self._history.popleft()

    def update(self, timestamp: float, clock: str, joints: list[dict]):
        """Return fresh enriched joint dictionaries and observation diagnostics."""
        self._accept(timestamp, clock, joints)
        stats, covered = self._statistics()
        enriched = []
        for j in joints:
            if j["name"] in self.names:
                i = self.names.index(j["name"])
                enriched.append({**j, "stats": stats[i],
                                 "motor_eff_nm": j["eff"] / self.gains[i],
                                 "motor_vel_rad_s": j["vel"] * self.ratios[i],
                                 "mechanical_power_w": j["eff"] * j["vel"]})
            else:
                enriched.append({**j, "stats": None})
        count = len(self._history)
        diagnostics = {
            "window_s": self.window_s, "covered_s": covered, "samples": count,
            "clock": self._clock or clock,
            "sample_rate_hz": (count - 1) / covered if covered else None,
            "resets": self.resets, "dropped_samples": self.dropped_samples,
            "gaps": self.gaps, "max_gap_s": self.max_gap_s,
            "stats_basis": "Received telemetry; sampled peaks, not physics-step peaks",
            "integration": "Time weighted, previous sample held until next valid sample",
            "mechanical_power_w": sum(j["eff"] * j["vel"] for j in joints),
        }
        return finite_json(enriched), finite_json(diagnostics)

    def _statistics(self):
        empty = {
            "window_s": self.window_s, "covered_s": 0.0, "samples": 0,
            **dict.fromkeys(("joint_torque_rms_nm", "joint_torque_peak_nm",
                            "motor_torque_rms_nm", "motor_torque_peak_nm",
                            "motor_speed_at_peak_rad_s", "above_continuous_s",
                            "above_vendor_rotating_s", "temperature_max_c",
                            "tracking_error_rms_rad", "mechanical_power_mean_w")),
        }
        if not self._history:
            return [dict(empty) for _ in self.names], 0.0
        times = np.array([sample[0] for sample in self._history])
        values = np.stack([sample[1] for sample in self._history])
        dt = np.diff(np.maximum(times, times[-1] - self.window_s))
        covered = float(dt.sum())
        torque, velocity = values[:, :, 0], values[:, :, 1]
        motor = torque / self.gains
        peak_indices = np.argmax(np.abs(motor), axis=0)
        result = []
        for i in range(len(self.names)):
            peak = peak_indices[i]
            def mean(data):
                return float(np.dot(dt, data[:-1]) / covered) if covered else None
            def rms(data):
                squared = mean(data ** 2)
                return math.sqrt(squared) if squared is not None else None
            result.append({
                "window_s": self.window_s, "covered_s": covered, "samples": len(times),
                "joint_torque_rms_nm": rms(torque[:, i]),
                "joint_torque_peak_nm": float(abs(torque[peak, i])),
                "motor_torque_rms_nm": rms(motor[:, i]),
                "motor_torque_peak_nm": float(abs(motor[peak, i])),
                "motor_speed_at_peak_rad_s": float(velocity[peak, i] * self.ratios[i]),
                "above_continuous_s": float(dt @ (abs(motor[:-1, i]) > self.continuous[i])) if covered else None,
                "above_vendor_rotating_s": float(dt @ (abs(motor[:-1, i]) > self.rotating[i])) if covered else None,
                "temperature_max_c": float(values[:, i, 2].max()),
                "tracking_error_rms_rad": rms(values[:, i, 3]),
                "mechanical_power_mean_w": mean(torque[:, i] * velocity[:, i]),
            })
        return result, covered
