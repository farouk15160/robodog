"""
First-order lumped winding-thermal model.

Used in two places:

  * simulation backends synthesise a plausible motor temperature, because the
    safety architecture must be exercised in simulation and MuJoCo has no
    concept of copper loss;
  * on the real robot the same model runs as an OBSERVER alongside the
    temperature the actuator reports. Sensor location and firmware thermal
    estimation vary between models; this uncalibrated observer may indicate
    a fast thermal excursion (a stalled leg pushing against an
    obstacle) before the reported temperature has moved.

    P_cu  = 3 * I_rms^2 * R_phase,   I_rms = tau / kt
    dT/dt = (P_cu - (T - T_ambient) / R_th) / C_th

Thermal impedance parameters are estimates from the active actuator YAML and
require instrumented identification within the manufacturer's stall/rotating
limits. This model omits per-phase stall hotspots, temperature-dependent
resistance and high-current torque nonlinearity; it cannot qualify cooling.
"""
from __future__ import annotations

import numpy as np


class ThermalModel:
    def __init__(self, n: int, *, torque_constant: float, phase_resistance: float,
                 r_th: float, c_th: float, ambient_c: float = 20.0) -> None:
        self.kt = float(torque_constant)
        self.r_phase = float(phase_resistance)
        self.r_th = float(r_th)
        self.c_th = float(c_th)
        self.ambient = float(ambient_c)
        self.temperature = np.full(n, float(ambient_c))

    def reset(self, ambient_c: float | None = None) -> None:
        if ambient_c is not None:
            self.ambient = float(ambient_c)
        self.temperature[:] = self.ambient

    def copper_loss(self, torque: np.ndarray) -> np.ndarray:
        """Estimated copper loss per motor [W]. Iron and switching losses are
        omitted; no measured loss map establishes their relative size. This is
        NOT conservative and
        the observer is an uncalibrated supplement to reported temperature,
        never evidence of safe winding temperature."""
        i_rms = np.abs(torque) / self.kt
        return 3.0 * i_rms ** 2 * self.r_phase

    def update(self, torque: np.ndarray, dt: float) -> np.ndarray:
        p = self.copper_loss(torque)
        dissipated = (self.temperature - self.ambient) / self.r_th
        self.temperature += (p - dissipated) * dt / self.c_th
        return self.temperature

    def steady_state(self, torque: float) -> float:
        """Temperature this torque would reach if held forever."""
        return self.ambient + 3.0 * (torque / self.kt) ** 2 * self.r_phase * self.r_th

    def time_to_limit(self, torque: np.ndarray, limit_c: float) -> np.ndarray:
        """Seconds until each joint reaches `limit_c` at constant torque; inf if
        the steady-state temperature is below the limit. Drives the 'time to
        thermal limit' readout in the GUI."""
        inf = np.full_like(self.temperature, np.inf)
        t_ss = self.ambient + self.copper_loss(torque) * self.r_th
        num = t_ss - limit_c
        den = t_ss - self.temperature
        with np.errstate(divide="ignore", invalid="ignore"):
            tau = self.r_th * self.c_th
            out = np.where((num > 0) & (den > 0), -tau * np.log(np.clip(num / den, 1e-12, None)), inf)
        return np.where(self.temperature >= limit_c, 0.0, out)
