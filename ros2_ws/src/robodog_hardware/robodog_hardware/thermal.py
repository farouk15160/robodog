"""
First-order lumped winding-thermal model.

Used in two places:

  * simulation backends synthesise a plausible motor temperature, because the
    safety architecture must be exercised in simulation and MuJoCo has no
    concept of copper loss;
  * on the real robot the same model runs as an OBSERVER alongside the
    temperature the RS02 reports. The motor's own sensor sits on the driver
    board and lags the winding by tens of seconds, so the observer is what
    catches a fast thermal excursion (a stalled leg pushing against an
    obstacle) before the reported temperature has moved.

    P_cu  = 3 * I_rms^2 * R_phase,   I_rms = tau / kt
    dT/dt = (P_cu - (T - T_ambient) / R_th) / C_th

Parameters are estimates from robstride02.yaml -> thermal and MUST be
calibrated: run a joint at rated torque against a stop and fit the step
response. Until then the limits in safety.yaml carry the margin.
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
        """Copper loss per joint [W]. Iron loss is ignored: below the rated
        speed of 10.5 rad/s it is a few percent of copper loss, and ignoring it
        is conservative in the direction that matters (it under-predicts
        heating at high speed, which is why the observer is a supplement to the
        reported temperature and not a replacement for it)."""
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
