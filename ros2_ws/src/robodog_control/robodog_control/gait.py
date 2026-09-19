"""
Gait generation: foot trajectories -> joint targets.

Everything is expressed as a foot position in each leg's HAA frame and passed
through the closed-form IK in kinematics.py, so a gait is defined by geometry
and timing, not by joint angles. Adding a gait is a block in gaits.yaml.

Phase model
-----------
A single normalised cycle phase advances at `step_frequency_hz`. Each leg has a
fixed offset; within its own cycle a leg is in STANCE for `duty_factor` of the
period and in SWING for the rest. The offsets are what distinguishes the gaits:

    stand   ----          all feet down, no motion
    walk    0, 0.5, 0.75, 0.25   duty 0.75, statically stable, one foot up
    trot    0, 0.5, 0.5, 0       duty 0.5,  diagonal pairs
    pace    0, 0.5, 0, 0.5       duty 0.5,  lateral pairs
    bound   0, 0, 0.5, 0.5       duty 0.5,  front pair then rear pair

Trajectory shapes
-----------------
Stance: the foot is planted, so relative to the body it translates backward at
exactly the commanded body velocity. Any deviation is the foot scuffing.

Swing: horizontal position follows s - sin(2*pi*s)/(2*pi), whose derivative
1 - cos(2*pi*s) is zero at both ends, so the foot lifts off and touches down
with no horizontal velocity relative to the ground. Height follows
sin^2(pi*s), which is likewise zero-slope at both ends. A small
`touchdown_depth_m` is subtracted near the end so the foot seeks the ground
instead of hovering over an uneven floor.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .balance import BalanceGains, BodyStabiliser
from .kinematics import (LEGS, LegGeometry, forward, gravity_torque, hip_origin,
                         inverse, jacobian, torque_for_force)

# leg -> (phase offset, ...) per gait
GAIT_OFFSETS = {
    "stand": (0.0, 0.0, 0.0, 0.0),
    "walk":  (0.00, 0.50, 0.75, 0.25),
    "trot":  (0.00, 0.50, 0.50, 0.00),
    "pace":  (0.00, 0.50, 0.00, 0.50),
    "bound": (0.00, 0.00, 0.50, 0.50),
}
DEFAULT_DUTY = {"stand": 1.0, "walk": 0.75, "trot": 0.5, "pace": 0.5, "bound": 0.5}


@dataclass
class GaitParams:
    gait: str = "stand"
    step_frequency_hz: float = 2.0
    step_height_m: float = 0.06
    stance_height_m: float = 0.32
    duty_factor: float = 0.5
    touchdown_depth_m: float = 0.005
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    # Cap on how far a foot may travel in one stride, as a fraction of the
    # nominal leg reach. Prevents a large velocity request from asking for a
    # stride the workspace cannot deliver.
    max_stride_fraction: float = 0.45
    # How the stance foot sweeps through the body frame:
    #   0.0 -> at the MEASURED body velocity: the foot stays planted in the
    #          world and all propulsion comes from the tangential ground force;
    #   1.0 -> at the COMMANDED velocity: the foot sweeps back regardless, and
    #          the impedance term drags the body forward.
    # Neither extreme works alone. At 0 there is no push-off and the feet march
    # out from under the body; at 1 the command walks away from a planted foot
    # and the stance legs fight the ground (measured: 0.17 rad of joint error,
    # 20 N.m at kp = 120). The blend is the usable region.
    stance_sweep_gain: float = 0.4


@dataclass
class BodyFeedback:
    """What the gait layer needs from the state estimator to balance.

    Optional: with no feedback the generator falls back to an open-loop
    m*g/n_stance force split, which is fine for a pinned base but will not walk
    on the ground -- see balance.py for the measurements.
    """
    height: float = 0.32
    vz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    omega: tuple[float, float, float] = (0.0, 0.0, 0.0)
    v_xy: tuple[float, float] = (0.0, 0.0)


@dataclass
class GaitOutput:
    q: np.ndarray = field(default_factory=lambda: np.zeros(12))
    qd: np.ndarray = field(default_factory=lambda: np.zeros(12))
    tau_ff: np.ndarray = field(default_factory=lambda: np.zeros(12))
    contact: np.ndarray = field(default_factory=lambda: np.ones(4, bool))
    foot_target: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    foot_force: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    phase: float = 0.0


def _smoothstep(s: float) -> tuple[float, float]:
    """Position and d/ds of the zero-endpoint-velocity horizontal profile."""
    return s - np.sin(2 * np.pi * s) / (2 * np.pi), 1.0 - np.cos(2 * np.pi * s)


class GaitGenerator:
    def __init__(self, geometry: LegGeometry, nominal_q: np.ndarray,
                 total_mass_kg: float = 10.0,
                 balance: BalanceGains | None = None) -> None:
        self.g = geometry
        self.mass = total_mass_kg
        self.params = GaitParams()
        self.stabiliser = BodyStabiliser(total_mass_kg, balance)
        self.phase = 0.0
        # Nominal foot position per leg in its HAA frame, taken from the stance
        # pose. The gait perturbs this; it never re-derives it.
        self.nominal = {leg: forward(geometry, leg, nominal_q) for leg in LEGS}
        self._last_q = np.concatenate([nominal_q] * 4)
        # Foot placement offset LATCHED per leg at touchdown. The offset must
        # not change while a foot is planted: the foot is on the ground, and
        # moving its commanded position under it is asking the leg to drag the
        # robot sideways through the contact. Without this latch the command
        # jumps by the full offset the instant the foot lands, which saturated
        # the actuators on every touchdown.
        self._offset = {leg: np.zeros(2) for leg in LEGS}
        self._was_stance = {leg: True for leg in LEGS}
        #: latched stance target per leg, in the hip frame
        self._stance_pos = {leg: self.nominal[leg].copy() for leg in LEGS}
        self._touchdown = {leg: self.nominal[leg].copy() for leg in LEGS}

    def set_params(self, p: GaitParams) -> None:
        if p.gait not in GAIT_OFFSETS:
            raise ValueError(f"unknown gait '{p.gait}'; have {sorted(GAIT_OFFSETS)}")
        if p.duty_factor <= 0.0:
            p.duty_factor = DEFAULT_DUTY[p.gait]
        self.params = p

    def reset(self) -> None:
        self.phase = 0.0
        for leg in LEGS:
            self._offset[leg][:] = 0.0
            self._was_stance[leg] = True
            self._stance_pos[leg] = self.nominal[leg].copy()

    # ---------------- per-cycle update ----------------
    def update(self, dt: float, feedback: BodyFeedback | None = None) -> GaitOutput:
        p = self.params
        out = GaitOutput()

        if p.gait == "stand":
            return self._static(p, feedback)

        self.phase = (self.phase + p.step_frequency_hz * dt) % 1.0
        out.phase = self.phase
        offsets = GAIT_OFFSETS[p.gait]
        period = 1.0 / max(p.step_frequency_hz, 1e-6)
        t_stance = period * p.duty_factor

        for i, leg in enumerate(LEGS):
            s_leg = (self.phase + offsets[i]) % 1.0
            nom = self.nominal[leg].copy()
            nom[2] = -(p.stance_height_m - self.g.foot_radius)

            # Stride vector for this foot: body velocity plus the tangential
            # component from the yaw rate at this foot's radius.
            r = np.array([nom[0] + self.g.haa_x * (1 if leg[0] == "F" else -1),
                          nom[1] + self.g.haa_y * (1 if leg[1] == "L" else -1)])
            v = np.array([p.vx - p.wz * r[1], p.vy + p.wz * r[0]])
            stride = v * t_stance
            max_stride = p.max_stride_fraction * self.g.reach_max
            n = float(np.linalg.norm(stride))
            if n > max_stride:
                stride *= max_stride / n

            # Raibert foot placement: a legged robot regulates velocity by
            # where it puts its feet, not by body attitude alone.
            raibert = np.zeros(2)
            if feedback is not None:
                raibert = self.stabiliser.foot_placement(
                    np.asarray(feedback.v_xy, float), v, t_stance)

            if s_leg < p.duty_factor:                      # ---- stance ----
                # A planted foot does not move in the WORLD, so in the body
                # frame it must translate at the MEASURED body velocity. Driving
                # it at the commanded velocity instead is what made the stance
                # legs fight the ground: the command walked away from the foot
                # at up to 0.17 rad of joint error, which at kp = 120 is 20 N.m
                # of pure disagreement with reality. Propulsion comes from the
                # tangential ground force in the wrench, not from dragging the
                # foot through the contact.
                if not self._was_stance[leg]:
                    self._offset[leg][:] = raibert     # latch on touchdown
                    self._was_stance[leg] = True
                    self._stance_pos[leg] = self._touchdown[leg].copy()
                if feedback is None:
                    v_body = v
                else:
                    v_meas = np.asarray(feedback.v_xy, float)
                    v_body = v_meas + p.stance_sweep_gain * (v[:2] - v_meas)
                sp_pos = self._stance_pos[leg]
                sp_pos[0] -= float(v_body[0]) * dt
                sp_pos[1] -= float(v_body[1]) * dt
                sp_pos[2] = nom[2]
                pos = sp_pos.copy()
                vel = np.array([-float(v_body[0]), -float(v_body[1]), 0.0])
                out.contact[i] = True
            else:                                          # ---- swing ----
                self._was_stance[leg] = False
                u = (s_leg - p.duty_factor) / max(1.0 - p.duty_factor, 1e-9)
                t_swing = period * (1.0 - p.duty_factor)
                sp, sv = _smoothstep(u)
                pos = nom + np.array([stride[0] * (sp - 0.5), stride[1] * (sp - 0.5), 0.0])
                # Blend from the offset this foot was standing on to the one it
                # will land with, so the path is continuous at BOTH lift-off
                # and touchdown.
                prev = self._offset[leg]
                pos[0] += prev[0] + (raibert[0] - prev[0]) * sp
                pos[1] += prev[1] + (raibert[1] - prev[1]) * sp
                pos[2] += p.step_height_m * np.sin(np.pi * u) ** 2
                pos[2] -= p.touchdown_depth_m * u ** 3     # seek the ground late in swing
                vel = np.array([stride[0] * sv / t_swing, stride[1] * sv / t_swing,
                                p.step_height_m * np.pi * np.sin(2 * np.pi * u) / (2 * t_swing)])
                out.contact[i] = False
                # remember where this foot is heading, so stance starts exactly
                # where swing ended
                self._touchdown[leg] = np.array([
                    nom[0] + stride[0] * 0.5 + raibert[0],
                    nom[1] + stride[1] * 0.5 + raibert[1], nom[2]])

            out.foot_target[i] = pos
            q = inverse(self.g, leg, pos)
            J = jacobian(self.g, leg, q)
            qd = np.linalg.lstsq(J, vel, rcond=1e-6)[0]
            out.q[3 * i:3 * i + 3] = q
            out.qd[3 * i:3 * i + 3] = qd

        self._apply_forces(out, p, feedback)
        self._last_q = out.q.copy()
        return out

    # ---------------- force distribution ----------------
    def _apply_forces(self, out: GaitOutput, p: GaitParams,
                      feedback: BodyFeedback | None) -> None:
        """Turn the desired body wrench into a feed-forward torque per joint.

        Without feedback this degrades to the open-loop m*g/n_stance split,
        which holds the weight but applies no moment -- so nothing corrects roll
        or pitch, and the robot does not walk. See balance.py.
        """
        n_stance = max(int(np.sum(out.contact)), 1)
        if feedback is None:
            out.foot_force[:] = 0.0
            out.foot_force[out.contact, 2] = self.mass * 9.81 / n_stance
        else:
            force, moment = self.stabiliser.wrench(
                feedback.height, p.stance_height_m, feedback.vz,
                feedback.roll, feedback.pitch, np.asarray(feedback.omega, float),
                np.asarray(feedback.v_xy, float), np.array([p.vx, p.vy]), p.wz)
            # foot positions relative to the body centre, which is where the
            # wrench is defined
            r = np.zeros((4, 3))
            for i, leg in enumerate(LEGS):
                r[i] = out.foot_target[i] + hip_origin(self.g, leg)
            out.foot_force[:] = self.stabiliser.distribute(r, out.contact, force, moment)
        for i, leg in enumerate(LEGS):
            if out.contact[i]:
                q = out.q[3 * i:3 * i + 3]
                out.tau_ff[3 * i:3 * i + 3] = torque_for_force(
                    self.g, leg, q, out.foot_force[i])

    def _static(self, p: GaitParams, feedback: BodyFeedback | None = None) -> GaitOutput:
        out = GaitOutput(phase=self.phase)
        for i, leg in enumerate(LEGS):
            nom = self.nominal[leg].copy()
            nom[2] = -(p.stance_height_m - self.g.foot_radius)
            out.foot_target[i] = nom
            out.q[3 * i:3 * i + 3] = inverse(self.g, leg, nom)
        out.contact[:] = True
        # Standing uses the same distribution as walking: with all four feet
        # down it also trims roll and pitch, so the robot stands level on a
        # slope instead of merely holding its joint angles.
        self._apply_forces(out, p, feedback)
        self._last_q = out.q.copy()
        return out


# --------------------------------------------------------------------------- #
# joint-level test motions
# --------------------------------------------------------------------------- #
@dataclass
class JointTestParams:
    """Open-loop joint exercises for bring-up and for checking a single
    actuator's response without involving the leg kinematics."""
    mode: str = "sine"            # 'sine' | 'sweep' | 'step' | 'sequential'
    joints: tuple[int, ...] = ()  # empty = all 12
    amplitude_rad: float = 0.20
    frequency_hz: float = 0.5
    f_start_hz: float = 0.2       # 'sweep' only
    f_end_hz: float = 3.0
    duration_s: float = 20.0
    dwell_s: float = 2.0          # 'sequential' and 'step'


class JointTestGenerator:
    """Superimposes a test signal on a base pose. Amplitude is clamped against
    the joint limits so a test can never drive a joint into its end stop."""

    def __init__(self, base_q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> None:
        self.base = np.asarray(base_q, float).copy()
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        self.params = JointTestParams()
        self.t = 0.0

    def reset(self) -> None:
        self.t = 0.0

    def update(self, dt: float) -> tuple[np.ndarray, np.ndarray, bool]:
        p = self.params
        self.t += dt
        sel = np.array(p.joints if p.joints else range(len(self.base)))
        amp = np.zeros_like(self.base)
        head = np.minimum(self.upper - self.base, self.base - self.lower)
        amp[sel] = np.minimum(p.amplitude_rad, np.maximum(head[sel], 0.0))

        q, qd = self.base.copy(), np.zeros_like(self.base)
        if p.mode == "sine":
            w = 2 * np.pi * p.frequency_hz
            q += amp * np.sin(w * self.t)
            qd = amp * w * np.cos(w * self.t)
        elif p.mode == "sweep":
            # linear chirp; instantaneous frequency f0 + (f1-f0)*t/T
            k = (p.f_end_hz - p.f_start_hz) / max(p.duration_s, 1e-6)
            phase = 2 * np.pi * (p.f_start_hz * self.t + 0.5 * k * self.t ** 2)
            w = 2 * np.pi * (p.f_start_hz + k * self.t)
            q += amp * np.sin(phase)
            qd = amp * w * np.cos(phase)
        elif p.mode == "step":
            q += amp * (1.0 if (self.t // p.dwell_s) % 2 else -1.0)
        elif p.mode == "sequential":
            # one joint at a time, so a wiring or sign error is unambiguous
            idx = int(self.t // p.dwell_s) % max(len(sel), 1)
            only = np.zeros_like(amp)
            only[sel[idx]] = amp[sel[idx]]
            w = 2 * np.pi * p.frequency_hz
            q += only * np.sin(w * self.t)
            qd = only * w * np.cos(w * self.t)
        else:
            raise ValueError(f"unknown joint-test mode '{p.mode}'")
        return np.clip(q, self.lower, self.upper), qd, self.t >= p.duration_s
