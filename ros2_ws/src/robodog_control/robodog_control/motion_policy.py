"""Startup, feedback and command preconditions shared by control entrypoints."""
import math


def startup_actions(backend: str, enable: str, stand: str) -> tuple[bool, bool]:
    simulation = backend in ("kinematic", "mujoco")

    def resolve(value):
        if value not in ("auto", "true", "false"):
            raise ValueError("startup flag must be auto, true or false")
        return simulation if value == "auto" else value == "true"

    return resolve(enable), resolve(stand)


def travel_feedback_ready(base) -> bool:
    # The real backend currently has no live IMU/base-state integration.
    # None must stay unavailable; fabricated gravity/zero gyro is not feedback.
    return base is not None


def joint_selection(names, requested):
    """Validate an enable/disable selection before any actuator is touched."""
    if not requested:
        return None
    unknown = sorted(set(requested) - set(names))
    if unknown:
        raise ValueError("unknown joints: " + ", ".join(unknown))
    return [name in requested for name in names]


def can_promote_cmd_vel(simulation: bool, enabled: bool, safety_latched: bool,
                        controller: str, active_pose: str, trajectory_active: bool) -> tuple[bool, str]:
    """Whether cmd_vel may turn a stable stand pose into the travel gait."""
    if safety_latched:
        return False, "e-stop latched, clear it first"
    if not enabled:
        return False, "cmd_vel ignored while joints are disabled"
    if not simulation:
        return False, "cmd_vel auto-travel is simulation-only; use an explicit gait after hardware feedback is live"
    if controller != "pose" or active_pose != "stand" or trajectory_active:
        return False, "cmd_vel ignored until the robot is stably standing"
    return True, ""


def bounded_body_velocity(vx: float, vy: float, wz: float, *,
                          max_linear: float, max_yaw: float) -> tuple[float, float, float]:
    values = (float(vx), float(vy), float(wz), float(max_linear), float(max_yaw))
    if not all(math.isfinite(v) for v in values) or max_linear < 0 or max_yaw < 0:
        raise ValueError("body velocity command and limits must be finite")
    return (_clip(values[0], max_linear), _clip(values[1], max_linear),
            _clip(values[2], max_yaw))


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
