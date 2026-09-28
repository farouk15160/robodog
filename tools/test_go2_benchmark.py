"""Physical adapter checks against the pinned official Go2 model, when present."""
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
for package in ("robodog_control", "robodog_hardware", "robodog_sim"):
    sys.path.insert(0, str(ROOT / "ros2_ws/src" / package))

from go2_benchmark import Go2Adapter, load_model
from robodog_control.kinematics import forward_in_base

REPO = Path("/tmp/robodog-unitree-mujoco")


@pytest.fixture
def adapter():
    if not (REPO / "unitree_robots/go2/go2.xml").exists():
        pytest.skip("clone official unitree_mujoco at the benchmark revision first")
    return Go2Adapter(load_model(REPO))


def test_adapter_foot_targets_match_official_geometry(adapter):
    """Independent MuJoCo FK catches signs, ordering and the 2 mm toe offset."""
    data = mujoco.MjData(adapter.model)
    data.qpos[:7] = [0, 0, 0.32, 1, 0, 0, 0]
    for angles in ([0.0, 0.6, -1.2], [0.15, 0.8, -1.6], [-0.2, 1.0, -1.4]):
        target = np.tile(angles, 4)
        data.qpos[adapter.qpos] = adapter.to_official_position(target)
        mujoco.mj_forward(adapter.model, data)
        for index, leg in enumerate(adapter.legs):
            expected = forward_in_base(adapter.geometry, leg, target[3*index:3*index+3])
            actual = data.geom_xpos[adapter.feet[index]] - data.xpos[adapter.base]
            assert actual == pytest.approx(expected, abs=1e-9)


def test_torque_ordering_and_official_limits_reach_physics(adapter):
    data = mujoco.MjData(adapter.model)
    data.qpos[:7] = [0, 0, 1, 1, 0, 0, 0]
    requested = np.arange(1.0, 13.0)
    adapter.apply_torque(data, requested)
    mujoco.mj_forward(adapter.model, data)
    assert data.qfrc_actuator[adapter.dofs] == pytest.approx(requested)
    adapter.apply_torque(data, np.full(12, 100.0))
    mujoco.mj_forward(adapter.model, data)
    assert data.qfrc_actuator[adapter.dofs] == pytest.approx(np.tile([23.7, 23.7, 45.43], 4))
    assert np.all(data.qfrc_applied == 0) and np.all(data.xfrc_applied == 0)


def test_environment_matches_robodog_and_preserves_official_dynamics(adapter):
    model = adapter.model
    assert model.opt.timestep == .0005
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert model.geom_friction[adapter.feet] == pytest.approx(np.tile([1., .05, .002], (4, 1)))
    assert np.all(model.geom_margin[adapter.feet] == 0)
    assert np.all(model.geom_condim[adapter.feet] == 6)
    assert adapter.mass == pytest.approx(15.206408)
    assert model.dof_armature[adapter.dofs] == pytest.approx(np.full(12, .01))
    assert model.dof_damping[adapter.dofs] == pytest.approx(np.full(12, .1))
    assert model.dof_frictionloss[adapter.dofs] == pytest.approx(np.full(12, .2))


def test_effective_contact_uses_same_foot_solver_parameters(adapter):
    from robodog_control.kinematics import inverse
    model, geometry = adapter.model, adapter.geometry
    data = mujoco.MjData(model)
    nominal = inverse(geometry, 'FL', np.array([0., geometry.lateral, -(.320 - geometry.foot_radius)]))
    data.qpos[:7] = [0, 0, .319, 1, 0, 0, 0]
    data.qpos[adapter.qpos] = adapter.to_official_position(np.tile(nominal, 4))
    mujoco.mj_forward(model, data)
    contacts = [c for c in data.contact if c.geom1 in adapter.feet or c.geom2 in adapter.feet]
    assert len(contacts) == 4
    for contact in contacts:
        assert contact.dim == 6
        assert contact.friction == pytest.approx([1., 1., .05, .002, .002])
        assert contact.solref == pytest.approx([.006, 1.])
        assert contact.solimp[:3] == pytest.approx([.95, .99, .001])


def test_simulation_reset_is_rejected():
    from go2_benchmark import validate_step
    from types import SimpleNamespace
    data = SimpleNamespace(time=0., qpos=np.zeros(19), qvel=np.zeros(18))
    with pytest.raises(RuntimeError, match='reset'):
        validate_step(data, .1, .0005)
    data = SimpleNamespace(time=.1005, qpos=np.full(19, np.nan), qvel=np.zeros(18))
    with pytest.raises(RuntimeError, match='nonfinite'):
        validate_step(data, .1, .0005)


def test_short_physics_run_and_report(adapter, tmp_path):
    import json
    from go2_benchmark import run_case, write_report
    case = run_case(REPO, vx=.4, seconds=.1, warmup=.02)
    assert case['physics_samples'] == 200
    assert case['initial_height_m'] == .322
    assert len(case['joints']) == 12
    assert any(row['joint_peak_nm'] > 1. for row in case['joints'])
    report = dict(official_revision='test', mujoco_version=mujoco.__version__, cases=[case])
    path = tmp_path / 'report'
    write_report(path, report)
    assert len(json.loads(path.with_suffix('.json').read_text())['cases']) == 1
    assert 'Duration 0.1 s; RMS warmup 0.02 s' in path.with_suffix('.md').read_text()
    assert len(path.with_suffix('.csv').read_text().splitlines()) == 13
