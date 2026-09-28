"""Exercise the upstream viewer + SDK runner on loopback; no robot commands."""
import argparse
import os
from pathlib import Path
import runpy
import sys
import threading
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path('/tmp/robodog-unitree-mujoco'))
args = parser.parse_args()
source = args.repo.resolve() / 'simulate_python'
sys.path.insert(0, str(source))
os.chdir(source)
import config
config.INTERFACE = 'lo'
config.DOMAIN_ID = 42
config.USE_JOYSTICK = False
config.PRINT_SCENE_INFORMATION = False
config.ENABLE_ELASTIC_BAND = False
import mujoco.viewer
original = mujoco.viewer.launch_passive
handles = []
def bounded_viewer(*args, **kwargs):
    viewer = original(*args, **kwargs)
    handles.append(viewer)
    threading.Timer(5.0, viewer.close).start()
    return viewer
mujoco.viewer.launch_passive = bounded_viewer
import unitree_sdk2py_bridge
bridges = []
base_bridge = unitree_sdk2py_bridge.UnitreeSdk2Bridge
class CapturedBridge(base_bridge):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        bridges.append(self)
unitree_sdk2py_bridge.UnitreeSdk2Bridge = CapturedBridge
namespace = runpy.run_path(str(source / 'unitree_mujoco.py'), run_name='__main__')
namespace['sim_thread'].join(timeout=10.)
namespace['viewer_thread'].join(timeout=10.)
for bridge in bridges:
    for name in ('lowStateThread', 'HighStateThread', 'WirelessControllerThread'):
        getattr(bridge, name).Wait(2.)
    for name in ('low_state_puber', 'high_state_puber', 'wireless_controller_puber', 'low_cmd_suber'):
        getattr(bridge, name).Close()
steps = namespace['mj_data'].time / namespace['mj_model'].opt.timestep
assert handles and steps > 50, steps
print(f'OFFICIAL_RUNNER_OK simulation_time={namespace["mj_data"].time:.3f} steps={steps:.0f}', flush=True)
