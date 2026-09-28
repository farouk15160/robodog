# RGB-D SLAM and OctoMap

Use **RTAB-Map SLAM with its built-in OctoMap** for this RGB-D robot. SLAM estimates the trajectory and corrects it when places are revisited; OctoMap stores occupied, free and unknown 3D space using those poses. Neither replaces the other. A separate OctoMap mapping server is unnecessary here: RTAB-Map can rebuild its occupancy representation from corrected graph poses. See [primary sources](mapping_sources.md).

## Start mapping

Install the ROS Humble packages once:

```bash
sudo apt-get install ros-humble-rtabmap-slam ros-humble-rtabmap-util ros-humble-rtabmap-sync ros-humble-octomap-server
```

The OctoMap server package supplies the export utility below; no separate mapping server is launched.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robodog_bringup robot.launch.py backend:=mujoco world:=house
```

Use `world:=flat` for the proving ground. Add `mujoco_viewer:=true` for the physics viewer. The default `mapping:=auto` enables RTAB-Map with MuJoCo, `camera_backend:=sim` and `use_camera:=true`. It leaves mapping disabled for kinematic or hardware backends and absent or unsupported cameras. Set `mapping:=none` to disable mapping and its processing load explicitly. The mapping RViz view shows the robot, 2D occupancy and occupied 3D voxels in the `map` frame.

Mapping mode uses registered 640 × 480 RGB and depth from the same simulated optical origin, with calibration derived from the rendered camera and a common acquisition timestamp. Depth remains 16-bit millimetres on ROS. Camera data is best effort; RTAB-Map uses matching QoS and exact image synchronization.

The initial configuration uses 5 cm occupancy cells, a 4 m depth range and a 2 Hz mapping rate. Cell size is map resolution, not a claim about sensor accuracy. Adjust `robodog_perception/config/rtabmap.yaml` after measuring the resulting load and map quality.

## Frames and outputs

Control supplies `odom → base_link`; RTAB-Map supplies only `map → odom`. The stack uses wall timestamps and does not publish `/clock`. The simulator supplies exact odometry: these runs exercise RGB-D mapping and the SLAM pipeline, but do not validate a real visual-odometry estimator.

| Topic | Type / purpose |
|---|---|
| `/robodog/mapping/map` | `nav_msgs/OccupancyGrid`, 2D occupancy |
| `/robodog/mapping/octomap_occupied_space` | `sensor_msgs/PointCloud2`, occupied 3D cells |
| `/robodog/mapping/octomap_full` | `octomap_msgs/Octomap`, full colored occupancy tree |
| `/robodog/mapping/octomap_binary` | `octomap_msgs/Octomap`, compact occupancy output |
| `/robodog/mapping/mapData` | `rtabmap_msgs/MapData`, graph and observations |

Map outputs are generated on demand when subscribed. RViz or the integration smoke test supplies those subscriptions. The map cache is retained when viewers disconnect.

## Preserve and export maps

The default database is `$ROS_HOME/robodog/maps/<world>.db`, or `~/.ros/robodog/maps/<world>.db` when `ROS_HOME` is unset. Restarting reuses it; nothing deletes it automatically. To start a separate mapping session, choose a new path:

```bash
ros2 launch robodog_bringup robot.launch.py backend:=mujoco world:=house mapping:=rtabmap mapping_database:=/absolute/path/house-session.db
```

Flush and back up the current database to its `.back` companion:

```bash
ros2 service call /robodog/mapping/rtabmap/backup std_srvs/srv/Empty '{}'
```

Export a full colored OctoMap (`.ot`) while the mapper is running:

```bash
ros2 run octomap_server octomap_saver_node --ros-args -p full:=true -p octomap_path:=/absolute/path/house.ot -r octomap_full:=/robodog/mapping/rtabmap/octomap_full
```

The service includes the `rtabmap` node name; the corresponding topic does not. Export a full `ColorOcTree`, because the saver does not support binary conversion of that tree type. The database retains the SLAM graph and observations; the `.ot` file is an occupancy snapshot.

## Hardware and navigation boundary

This integration currently supports MuJoCo simulation. The default `mapping:=auto` disables mapping on unsupported sensor pipelines; an explicit `mapping:=rtabmap` rejects them with an error. Kinematic mode has no moving base odometry or world spawn placement. The existing real NUWA backend does not yet deliver depth, and real base odometry is not integrated. Hardware needs calibrated registered depth and an independent odometry source, or a separately validated RGB-D odometry node with exclusive ownership of `odom → base_link`.

This adds mapping, not autonomous navigation, obstacle avoidance or a new locomotion controller. Featureless or repetitive scenes can prevent reliable visual loop closure even while depth occupancy updates succeed.

## Validation

Validated on 2026-09-28 with ROS Humble, MuJoCo 3.13.0, RTAB-Map ROS 0.23.7 and OctoMap server 2.3.1. Both runs used headless simulation, registered 640 × 480 images and 5 cm occupancy cells. [Recorded measurements](mapping_validation.json) include calibration, timestamps, TF checks and export checksums.

The initial mapping integration passed 338 regression tests, including 39 perception tests. After enabling mapping by default and adding GUI diagnostics, the full suite passed 364 tests; the mapping policy has 100% statement coverage. Camera tests include reconstructing the rendered floor using the published calibration. The later [GUI validation](gui_telemetry.md#verification) also confirmed default mapping startup in both worlds.

| World | Distance walked | Occupied 3D points, start → finish | Known 2D cells, start → finish | Final real-time factor |
|---|---:|---:|---:|---:|
| House | 0.500 m | 687 → 1,053 | 519 → 523 | 0.608 |
| Flat | 0.501 m | 1,837 → 4,163 | 3,607 → 4,262 | 0.617 |

Both short mapping runs produced new maps during motion, acknowledged database backup and finished with zero velocity and a stand command. Neither recorded actuator clamp events during those checks. Later, longer GUI walk-to-stand tests reported `TORQUE_LIMIT` in both worlds; a stand acknowledgement does not establish a sustained fault-free stance. See [the recorded limitation](gui_telemetry.md#verification). Flat-world occupied 3D points include the ground; its nearby 2D obstacle count was zero. The reported real-time factors mean these runs were slower than real time on this workstation.

The house database reloaded successfully: the known 2D cells remained at 523 before any new walking, and stored observations increased from 18 before restart to 24 afterward. Full colored OctoMap exports succeeded for both worlds: 7,964 tree nodes for the house and 27,567 for the flat world. Tree nodes and occupied point-cloud points count different things.

To repeat the bounded integration check against an already running simulation, use the same ROS domain and sourced workspace as its launch:

```bash
python3 tools/mapping_smoke.py --distance 0.5 --backup --output /tmp/mapping-check.json
```

This script commands motion; use it with the MuJoCo mapping stack. It verifies the camera contract, nonempty map outputs, updates during walking, expected TF publishers and edges, and stop/stand acknowledgement. Source review establishes TF edge ownership because Humble's Python message metadata does not expose publisher GIDs for live per-edge attribution. These short straight walks validate mapping integration and persistence, not visual loop closure, hardware localization accuracy or whole-world coverage.
