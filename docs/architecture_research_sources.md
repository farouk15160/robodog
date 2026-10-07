# RoboDog architecture research source ledger

Checked on 2026-10-07 while expanding `docs/robodog_architecture.tex`.
This file records primary sources used by the architecture paper and local
files that support implementation-specific claims.

## Primary sources

- Raibert's legged locomotion work separates balance into speed, attitude and
  height/hopping regulation. RoboDog mirrors that split with foot placement,
  body-wrench regulation and joint impedance. Sources:
  <https://mitpress.mit.edu/9780262681193/legged-robots-that-balance/> and
  <https://publications.ri.cmu.edu/dynamically-stable-legged-locomotion-second-report-to-darpa-october-11981-december-311982>.
- Khatib's operational-space formulation is the primary source for task-space
  force control and Jacobian-transpose force-to-torque mapping. Source:
  <https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1985_ISIR.pdf>.
- Hogan's impedance-control paper is the primary source for interpreting a
  command as a dynamic relation between motion and force. Source:
  <https://doi.org/10.1115/1.3140702>.
- ROS 2 node, topic and QoS semantics were checked against official
  documentation source:
  <https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/About-Nodes.rst>,
  <https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/About-Topics.rst>,
  and
  <https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/topics/About-Quality-of-Service-Settings.rst>.
- `robot_state_publisher` behavior was checked against the upstream README.
  Source: <https://github.com/ros/robot_state_publisher>.
- Frame conventions come from REP-103 and REP-105:
  <https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst> and
  <https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst>.
- RTAB-Map and OctoMap behavior were checked against the RTAB-Map paper,
  RTAB-Map ROS source, and OctoMap project/paper pages:
  <https://arxiv.org/abs/2403.06341>,
  <https://github.com/introlab/rtabmap_ros>, and
  <https://octomap.github.io/>.
- `sensor_msgs/msg/PointCloud2` format was checked against the ROS 2 message
  definition:
  <https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg>.
- MuJoCo actuator effort interpretation comes from the official computation
  documentation, where actuator outputs are mapped through transmissions into
  `mjData.qfrc_actuator`: <https://mujoco.readthedocs.io/en/stable/computation/>.
- Unitree Go2 benchmark provenance comes from the official Unitree MuJoCo
  repository: <https://github.com/unitreerobotics/unitree_mujoco>.

## Repository evidence

- Top-level launch behavior, mapping defaults, GUI binding and backend
  selection: `ros2_ws/src/robodog_bringup/launch/robot.launch.py`.
- RTAB-Map launch composition and export service:
  `ros2_ws/src/robodog_perception/launch/mapping.launch.py` and
  `ros2_ws/src/robodog_perception/robodog_perception/map_export.py`.
- Gait timing, stance/swing trajectories, Raibert foot placement and balance
  handoff: `ros2_ws/src/robodog_control/robodog_control/gait.py`.
- Minimum-jerk named-pose interpolation:
  `ros2_ws/src/robodog_control/robodog_control/trajectory.py`.
- The 400 Hz control loop, aggregate telemetry, TF publication and command
  services: `ros2_ws/src/robodog_control/robodog_control/control_node.py`.
- External knee transmission conversion:
  `ros2_ws/src/robodog_hardware/robodog_hardware/transmission.py`.
- Physics-rate RMS/peak/threshold exposure and spike-location metrics:
  `tools/torque_report.py` and `tools/test_torque_report.py`.
- Current loaded-model output:
  `docs/rs06_28kg_1ms_100s.md`, `.json`, `.csv` and `.xml`.
- Manufacturer RS06 ratings, conflicts and project interpretation:
  `docs/rs06_datasheet_audit.md`.
