Bring-up overrides go here. Package defaults live with the package that owns
them, so that a parameter has exactly one authoritative source:

    robodog_description/config/robot_parameters.yaml   geometry, inertia (generated)
    robodog_description/config/robstride06.yaml        actuator data and limits
    robodog_control/config/control.yaml                rates and impedance gains
    robodog_control/config/gaits.yaml                  gait library
    robodog_hardware/config/robstride_bus.yaml         CAN topology, motor map
    robodog_perception/config/nuwa_hp60c.yaml          camera intrinsics, range
    robodog_perception/config/rtabmap.yaml             optional RGB-D SLAM and OctoMap
    robodog_sim/config/simulation.yaml                 physics fidelity
    robodog_web/config/web.yaml                        GUI panels and limits

`robot_parameters.yaml` selects the actuator file through `actuator_config`.
The current RS06 configuration includes the external knee ratio and efficiency.

Startup flags `auto_enable:=auto auto_stand:=auto` enable and stand simulation,
but leave real CAN hardware disabled. The RS06 backend refuses normal enable or
motion on joints that have not been marked `calibrated: true` after physical
commissioning. Real IMU/base feedback is not integrated: hardware travel commands
are rejected while base feedback is unavailable.

The GUI binds localhost and validates WebSocket origins by default. Changing
network exposure is a separate deployment decision, not a requirement for local
simulation use.

`mapping:=rtabmap` enables RTAB-Map RGB-D SLAM and its graph-corrected OctoMap.
Use `backend:=mujoco`, `camera_backend:=sim`, and
`use_camera:=true`. Mapping selects calibrated, registered 640×480 simulated
RGB-D streams and an RViz view containing the occupancy grid and occupied voxels.
Install `ros-humble-rtabmap-ros` before enabling it. `mapping:=none` is the default
and needs no RTAB-Map runtime. Both `world:=flat` and `world:=house` are supported.

The control node supplies simulation ground-truth `odom → base_link`; RTAB-Map
supplies `map → odom`, using wall time because the stack publishes no `/clock`.
This validates mapping, not visual odometry accuracy. Real camera depth and real
robot odometry must be integrated before hardware mapping is supported.

Output topics are under `/robodog/mapping`: `map`, `octomap_binary`,
`octomap_full`, and `octomap_occupied_space`. Map products are produced when
subscribed; the mapping RViz view subscribes automatically. Databases persist at
`${ROS_HOME:-~/.ros}/robodog/maps/{flat,house}.db`. Override with
`mapping_database:=/absolute/path/session.db` for a separate session or to reload
another map. Startup never deletes an existing database. Existing databases are
reopened for mapping, not localization-only operation.
