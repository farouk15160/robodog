# RGB-D SLAM and OctoMap integration sources

Checked 2026-09-28 against the upstream ROS 2 implementations linked below. These links follow upstream branches; deployed package versions and runtime validation belong in the mapping runbook.

## Which to use

Use both. SLAM estimates the robot trajectory and corrects accumulated drift when places are revisited. OctoMap represents observed occupied, free and unknown 3D space in a probabilistic octree. OctoMap requires sensor poses; it does not replace localization or loop closure. [OctoMap project](https://octomap.github.io/), [RTAB-Map project](https://introlab.github.io/rtabmap/).

For Robodog's RGB-D camera, RTAB-Map is the SLAM integration choice. Its ROS map manager already maintains OctoMap from graph poses and local observations, so a second accumulating `octomap_server` is unnecessary. This matters after loop closure: the occupancy representation can follow corrected poses. Outputs include `map`, `cloud_map`, `octomap_binary`, `octomap_full`, `octomap_occupied_space`, `octomap_obstacles`, `octomap_ground`, `octomap_empty_space` and `octomap_grid`. OctoMap support is conditional on how RTAB-Map was built. [RTAB-Map MapsManager](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_util/src/MapsManager.cpp).

## Frames, timestamps and camera prerequisites

The ROS launch interface separates RGB-D odometry from SLAM. `rgbd_odometry` can publish `odom → base_link`; `rtabmap` publishes `map → odom`. A nonempty SLAM `odom_frame_id` tells it to obtain odometry from TF. Only one component should own each TF edge. For simulation mapping with existing control TF, use `frame_id: base_link`, `odom_frame_id: odom`, `map_frame_id: map`, `publish_tf: true` on SLAM and do not launch a second odometry broadcaster. Simulation ground truth provides an odometry input, not evidence of hardware visual-odometry accuracy. [Official launch implementation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py).

Depth must be registered to the RGB optical projection, with camera intrinsics describing the actual rendered/acquired image. The official RealSense example enables depth alignment and consumes `aligned_depth_to_color/image_raw`. Resizing a depth image independently of RGB does not establish physical registration. [Official RGB-D example](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_color.launch.py).

RGB-D odometry checks that RGB and depth dimensions have compatible scaling; the former Robodog pair, 1280 × 720 RGB and 640 × 480 depth, fails its integer ratio assertion. Identical registered dimensions avoid this ambiguity. Accepted depth encodings include millimetre `16UC1` and metre `32FC1`. RGB, registered depth, camera calibration and the sensor pose used to produce them must share the acquisition timestamp. Exact synchronization is appropriate only when those timestamps are identical. [RGB-D odometry implementation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/nodelets/rgbd_odometry.cpp).

Robodog publishes camera data with best-effort QoS. RTAB-Map's image and camera-info subscription QoS must therefore use `qos_image: 2` and `qos_camera_info: 2`; its launch interface documents 1 as reliable and 2 as best effort. [Official launch QoS declarations](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py).

## Parameter and output details

ROS wrapper parameters retain native types, for example `subscribe_depth: true`, `approx_sync: false`, `wait_for_transform: 0.2`. RTAB-Map core parameters are declared as strings in ROS: quote values such as `Grid/3D: 'true'`, `Grid/Sensor: '1'`, `Grid/CellSize: '0.05'` and `Rtabmap/DetectionRate: '1.0'`. `database_path` selects persistent map storage; avoid automatic database deletion in normal launches. [CoreWrapper parameter declarations](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/src/CoreWrapper.cpp).

`Grid/Sensor=1` selects depth-derived local occupancy; `Grid/3D=true` retains height information. `Grid/RayTracing=true` fills observed free space between sensor and occupied cells and requires OctoMap support for 3D ray tracing. Cell size is a resolution/cost choice, not camera accuracy; 0.05 m is an initial mapping configuration. [Core parameter definitions](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h).

Map publication and computation depend on subscribers. `map_cleanup` controls clearing unused assembled map caches; retaining it or keeping an occupied-space subscriber makes live inspection predictable. The database remains the source of local observations for rebuilding maps. [RTAB-Map MapsManager](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_util/src/MapsManager.cpp).

## Saving and inspecting

RTAB-Map exposes `backup` (`std_srvs/srv/Empty`) and `octomap_full`/`octomap_binary` (`octomap_msgs/srv/GetOctomap`) services. Services include the node-name prefix, whereas map topics use the namespace: with namespace `robodog/mapping` and node `rtabmap`, the full-map service is `/robodog/mapping/rtabmap/octomap_full`, but its topic is `/robodog/mapping/octomap_full`. The backup callback flushes the current database, copies it to `<database_path>.back`, then reloads it. [CoreWrapper service and backup implementation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/src/CoreWrapper.cpp).

For an OctoMap file, the ROS 2 saver accepts `full` and `octomap_path` parameters and calls a `GetOctomap` service. Request the full `ColorOcTree` and write `.ot` to preserve colors and occupancy probabilities. The saver explicitly warns that requesting binary `ColorOcTree` is unsupported. The executable name must be checked against the installed package. Example when it is `octomap_saver_node`:

```bash
ros2 run octomap_server octomap_saver_node --ros-args \
  -p full:=true -p octomap_path:=/absolute/path/house.ot \
  -r octomap_full:=/robodog/mapping/rtabmap/octomap_full
```

This consumes RTAB-Map's service; it does not require running an independent OctoMap mapping server. [ROS 2 OctoMap saver](https://github.com/OctoMap/octomap_mapping/blob/ros2/octomap_server/src/octomap_saver.cpp).

## Validation boundaries

A useful integration smoke test observes synchronized registered RGB-D, a single TF chain, increasing map graph nodes while the robot moves, nonempty 2D occupancy and 3D occupied-space/OctoMap data, and successful persistence/export. Featureless views can defeat visual loop closure even when depth occupancy updates work. A short simulated mapping run validates the pipeline, not field localization accuracy or autonomous navigation. Real hardware requires calibrated camera intrinsics/extrinsics and a valid independent odometry source or separately validated RGB-D odometry.
