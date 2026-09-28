#!/usr/bin/env python3
"""Bounded live MuJoCo / RTAB-Map / OctoMap verification; never launches a robot.

Source ROS and the workspace first. Default motion is 0.5 m at 0.10 m/s,
limited to 15 wall seconds. Use --distance 0 to inspect a persisted-map reload.
Every attempted run writes JSON, including partial measurements and failures.
"""
import argparse
from collections import OrderedDict
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from octomap_msgs.msg import Octomap
from robodog_msgs.msg import RobotState
from robodog_msgs.srv import SetGait
from rtabmap_msgs.msg import MapData
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

PREFIX = '/robodog/mapping/'
CAMERA = '/robodog/camera/'
SENSORS = ('color/image_raw', 'depth/image_rect_raw', 'color/camera_info', 'depth/camera_info')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def xyz(point):
    return [point.x, point.y, point.z]


def cloud_points(message):
    fields = {field.name: field for field in message.fields}
    require(all(name in fields for name in ('x', 'y', 'z')), 'Occupied cloud lacks XYZ fields')
    codes = {7: 'f', 8: 'd'}
    require(all(fields[name].datatype in codes for name in ('x', 'y', 'z')), 'Unsupported XYZ datatype')
    prefix = '>' if message.is_bigendian else '<'
    count = message.width * message.height
    require(count > 0 and len(message.data) >= message.row_step * message.height, 'Empty/truncated occupied cloud')
    for row in range(message.height):
        for column in range(message.width):
            offset = row * message.row_step + column * message.point_step
            values = [struct.unpack_from(prefix + codes[fields[name].datatype], message.data,
                                         offset + fields[name].offset)[0] for name in ('x', 'y', 'z')]
            require(all(math.isfinite(value) for value in values), 'Occupied cloud contains nonfinite points')
    return count


class Probe:
    def __init__(self, node):
        self.node, self.state, self.state_at = node, None, 0.
        self.maps, self.map_counts, self.map_subscriptions = {}, {}, {}
        self.sensor = {name: OrderedDict() for name in SENSORS}
        self.tf_edges = set()
        self.tf_errors = []
        self.subscriptions = [node.create_subscription(RobotState, '/robodog/robot_state', self.on_state, 10),
                              node.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)]
        for name in SENSORS:
            kind = CameraInfo if name.endswith('camera_info') else Image
            self.subscriptions.append(node.create_subscription(kind, CAMERA + name,
                lambda message, key=name: self.on_sensor(key, message), qos_profile_sensor_data))

    def on_state(self, message):
        self.state, self.state_at = message, time.monotonic()

    def on_sensor(self, key, message):
        metadata = dict(stamp_ns=stamp(message), width=message.width, height=message.height,
                        received_at=time.monotonic(),
                        frame=message.header.frame_id, encoding=getattr(message, 'encoding', None),
                        k=list(message.k) if hasattr(message, 'k') else None)
        self.sensor[key][metadata['stamp_ns']] = metadata
        while len(self.sensor[key]) > 12:
            self.sensor[key].popitem(last=False)

    def on_tf(self, message):
        for transform in message.transforms:
            edge = (transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/'))
            if edge not in (('map', 'odom'), ('odom', 'base_link')):
                continue
            self.tf_edges.add(edge)
            rotation = transform.transform.rotation
            values = xyz(transform.transform.translation) + [rotation.x, rotation.y, rotation.z, rotation.w]
            if not all(math.isfinite(value) for value in values) and not self.tf_errors:
                self.tf_errors.append(f'Nonfinite TF on {edge}')

    def tf_graph(self):
        expected = {'/robodog_control_node', '/robot_state_publisher', '/robodog/mapping/rtabmap'}
        publishers = self.node.get_publishers_info_by_topic('/tf')
        names = [f'{item.node_namespace.rstrip("/")}/{item.node_name}' for item in publishers]
        require(set(names) == expected and len(names) == len(expected),
                f'Unexpected/duplicate TF publishers: {names}')
        nodes = [f'{namespace.rstrip("/")}/{name}' for name, namespace in
                 self.node.get_node_names_and_namespaces()]
        require(all(nodes.count(name) == 1 for name in expected), 'Duplicate/missing control or mapping node')
        require(len(self.tf_edges) == 2, 'Missing TF edge in map→odom→base_link')
        return dict(publishers=names, edges_received=[f'{a}->{b}' for a, b in sorted(self.tf_edges)],
                    per_edge_publisher_gid_verified=False,
                    authority_basis='Humble rclpy lacks message publisher GID; expected node graph and received edges checked. '
                                    'Edge ownership requires launch/source review, not live GID attribution.')

    def discover_maps(self):
        for name, kind in [('map', OccupancyGrid), ('octomap_full', Octomap),
                           ('octomap_occupied_space', PointCloud2), ('mapData', MapData)]:
            if name in self.map_subscriptions:
                continue
            publishers = self.node.get_publishers_info_by_topic(PREFIX + name)
            if publishers:
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=publishers[0].qos_profile.durability)
                self.map_subscriptions[name] = self.node.create_subscription(kind, PREFIX + name,
                    lambda message, key=name: self.on_map(key, message), qos)

    def on_map(self, key, message):
        self.maps[key] = message
        self.map_counts[key] = self.map_counts.get(key, 0) + 1

    def guard_robot(self):
        require(self.state is not None and time.monotonic() - self.state_at < 1.5, 'Robot telemetry missing/stale')
        require(self.state.simulation.active and self.state.simulation.backend == 'mujoco', 'Refusing non-MuJoCo robot')
        require(len(self.node.get_publishers_info_by_topic('/robodog/robot_state')) == 1,
                'Expected exactly one RobotState publisher in this ROS domain')
        require(not self.state.safety.estop_latched and not self.state.safety.estop_engaged, 'Robot emergency stop is active')
        require(all(math.isfinite(value) for value in xyz(self.state.base_pose.position)), 'Nonfinite robot odometry')
        require(all(math.isfinite(value) for value in
                    (self.state.simulation.sim_time_s, self.state.simulation.realtime_factor)),
                'Nonfinite simulation time/RTF')

    def aligned_camera(self):
        common = set.intersection(*(set(value) for value in self.sensor.values()))
        require(bool(common), 'No timestamp-aligned RGB, depth and both camera-info messages')
        latest = max(common)
        values = [self.sensor[key][latest] for key in SENSORS]
        require(all(time.monotonic() - value['received_at'] < 3. for value in values), 'Aligned camera messages are stale')
        require(latest > 0, 'Camera timestamp is zero')
        require(all((value['width'], value['height']) == (640, 480) for value in values), 'Expected 640x480 aligned images/info')
        require(all(value['frame'] == 'camera_color_optical_frame' for value in values), 'RGB/depth optical frames differ')
        require(values[0]['encoding'] in ('rgb8', 'bgr8') and values[1]['encoding'] in ('16UC1', '32FC1'),
                'Unexpected RGB/depth encoding')
        left, right = values[2]['k'], values[3]['k']
        require(all(math.isfinite(value) for value in left + right) and left[0] > 0 and left[4] > 0,
                'Camera intrinsics are invalid')
        require(all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(left, right)), 'Color/depth intrinsics differ')
        return dict(stamp_ns=latest, width=640, height=480, frame=values[0]['frame'], k=left,
                    depth_encoding=values[1]['encoding'],
                    depth_units='millimetres' if values[1]['encoding'] == '16UC1' else 'metres')

    def snapshot(self):
        self.guard_robot()
        camera = self.aligned_camera()
        for name in ('map', 'octomap_full', 'octomap_occupied_space'):
            require(name in self.maps, f'Missing {PREFIX + name}')
            require(self.maps[name].header.frame_id.lstrip('/') == 'map', f'{name} is not in map frame')
        grid, octree, cloud = (self.maps[key] for key in ('map', 'octomap_full', 'octomap_occupied_space'))
        require(math.isclose(grid.info.resolution, .05, abs_tol=1e-6) and
                math.isclose(octree.resolution, .05, abs_tol=1e-6), 'Expected 5cm map and OctoMap resolution')
        require(len(grid.data) == grid.info.width * grid.info.height > 0, 'Empty/truncated occupancy grid')
        require(all(-1 <= cell <= 100 for cell in grid.data), 'Invalid occupancy-grid cell value')
        known = sum(cell >= 0 for cell in grid.data)
        require(known > 0, 'Occupancy grid has no known cells')
        require(all(math.isfinite(value) for value in xyz(grid.info.origin.position)), 'Nonfinite map origin')
        require(len(octree.data) > 0 and not octree.binary and octree.id in ('OcTree', 'ColorOcTree'), 'Invalid full OctoMap payload')
        points = cloud_points(cloud)
        require(not self.tf_errors, '; '.join(self.tf_errors))
        tf_graph = self.tf_graph()
        graph = self.maps.get('mapData')
        return dict(known_cells=known, occupied_cells=sum(cell > 50 for cell in grid.data),
                    cloud_points=points, octomap_bytes=len(octree.data), resolution_m=grid.info.resolution,
                    graph_nodes=len(graph.graph.poses_id) if graph is not None else None,
                    odom_position_m=xyz(self.state.base_pose.position), simulation_time_s=self.state.simulation.sim_time_s,
                    clamp_events=self.state.safety.clamp_events, realtime_factor=self.state.simulation.realtime_factor,
                    map_publications=dict(self.map_counts), aligned_camera=camera,
                    tf_verification=tf_graph)


def await_snapshot(node, probe, timeout, after=None):
    deadline, reason = time.monotonic() + timeout, 'No mapping samples'
    while time.monotonic() < deadline:
        probe.discover_maps()
        rclpy.spin_once(node, timeout_sec=.1)
        try:
            if after is not None:
                require(all(probe.map_counts.get(key, 0) > after.get(key, 0)
                            for key in ('map', 'octomap_full', 'octomap_occupied_space')),
                        'Mapping outputs did not refresh during motion')
            return probe.snapshot()
        except RuntimeError as error:
            reason = str(error)
    raise RuntimeError(f'Mapping readiness timed out: {reason}')


def call(node, client, request, timeout=3.):
    require(client.wait_for_service(timeout_sec=timeout), f'Service unavailable: {client.srv_name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    require(future.done() and future.exception() is None and future.result() is not None,
            f'Service failed/timed out: {client.srv_name}')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=.5)
    parser.add_argument('--output', type=Path, default=Path('/tmp/robodog_mapping_smoke.json'))
    parser.add_argument('--backup', action='store_true')
    args = parser.parse_args()
    require(math.isfinite(args.distance) and 0 <= args.distance <= .6, 'Distance must be between 0 and 0.6m')
    report = dict(passed=False, requested_distance_m=args.distance, errors=[])
    rclpy.init()
    node = rclpy.create_node('robodog_mapping_smoke')
    probe = Probe(node)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    stand = node.create_client(SetGait, '/robodog/set_gait')
    safe_robot = False
    try:
        report['initial'] = await_snapshot(node, probe, 45.)
        probe.guard_robot()
        safe_robot = True
        start = report['initial']['odom_position_m']
        deadline, travelled = time.monotonic() + 15., 0.
        while travelled < args.distance and time.monotonic() < deadline:
            probe.guard_robot()
            command = Twist()
            command.linear.x = .10
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.1)
            now = xyz(probe.state.base_pose.position)
            travelled = math.hypot(now[0] - start[0], now[1] - start[1])
            require(travelled <= .65, 'Travel bound exceeded')
        publisher.publish(Twist())
        report['final'] = await_snapshot(node, probe, 8.,
            report['initial']['map_publications'] if args.distance else None)
        report['odom_delta_m'] = [b - a for a, b in zip(start, report['final']['odom_position_m'])]
        report['travelled_m'] = travelled
        require(travelled >= args.distance, f'Movement timed out after 15 wall seconds: {travelled:.3f}m')
        if args.backup:
            backup = node.create_client(Empty, '/robodog/mapping/rtabmap/backup')
            call(node, backup, Empty.Request())
            report['backup_service_acknowledged'] = True
    except (Exception, KeyboardInterrupt) as error:
        report['errors'].append(f'{type(error).__name__}: {error}')
    finally:
        try:
            if safe_robot:
                for _ in range(3):
                    publisher.publish(Twist())
                    rclpy.spin_once(node, timeout_sec=.05)
                request = SetGait.Request()
                request.command.gait, request.command.enable = 'stand', True
                response = call(node, stand, request)
                require(response.success, f'Stand cleanup rejected: {response.message}')
                report['stop_and_stand_acknowledged'] = True
        except Exception as error:
            report['errors'].append(f'Cleanup: {error}')
        report['passed'] = not report['errors']
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
