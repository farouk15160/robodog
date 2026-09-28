#!/usr/bin/env python3
"""Exercise cmd_vel from the normal simulated standing pose, without a gait request.

Source ROS and the workspace, then run against a freshly launched MuJoCo stack.
Refuses non-simulation or multiple robot-state publishers. Leaves zero velocity
and the stand gait. This tests the public ROS path that previously ignored Twist.
"""
import json
import math
from collections import deque
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from robodog_msgs.msg import RobotState
from robodog_msgs.srv import SetGait


def return_to_stand(node, publisher, stand):
    for _ in range(3):
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=.05)
    assert stand.wait_for_service(timeout_sec=2), 'Cleanup failed: stand service unavailable'
    request = SetGait.Request()
    request.command.gait, request.command.enable = 'stand', True
    future = stand.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    assert future.done(), 'Cleanup failed: stand service timed out'
    response = future.result()
    assert response is not None and response.success, 'Cleanup failed: stand command rejected'


def main():
    rclpy.init()
    node = rclpy.create_node('robodog_cmd_vel_smoke')
    states = deque(maxlen=1)
    subscription = node.create_subscription(
        RobotState, '/robodog/robot_state',
        lambda message: states.append((time.monotonic(), message)), 10)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    stand = node.create_client(SetGait, '/robodog/set_gait')
    controlled = False
    try:
        deadline = time.monotonic() + 30
        while not states and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert states, 'No robot telemetry in this ROS domain'
        state = states[-1][1]
        assert state.simulation.active and state.simulation.backend == 'mujoco', 'Refusing non-MuJoCo robot'
        assert len(node.get_publishers_info_by_topic('/robodog/robot_state')) == 1, 'Multiple robot instances in this ROS domain'
        assert state.controller.active_controller == 'pose' and state.controller.active_pose == 'stand', 'Start with a freshly launched standing simulation'
        assert not state.controller.trajectory_active
        first_x, first_t = state.base_pose.position.x, state.simulation.sim_time_s
        command = Twist()
        command.linear.x = .2
        controlled = True
        deadline = time.monotonic() + 40
        while states[-1][1].simulation.sim_time_s - first_t < 3 and time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.1)
        received_at, final = states[-1]
        assert final.simulation.sim_time_s - first_t >= 3, 'Simulation stalled before the required three seconds'
        assert time.monotonic() - received_at < 1, 'Final robot telemetry is stale'
        displacement = final.base_pose.position.x - first_x
        assert final.controller.active_controller == 'gait', 'cmd_vel did not enter the gait controller'
        assert final.controller.active_gait == 'trot', 'cmd_vel did not enter the configured travel gait'
        assert displacement > .15, f'Robot did not move: {displacement:.3f} m'
        assert not final.safety.estop_latched
        assert len(final.joints) == 12 and all(math.isfinite(j.effort) for j in final.joints)
        report = dict(start_controller=state.controller.active_controller,
                      final_controller=final.controller.active_controller,
                      gait=final.controller.active_gait, displacement_m=displacement,
                      simulation_seconds=final.simulation.sim_time_s-first_t,
                      realtime_factor=final.simulation.realtime_factor,
                      joint_torques_nm={j.name: j.effort for j in final.joints})
    finally:
        try:
            if controlled:
                return_to_stand(node, publisher, stand)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
    Path('/tmp/robodog_cmd_vel_smoke.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
