"""Command-line client for an atomic room-scan snapshot."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

def _absolute_directory(value: str) -> str:
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("output directory must be absolute")
    return str(path)


def parse_args(arguments=None):
    parser = argparse.ArgumentParser(
        description="Save the assembled RTAB point cloud, full OctoMap and database backup")
    parser.add_argument("--output-dir", type=_absolute_directory, default="",
                        dest="output_directory",
                        help="absolute export root; default: ~/.ros/robodog/exports")
    parser.add_argument("--name", default="", help="unique session name; default: UTC timestamp")
    parser.add_argument("--timeout", type=float, default=60.0, help="service timeout in seconds")
    return parser.parse_args(arguments)


def main(arguments=None):
    import rclpy
    from rclpy.node import Node
    from robodog_msgs.srv import SaveMap

    args = parse_args(arguments)
    rclpy.init(args=[])
    node = Node("save_room_map_cli")
    client = node.create_client(SaveMap, "/robodog/mapping/save_map")
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            print("Map save service is unavailable; start bringup with mapping enabled", file=sys.stderr)
            return 2
        request = SaveMap.Request()
        request.output_directory = args.output_directory
        request.name = args.name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print("Timed out waiting for the map export", file=sys.stderr)
            return 2
        response = future.result()
        if not response.success:
            print(response.message, file=sys.stderr)
            return 1
        print(response.manifest_path)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
