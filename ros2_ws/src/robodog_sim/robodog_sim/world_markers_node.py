"""
Publish the test world to RViz as a MarkerArray.

The markers come from the SAME world_spec the MuJoCo model is built from, so
what the operator sees in RViz is geometrically what the robot collides with in
simulation. This is also what makes RViz useful on the real robot later: point
the node at the same world and the map shown around the live TF tree is the map
the robot was developed against.

Markers are latched (TRANSIENT_LOCAL) and re-published slowly, so an RViz
started after the node still sees the world.

Namespaces follow the spec's tags (wall, furniture, obstacle, stair, ...), so a
whole class of object can be switched off in the RViz display tree -- useful for
seeing the robot inside a room without the walls in the way.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .world_spec import Prim, World
from .worlds.house import WORLDS

LATCHED = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)

SHAPE = {"box": Marker.CUBE, "cylinder": Marker.CYLINDER, "sphere": Marker.SPHERE}


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def prim_marker(p: Prim, i: int, frame: str, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = p.tag
    m.id = i
    m.type = SHAPE[p.type]
    m.action = Marker.ADD
    m.pose.position = Point(x=float(p.pos[0]), y=float(p.pos[1]), z=float(p.pos[2]))
    q = quat_from_rpy(*p.rpy)
    m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
    m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
    if p.type == "box":
        m.scale = Vector3(x=float(p.size[0]), y=float(p.size[1]), z=float(p.size[2]))
    elif p.type == "cylinder":
        d = float(p.size[0]) * 2.0
        m.scale = Vector3(x=d, y=d, z=float(p.size[1]))
    else:
        d = float(p.size[0]) * 2.0
        m.scale = Vector3(x=d, y=d, z=d)
    r, g, b, a = p.rgba
    # Movable objects are drawn translucent, so an operator can tell at a glance
    # which obstacles the robot is allowed to push through.
    m.color = ColorRGBA(r=r, g=g, b=b, a=(0.65 if p.movable else float(a)))
    return m


def waypoint_markers(world: World, frame: str, stamp, start_id: int) -> list[Marker]:
    out = []
    for k, wp in enumerate(world.waypoints):
        m = Marker()
        m.header.frame_id, m.header.stamp = frame, stamp
        m.ns, m.id, m.type, m.action = "waypoint", start_id + 2 * k, Marker.CYLINDER, Marker.ADD
        m.pose.position = Point(x=float(wp.pos[0]), y=float(wp.pos[1]), z=0.01)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.22, y=0.22, z=0.02)
        m.color = ColorRGBA(r=0.10, g=0.70, b=0.85, a=0.55)
        out.append(m)
        t = Marker()
        t.header.frame_id, t.header.stamp = frame, stamp
        t.ns, t.id = "waypoint_label", start_id + 2 * k + 1
        t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position = Point(x=float(wp.pos[0]), y=float(wp.pos[1]), z=0.32)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.11
        t.color = ColorRGBA(r=0.95, g=0.95, b=0.98, a=0.9)
        t.text = wp.name if not wp.note else f"{wp.name}\n{wp.note}"
        out.append(t)
    return out


class WorldMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__("robodog_world_markers")
        self.declare_parameter("world", "house")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("republish_period_s", 5.0)
        self.declare_parameter("show_waypoints", True)

        name = str(self.get_parameter("world").value)
        if name not in WORLDS:
            raise ValueError(f"unknown world '{name}'; available: {sorted(WORLDS)}")
        self.world = WORLDS[name]()
        self.frame = str(self.get_parameter("frame_id").value)
        self.pub = self.create_publisher(MarkerArray, "robodog/world_markers", LATCHED)
        self.timer = self.create_timer(float(self.get_parameter("republish_period_s").value),
                                       self.publish)
        self.publish()
        self.get_logger().info(
            f"world '{name}': {len(self.world.prims)} objects, "
            f"{len(self.world.waypoints)} waypoints, frame '{self.frame}'")

    def publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        arr.markers = [prim_marker(p, i, self.frame, stamp)
                       for i, p in enumerate(self.world.prims)]
        if bool(self.get_parameter("show_waypoints").value):
            arr.markers += waypoint_markers(self.world, self.frame, stamp,
                                            len(self.world.prims) + 1)
        self.pub.publish(arr)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = WorldMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
