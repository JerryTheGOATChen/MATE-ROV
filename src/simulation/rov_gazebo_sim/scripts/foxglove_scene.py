#!/usr/bin/env python3
"""Publish a Foxglove-friendly view of the Gazebo pool and moving ROV."""

from collections import deque
from math import cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """Return a geometry_msgs Quaternion-compatible tuple from roll, pitch and yaw."""
    cr, sr = cos(roll * 0.5), sin(roll * 0.5)
    cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
    cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class FoxgloveScene(Node):
    """Mirror the important Gazebo visuals as standard ROS markers."""

    def __init__(self):
        super().__init__("foxglove_scene")
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scene_pub = self.create_publisher(
            MarkerArray, "/sim/foxglove_scene", static_qos
        )
        self.rov_pub = self.create_publisher(MarkerArray, "/sim/foxglove_rov", 10)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_odometry, 10)
        self.create_timer(1.0, self.publish_scene)
        self.trail = deque(maxlen=3000)
        self.marker_id = 0

    def base_marker(self, namespace: str, marker_type: int, name: str = "") -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.text = name
        return marker

    @staticmethod
    def set_pose(marker: Marker, position, rotation=(0.0, 0.0, 0.0)):
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        qx, qy, qz, qw = quaternion_from_rpy(*rotation)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

    @staticmethod
    def set_color(marker: Marker, color):
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    def shape(
        self,
        markers,
        namespace,
        marker_type,
        position,
        scale,
        color,
        rotation=(0.0, 0.0, 0.0),
    ):
        marker = self.base_marker(namespace, marker_type)
        self.set_pose(marker, position, rotation)
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        self.set_color(marker, color)
        markers.append(marker)

    def label(self, markers, text, position, color=(0.9, 0.95, 1.0, 0.9)):
        marker = self.base_marker("labels", Marker.TEXT_VIEW_FACING, text)
        self.set_pose(marker, position)
        marker.scale.z = 0.28
        self.set_color(marker, color)
        markers.append(marker)

    def publish_scene(self):
        self.marker_id = 0
        markers = []

        # Clear the old translucent water markers from clients that loaded an earlier scene.
        # Large transparent volumes can produce distracting depth-sorting artifacts in 3D viewers.
        for marker_id in (0, 1):
            marker = self.base_marker("water", Marker.CUBE)
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.append(marker)

        # Pool shell. Gazebo still provides the simulation; only the Foxglove water overlay is gone.
        self.shape(
            markers,
            "pool",
            Marker.CUBE,
            (0, 0, -4.1),
            (21, 13, 0.2),
            (0.45, 0.7, 0.82, 0.85),
        )
        wall_color = (0.72, 0.82, 0.88, 0.32)
        self.shape(markers, "pool", Marker.CUBE, (10.1, 0, -1.8), (0.2, 12.4, 4.4), wall_color)
        self.shape(markers, "pool", Marker.CUBE, (-10.1, 0, -1.8), (0.2, 12.4, 4.4), wall_color)
        self.shape(markers, "pool", Marker.CUBE, (0, 6.1, -1.8), (20.4, 0.2, 4.4), wall_color)
        self.shape(markers, "pool", Marker.CUBE, (0, -6.1, -1.8), (20.4, 0.2, 4.4), wall_color)

        # Mooring and buoy.
        self.shape(
            markers,
            "mooring",
            Marker.CYLINDER,
            (4, 2, -2),
            (0.3, 0.3, 4),
            (0.95, 0.55, 0.1, 1.0),
        )
        self.shape(
            markers,
            "mooring",
            Marker.SPHERE,
            (4, 2, -0.4),
            (0.5, 0.5, 0.5),
            (0.9, 0.15, 0.1, 1.0),
        )
        self.label(markers, "Mooring", (4, 2, 0.25))

        # Coral restoration frame.
        pvc = (0.95, 0.95, 0.95, 1.0)
        for x in (-5.6, -4.4):
            for y in (-3.6, -2.4):
                self.shape(
                    markers,
                    "coral_frame",
                    Marker.CYLINDER,
                    (x, y, -3.75),
                    (0.1, 0.1, 0.5),
                    pvc,
                )
        self.shape(
            markers,
            "coral_frame",
            Marker.CYLINDER,
            (-5, -2.4, -3.5),
            (0.1, 0.1, 1.2),
            pvc,
            (0, 1.5708, 0),
        )
        self.shape(
            markers,
            "coral_frame",
            Marker.CYLINDER,
            (-5, -3.6, -3.5),
            (0.1, 0.1, 1.2),
            pvc,
            (0, 1.5708, 0),
        )
        self.shape(
            markers,
            "coral_frame",
            Marker.CYLINDER,
            (-4.4, -3, -3.5),
            (0.1, 0.1, 1.2),
            pvc,
            (1.5708, 0, 0),
        )
        self.shape(
            markers,
            "coral_frame",
            Marker.CYLINDER,
            (-5.6, -3, -3.5),
            (0.1, 0.1, 1.2),
            pvc,
            (1.5708, 0, 0),
        )
        self.label(markers, "Coral frame", (-5, -3, -3.2))

        # Shipwreck, gate, hanging instrument, and net wall.
        self.shape(
            markers,
            "shipwreck",
            Marker.CUBE,
            (-4, 3.5, -3.55),
            (2.4, 1.0, 0.9),
            (0.45, 0.3, 0.2, 1.0),
            (0, 0, 0.35),
        )
        self.label(markers, "Shipwreck", (-4, 3.5, -2.85))
        self.shape(markers, "gate", Marker.CYLINDER, (7, -1.25, -2.5), (0.1, 0.1, 3), pvc)
        self.shape(markers, "gate", Marker.CYLINDER, (7, -2.75, -2.5), (0.1, 0.1, 3), pvc)
        self.shape(
            markers,
            "gate",
            Marker.CYLINDER,
            (7, -2, -1),
            (0.1, 0.1, 1.5),
            (0.95, 0.55, 0.1, 1.0),
            (1.5708, 0, 0),
        )
        self.label(markers, "Gate", (7, -2, -0.65))
        self.shape(
            markers,
            "instrument",
            Marker.CUBE,
            (0, -4, -1.5),
            (0.5, 0.5, 0.5),
            (0.95, 0.8, 0.1, 1.0),
        )
        self.shape(
            markers,
            "instrument",
            Marker.CYLINDER,
            (0, -4, -0.625),
            (0.02, 0.02, 1.25),
            (0.2, 0.2, 0.2, 1.0),
        )
        self.label(markers, "Instrument", (0, -4, -1.05))
        self.shape(
            markers,
            "net",
            Marker.CUBE,
            (-7, 2.75, -2),
            (0.1, 6.5, 4),
            (0.1, 0.55, 0.3, 0.42),
        )
        self.label(markers, "Net wall", (-6.7, 2.75, -0.25))

        self.scene_pub.publish(MarkerArray(markers=markers))

    def on_odometry(self, msg: Odometry):
        pose = msg.pose.pose
        point = pose.position
        if not self.trail or (
            (point.x - self.trail[-1].x) ** 2
            + (point.y - self.trail[-1].y) ** 2
            + (point.z - self.trail[-1].z) ** 2
            > 0.0025
        ):
            self.trail.append(point)

        body = self.base_marker("rov", Marker.CUBE)
        body.id = 0
        body.header.stamp = msg.header.stamp
        body.pose = pose
        body.scale.x, body.scale.y, body.scale.z = 0.457, 0.575, 0.254
        self.set_color(body, (0.08, 0.25, 0.75, 1.0))

        heading = self.base_marker("rov_heading", Marker.ARROW)
        heading.id = 0
        heading.header.stamp = msg.header.stamp
        heading.pose = pose
        heading.scale.x, heading.scale.y, heading.scale.z = 0.75, 0.12, 0.12
        self.set_color(heading, (1.0, 0.55, 0.05, 1.0))

        trail = self.base_marker("rov_trail", Marker.LINE_STRIP)
        trail.id = 0
        trail.header.stamp = msg.header.stamp
        trail.scale.x = 0.045
        self.set_color(trail, (0.2, 1.0, 0.65, 0.9))
        trail.points = list(self.trail)

        self.rov_pub.publish(MarkerArray(markers=[body, heading, trail]))


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
