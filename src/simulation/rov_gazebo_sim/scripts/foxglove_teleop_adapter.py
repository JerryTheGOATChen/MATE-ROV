#!/usr/bin/env python3
"""Adapt Foxglove Teleop Twist commands to the ROV's stamped velocity interface."""

from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


@dataclass
class CommandSource:
    """Latest command received from one Foxglove Teleop panel."""

    command: Twist = field(default_factory=Twist)
    received_ns: int | None = None


class FoxgloveTeleopAdapter(Node):
    """Merge planar and depth/yaw controls into the existing ``/cmd_vel`` input."""

    def __init__(self) -> None:
        super().__init__("foxglove_teleop_adapter")
        self.declare_parameter("timeout_s", 0.4)
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("max_surge", 0.5)
        self.declare_parameter("max_sway", 0.3)
        self.declare_parameter("max_heave", 0.3)
        self.declare_parameter("max_yaw_rate", 0.6)

        self._timeout_ns = int(self.get_parameter("timeout_s").value * 1e9)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._limits = {
            "surge": float(self.get_parameter("max_surge").value),
            "sway": float(self.get_parameter("max_sway").value),
            "heave": float(self.get_parameter("max_heave").value),
            "yaw": float(self.get_parameter("max_yaw_rate").value),
        }

        self._planar = CommandSource()
        self._depth_yaw = CommandSource()
        self._session_active = False
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(
            Twist, "/foxglove/planar_cmd_vel", self._on_planar, 10
        )
        self.create_subscription(
            Twist, "/foxglove/depth_yaw_cmd_vel", self._on_depth_yaw, 10
        )
        self.create_timer(0.05, self._watchdog)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _on_planar(self, command: Twist) -> None:
        self._planar.command = command
        self._planar.received_ns = self.get_clock().now().nanoseconds
        self._session_active = True
        self._publish_combined()

    def _on_depth_yaw(self, command: Twist) -> None:
        self._depth_yaw.command = command
        self._depth_yaw.received_ns = self.get_clock().now().nanoseconds
        self._session_active = True
        self._publish_combined()

    def _fresh(self, source: CommandSource, now_ns: int) -> bool:
        return (
            source.received_ns is not None
            and now_ns - source.received_ns <= self._timeout_ns
        )

    def _publish_combined(self, force_stop: bool = False) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        planar = self._planar.command if self._fresh(self._planar, now_ns) else Twist()
        depth_yaw = (
            self._depth_yaw.command
            if self._fresh(self._depth_yaw, now_ns)
            else Twist()
        )

        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self._output_frame
        if not force_stop:
            output.twist.linear.x = self._clamp(
                planar.linear.x, self._limits["surge"]
            )
            output.twist.linear.y = self._clamp(
                planar.linear.y, self._limits["sway"]
            )
            output.twist.linear.z = self._clamp(
                depth_yaw.linear.z, self._limits["heave"]
            )
            output.twist.angular.z = self._clamp(
                depth_yaw.angular.z, self._limits["yaw"]
            )
        self._publisher.publish(output)

    def _watchdog(self) -> None:
        if not self._session_active:
            return
        now_ns = self.get_clock().now().nanoseconds
        if not self._fresh(self._planar, now_ns) and not self._fresh(
            self._depth_yaw, now_ns
        ):
            self._publish_combined(force_stop=True)
            self._session_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FoxgloveTeleopAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
