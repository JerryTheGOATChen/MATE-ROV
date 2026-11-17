# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Lifecycle driver for the Blue Robotics Bar30 pressure sensor."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

from pressure_driver.ms5837 import MS5837

GRAVITY_M_S2 = 9.80665
# Index of the z variance in a row-major 6x6 pose covariance.
_Z_VARIANCE_INDEX = 14


def depth_from_pressure(pressure_pa, surface_pressure_pa, fluid_density_kg_m3):
    """Return the depth below the surface in metres, positive downwards."""
    return (pressure_pa - surface_pressure_pa) / (fluid_density_kg_m3 * GRAVITY_M_S2)


def open_bar30(i2c_bus):
    """Open the Bar30 on the given I2C bus number and load its calibration."""
    from smbus2 import SMBus

    bus = SMBus(i2c_bus)
    try:
        sensor = MS5837(bus)
        sensor.init()
    except Exception:
        bus.close()
        raise
    return sensor


class Bar30Node(Node):
    """Publish the ROV's depth as the z position of an odometry message.

    Parameters:
     - `i2c_bus` (int): I2C bus number the sensor is wired to (default 1)
     - `rate_hz` (double): publish rate, at most about 20 Hz (default 10.0)
     - `fluid_density` (double): water density in kg/m^3 (default 997.0, fresh water)
     - `surface_pressure_pa` (double): air pressure at the surface in Pa (default 101325.0)
     - `depth_variance` (double): variance of the published z in m^2 (default 0.0004)
     - `frame_id` (string): frame the z position is expressed in (default "odom")
     - `child_frame_id` (string): the ROV's body frame (default "base_link")

    Publishes:
     - `depth` (nav_msgs/Odometry): z = -depth. Only z and its variance are meaningful.
    """

    def __init__(self, open_sensor=open_bar30, **kwargs):
        super().__init__("bar30_node", **kwargs)
        self._open_sensor = open_sensor
        self._sensor = None
        self._pub = None
        self._timer = None

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("fluid_density", 997.0)
        self.declare_parameter("surface_pressure_pa", 101325.0)
        self.declare_parameter("depth_variance", 0.0004)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._rate_hz = self.get_parameter("rate_hz").value
        self._fluid_density = self.get_parameter("fluid_density").value
        self._surface_pressure_pa = self.get_parameter("surface_pressure_pa").value
        self._depth_variance = self.get_parameter("depth_variance").value
        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value
        i2c_bus = self.get_parameter("i2c_bus").value

        if (
            self._rate_hz <= 0.0
            or self._fluid_density <= 0.0
            or self._depth_variance <= 0.0
        ):
            self.get_logger().error(
                "rate_hz, fluid_density and depth_variance must be positive"
            )
            return TransitionCallbackReturn.FAILURE

        try:
            self._sensor = self._open_sensor(i2c_bus)
        except Exception as error:
            self.get_logger().error(
                f"Could not open the Bar30 on I2C bus {i2c_bus}: {error}"
            )
            return TransitionCallbackReturn.FAILURE

        self._pub = self.create_lifecycle_publisher(Odometry, "depth", 10)
        self.get_logger().info(f"Configured the Bar30 on I2C bus {i2c_bus}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_depth)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._stop_timer()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        return TransitionCallbackReturn.SUCCESS

    def _publish_depth(self):
        try:
            pressure_pa, _ = self._sensor.read()
        except OSError as error:
            # Publish nothing, so that consumers see a stale topic and not a wrong depth.
            self.get_logger().warning(
                f"Bar30 read failed: {error}", throttle_duration_sec=5.0
            )
            return

        depth = depth_from_pressure(
            pressure_pa, self._surface_pressure_pa, self._fluid_density
        )
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id
        msg.pose.pose.position.z = -depth
        msg.pose.covariance[_Z_VARIANCE_INDEX] = self._depth_variance
        self._pub.publish(msg)

    def _stop_timer(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

    def _release(self):
        self._stop_timer()
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        if self._sensor is not None:
            self._sensor.close()
            self._sensor = None


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Bar30Node())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
