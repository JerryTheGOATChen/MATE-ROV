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
"""Lifecycle driver for the Blue Robotics Ping2 altimeter."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from sensor_msgs.msg import Range

_MM_PER_M = 1000.0


class Ping2:
    """The Ping2 operations this node needs, on top of a connected `brping.Ping1D`."""

    def __init__(self, device):
        self._device = device

    def configure(self, speed_of_sound_mm_s, ping_interval_ms):
        """Write the device settings. Raises `OSError` if the device does not report them back."""
        from brping import definitions

        self._write_setting(
            "speed of sound",
            definitions.PING1D_SET_SPEED_OF_SOUND,
            definitions.PING1D_SPEED_OF_SOUND,
            speed_of_sound_mm_s,
        )
        self._write_setting(
            "ping interval",
            definitions.PING1D_SET_PING_INTERVAL,
            definitions.PING1D_PING_INTERVAL,
            ping_interval_ms,
        )

    def get_distance(self):
        """Return the latest reading as brping's `get_distance` dict, or None without a reply."""
        return self._device.get_distance()

    def close(self):
        self._device.iodev.close()

    def _write_setting(self, name, set_id, get_id, value):
        # brping 0.2.5's set_speed_of_sound() and set_ping_interval() use field names that no
        # longer match its protocol definitions: the first sends zero and the second checks a
        # stale value. Fill in the message's one field by its defined name instead.
        from brping import PingMessage

        msg = PingMessage(set_id)
        setattr(msg, msg.payload_field_names[0], value)
        msg.pack_msg_data()
        self._device.write(msg.msg_data)

        reply = self._device.legacyRequest(get_id)
        if reply is None:
            raise OSError(f"no reply when setting the {name}")
        reported = getattr(reply, reply.payload_field_names[0])
        if reported != value:
            raise OSError(
                f"asked for a {name} of {value}, the device reports {reported}"
            )


def open_ping2(port, baudrate):
    """Open the Ping2 on the given serial port and check that it answers."""
    from brping import Ping1D

    device = Ping1D()
    device.connect_serial(port, baudrate)
    ping2 = Ping2(device)
    if not device.initialize():
        ping2.close()
        raise OSError("the device did not answer")
    return ping2


def range_from_reading(reading, min_confidence):
    """Convert a Ping `get_distance` reply to `(range, min_range, max_range)` in metres.

    Returns None when there is no reply or the device's confidence in it is too low.
    """
    if reading is None or reading["confidence"] < min_confidence:
        return None
    min_range = reading["scan_start"] / _MM_PER_M
    max_range = (reading["scan_start"] + reading["scan_length"]) / _MM_PER_M
    return reading["distance"] / _MM_PER_M, min_range, max_range


class Ping2Node(Node):
    """Publish the distance from the Ping2 to the floor.

    Parameters:
     - `port` (string): serial device the Ping2 is on (default "/dev/ttyUSB0")
     - `baudrate` (int): serial baud rate (default 115200)
     - `rate_hz` (double): ping and publish rate (default 10.0)
     - `speed_of_sound` (double): speed of sound in the water in m/s (default 1500.0)
     - `min_confidence` (int): readings the device rates below this percentage are dropped
       (default 80)
     - `field_of_view` (double): beam width in radians (default 0.436, the Ping2's 25 degrees)
     - `frame_id` (string): frame of the transducer (default "ping2_link")

    Publishes:
     - `altitude` (sensor_msgs/Range): distance along the beam to the floor.
    """

    def __init__(self, open_device=open_ping2, **kwargs):
        super().__init__("ping2_node", **kwargs)
        self._open_device = open_device
        self._device = None
        self._pub = None
        self._timer = None

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("speed_of_sound", 1500.0)
        self.declare_parameter("min_confidence", 80)
        self.declare_parameter("field_of_view", math.radians(25.0))
        self.declare_parameter("frame_id", "ping2_link")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._rate_hz = self.get_parameter("rate_hz").value
        self._min_confidence = self.get_parameter("min_confidence").value
        self._field_of_view = self.get_parameter("field_of_view").value
        self._frame_id = self.get_parameter("frame_id").value
        speed_of_sound = self.get_parameter("speed_of_sound").value
        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        if self._rate_hz <= 0.0 or speed_of_sound <= 0.0:
            self.get_logger().error("rate_hz and speed_of_sound must be positive")
            return TransitionCallbackReturn.FAILURE

        try:
            self._device = self._open_device(port, baudrate)
            self._device.configure(
                round(speed_of_sound * _MM_PER_M), round(_MM_PER_M / self._rate_hz)
            )
        except Exception as error:
            self.get_logger().error(f"Could not set up the Ping2 on {port}: {error}")
            self._release()
            return TransitionCallbackReturn.FAILURE

        self._pub = self.create_lifecycle_publisher(Range, "altitude", 10)
        self.get_logger().info(f"Configured the Ping2 on {port}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_altitude)
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

    def _publish_altitude(self):
        try:
            reading = self._device.get_distance()
        except OSError as error:
            self.get_logger().warning(
                f"Ping2 read failed: {error}", throttle_duration_sec=5.0
            )
            return

        # Publish nothing for a missing or low-confidence reading, so that consumers see a
        # stale topic and not a wrong altitude.
        result = range_from_reading(reading, self._min_confidence)
        if result is None:
            self.get_logger().warning(
                "No confident Ping2 reading", throttle_duration_sec=5.0
            )
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self._field_of_view
        msg.range, msg.min_range, msg.max_range = result
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
        if self._device is not None:
            self._device.close()
            self._device = None


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Ping2Node())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
