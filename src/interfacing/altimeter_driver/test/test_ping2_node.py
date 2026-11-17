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
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Range

from altimeter_driver.ping2_node import Ping2Node, range_from_reading


def reading(distance_mm=2500, confidence=100, scan_start_mm=300, scan_length_mm=9700):
    return {
        "distance": distance_mm,
        "confidence": confidence,
        "scan_start": scan_start_mm,
        "scan_length": scan_length_mm,
    }


class FakePing:
    """Stand in for brping.Ping1D."""

    def __init__(self, reply):
        self.reply = reply
        self.accept_settings = True
        self.speed_of_sound = None
        self.ping_interval = None
        self.closed = False
        self.iodev = SimpleNamespace(close=self._close)

    def set_speed_of_sound(self, speed_of_sound):
        self.speed_of_sound = speed_of_sound
        return self.accept_settings

    def set_ping_interval(self, ping_interval):
        self.ping_interval = ping_interval
        return self.accept_settings

    def get_distance(self):
        return self.reply

    def _close(self):
        self.closed = True


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def collect(node, timeout_s):
    """Run the active node and return the Range messages it publishes."""
    listener = rclpy.create_node("listener")
    received = []
    listener.create_subscription(Range, "altitude", received.append, 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(listener)
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while not received and node.get_clock().now().nanoseconds < deadline:
        executor.spin_once(timeout_sec=0.05)
    executor.shutdown()
    listener.destroy_node()
    return received


def fast_node(device):
    node = Ping2Node(open_device=lambda port, baudrate: device)
    node.set_parameters([rclpy.parameter.Parameter("rate_hz", value=50.0)])
    return node


def test_range_from_reading_converts_to_metres():
    assert range_from_reading(reading(), 80) == pytest.approx((2.5, 0.3, 10.0))


def test_range_from_reading_drops_low_confidence():
    assert range_from_reading(reading(confidence=79), 80) is None
    assert range_from_reading(reading(confidence=80), 80) is not None


def test_range_from_reading_drops_missing_reply():
    assert range_from_reading(None, 80) is None


def test_publishes_range(ros):
    node = fast_node(FakePing(reading()))

    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    received = collect(node, timeout_s=2.0)

    assert received, "no altitude message arrived"
    msg = received[0]
    assert msg.header.frame_id == "ping2_link"
    assert msg.radiation_type == Range.ULTRASOUND
    assert msg.range == pytest.approx(2.5)
    assert msg.min_range == pytest.approx(0.3)
    assert msg.max_range == pytest.approx(10.0)
    assert msg.field_of_view == pytest.approx(0.4363, abs=1e-4)

    node.destroy_node()


def test_low_confidence_publishes_nothing(ros):
    node = fast_node(FakePing(reading(confidence=10)))

    node.trigger_configure()
    node.trigger_activate()

    assert not collect(node, timeout_s=0.5)

    node.destroy_node()


def test_configure_applies_settings(ros):
    device = FakePing(reading())
    node = Ping2Node(open_device=lambda port, baudrate: device)

    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    assert device.speed_of_sound == 1500000  # mm/s
    assert device.ping_interval == 100  # ms, from the default 10 Hz

    node.destroy_node()


def test_configure_fails_without_the_device(ros):
    def open_device(port, baudrate):
        raise OSError("no such port")

    node = Ping2Node(open_device=open_device)

    assert node.trigger_configure() == TransitionCallbackReturn.FAILURE

    node.destroy_node()


def test_configure_fails_when_settings_are_rejected(ros):
    device = FakePing(reading())
    device.accept_settings = False
    node = Ping2Node(open_device=lambda port, baudrate: device)

    assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
    assert device.closed

    node.destroy_node()


def test_cleanup_closes_the_device(ros):
    device = FakePing(reading())
    node = Ping2Node(open_device=lambda port, baudrate: device)

    node.trigger_configure()
    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS

    assert device.closed

    node.destroy_node()
