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
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.lifecycle import TransitionCallbackReturn

from pressure_driver.bar30_node import Bar30Node, depth_from_pressure


class FakeSensor:
    def __init__(self, pressure_pa):
        self.pressure_pa = pressure_pa
        self.fail = False
        self.closed = False

    def read(self):
        if self.fail:
            raise OSError("bus error")
        return self.pressure_pa, 20.0

    def close(self):
        self.closed = True


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(node, listener, condition, timeout_s=2.0):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(listener)
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while not condition() and node.get_clock().now().nanoseconds < deadline:
        executor.spin_once(timeout_sec=0.05)
    executor.shutdown()


def test_depth_from_pressure():
    assert depth_from_pressure(101325.0, 101325.0, 997.0) == pytest.approx(0.0)
    # One metre of fresh water adds 997 * 9.80665 Pa.
    assert depth_from_pressure(
        101325.0 + 997.0 * 9.80665, 101325.0, 997.0
    ) == pytest.approx(1.0)


def test_publishes_negative_depth_as_z(ros):
    sensor = FakeSensor(101325.0 + 2.0 * 997.0 * 9.80665)
    node = Bar30Node(open_sensor=lambda bus: sensor)
    node.set_parameters([rclpy.parameter.Parameter("rate_hz", value=50.0)])
    listener = rclpy.create_node("listener")
    received = []
    listener.create_subscription(Odometry, "depth", received.append, 10)

    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    spin_until(node, listener, lambda: received)

    assert received, "no depth message arrived"
    msg = received[0]
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    assert msg.pose.pose.position.z == pytest.approx(-2.0)
    assert msg.pose.covariance[14] == pytest.approx(0.0004)

    node.destroy_node()
    listener.destroy_node()


def test_failed_read_publishes_nothing(ros):
    sensor = FakeSensor(101325.0)
    sensor.fail = True
    node = Bar30Node(open_sensor=lambda bus: sensor)
    node.set_parameters([rclpy.parameter.Parameter("rate_hz", value=50.0)])
    listener = rclpy.create_node("listener")
    received = []
    listener.create_subscription(Odometry, "depth", received.append, 10)

    node.trigger_configure()
    node.trigger_activate()
    spin_until(node, listener, lambda: received, timeout_s=0.5)

    assert not received

    node.destroy_node()
    listener.destroy_node()


def test_configure_fails_without_the_sensor(ros):
    def open_sensor(bus):
        raise OSError("no such bus")

    node = Bar30Node(open_sensor=open_sensor)

    assert node.trigger_configure() == TransitionCallbackReturn.FAILURE

    node.destroy_node()


def test_configure_rejects_bad_parameters(ros):
    node = Bar30Node(open_sensor=lambda bus: FakeSensor(101325.0))
    node.set_parameters([rclpy.parameter.Parameter("rate_hz", value=0.0)])

    assert node.trigger_configure() == TransitionCallbackReturn.FAILURE

    node.destroy_node()


def test_cleanup_closes_the_sensor(ros):
    sensor = FakeSensor(101325.0)
    node = Bar30Node(open_sensor=lambda bus: sensor)

    node.trigger_configure()
    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS

    assert sensor.closed

    node.destroy_node()
