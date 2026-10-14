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
"""Runs missions through Nav2 and records what happens."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from rov_mission_runner.mission import Mission, Waypoint
from rov_mission_runner.scoring import Pose, Telemetry, yaw_from_quaternion

_OUTCOMES = {
    TaskResult.SUCCEEDED: "succeeded",
    TaskResult.FAILED: "failed",
    TaskResult.CANCELED: "canceled",
}
_ROBOT_FRAME = "base_link"
_SAMPLE_PERIOD = 0.05  # s between samples of the robot's position
_STOP_TIMEOUT = 1.0  # s for the controller's stop command to arrive after a cancel


class MissionRunner:
    """Sends missions to Nav2 through its simple commander, recording telemetry while they run.

    A mission with one waypoint is sent as NavigateToPose, one with several as
    NavigateThroughPoses. Telemetry comes from TF (the robot's pose), the planner's
    `plan` topic and the controller's `cmd_vel` topic.

    With `truth_topic`, a simulator's ground-truth odometry, the robot's pose comes from there
    instead, so a run is scored on where the vehicle really went rather than where localization
    thought it was. The ground truth must be in the missions' frame.
    """

    def __init__(self, frame_id: str, truth_topic: str | None = None):
        self._frame_id = frame_id
        self._telemetry: Telemetry | None = None  # recording while set
        self._truth: Pose | None = None  # latest ground truth, with truth_topic

        self._navigator = BasicNavigator("mission_runner")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._navigator)
        self._navigator.create_subscription(Path, "plan", self._on_plan, 10)
        self._navigator.create_subscription(
            TwistStamped, "cmd_vel", self._on_cmd_vel, 10
        )
        if truth_topic:
            self._navigator.create_subscription(
                Odometry, truth_topic, self._on_truth, 10
            )
        self._truth_topic = truth_topic
        self._navigator.create_timer(_SAMPLE_PERIOD, self._sample_position)

    @property
    def logger(self):
        return self._navigator.get_logger()

    def close(self) -> None:
        self._navigator.destroy_node()

    def wait_for_nav2(self) -> None:
        # There is no AMCL to wait for: the vehicle, or the simulator, provides localization
        self._navigator.waitUntilNav2Active(localizer="robot_localization")

    def run(self, mission: Mission) -> Telemetry:
        """Navigate a mission, canceling it when scheduled or when it runs out of time."""
        telemetry = Telemetry()
        self._telemetry = telemetry
        try:
            start = time.monotonic()
            if not self._send_goal(mission):
                telemetry.outcome = "rejected"
                return telemetry

            self._await_completion(mission, start)
            telemetry.outcome = _OUTCOMES.get(self._navigator.getResult(), "unknown")
            telemetry.duration = time.monotonic() - start
            telemetry.final_pose = self._robot_pose()
            self._log_localization_error()
            if mission.expect == "canceled":
                self._await_stop(telemetry)
        finally:
            self._telemetry = None
        return telemetry

    def _send_goal(self, mission: Mission) -> bool:
        poses = [self._pose_stamped(waypoint) for waypoint in mission.waypoints]
        if len(poses) == 1:
            return self._navigator.goToPose(poses[0])
        return self._navigator.goThroughPoses(poses)

    def _await_completion(self, mission: Mission, start: float) -> None:
        cancel_after = (
            mission.cancel_after
            if mission.cancel_after is not None
            else mission.criteria.timeout
        )
        canceled = False
        try:
            while not self._navigator.isTaskComplete():
                if not canceled and time.monotonic() - start >= cancel_after:
                    self._navigator.cancelTask()
                    canceled = True
        except KeyboardInterrupt:
            # Never leave the vehicle driving toward a goal nobody is watching
            self._navigator.cancelTask()
            raise

    def _robot_pose(self) -> Pose | None:
        if self._truth_topic:
            return self._truth
        return self._estimated_pose()

    def _estimated_pose(self) -> Pose | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id, _ROBOT_FRAME, Time()
            )
        except TransformException:
            return None
        return _to_pose(transform.transform.translation, transform.transform.rotation)

    def _on_truth(self, msg: Odometry) -> None:
        self._truth = _to_pose(msg.pose.pose.position, msg.pose.pose.orientation)

    def _log_localization_error(self) -> None:
        estimate = self._estimated_pose()
        if not self._truth_topic or self._truth is None or estimate is None:
            return
        error = math.dist(
            (estimate.x, estimate.y, estimate.z),
            (self._truth.x, self._truth.y, self._truth.z),
        )
        self.logger.info(
            f"Localization was {error:.2f} m from the ground truth at the end"
        )

    def _pose_stamped(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = waypoint.z
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def _await_stop(self, telemetry: Telemetry) -> None:
        """Spin until the controller's stop command arrives, or the timeout passes."""
        deadline = time.monotonic() + _STOP_TIMEOUT
        while (
            not telemetry.stopped() and (remaining := deadline - time.monotonic()) > 0.0
        ):
            rclpy.spin_once(self._navigator, timeout_sec=remaining)

    def _on_plan(self, msg: Path) -> None:
        if self._telemetry is None:
            return
        if msg.header.frame_id != self._frame_id:
            self.logger.warning(
                f"Ignoring plans in frame '{msg.header.frame_id}': missions are in '{self._frame_id}'",
                once=True,
            )
            return
        self._telemetry.record_plan(
            [
                (p.pose.position.x, p.pose.position.y, p.pose.position.z)
                for p in msg.poses
            ]
        )

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        if self._telemetry is not None:
            twist = msg.twist
            self._telemetry.record_command(
                (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)
            )

    def _sample_position(self) -> None:
        if self._telemetry is None:
            return
        pose = self._robot_pose()
        if pose is not None:
            self._telemetry.positions.append((pose.x, pose.y, pose.z))


def _to_pose(position, rotation) -> Pose:
    yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
    return Pose(position.x, position.y, position.z, yaw)
