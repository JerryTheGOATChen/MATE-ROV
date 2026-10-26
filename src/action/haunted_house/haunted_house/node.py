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
"""The haunted house simulator: runs the game, moves the ghost, and simulates its sensors.

The ghost's brain gets only what a real vehicle would: velocity and heading, a scanning sonar, a depth
camera with detections, and the floor plan. Where everything really is goes on /spectator topics, for
Foxglove and judging only.
"""

from __future__ import annotations

import math
import os
import random

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, TwistStamped
from haunted_house_msgs.msg import Box, BoxArray, GameState, Target, TargetArray
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import ImageMarker, MarkerArray

from haunted_house.config import load_game
from haunted_house.game import Game, Phase
from haunted_house.ghost import Command
from haunted_house.markers import build_markers
from haunted_house.sensors import (
    Camera,
    CameraConfig,
    MotionSensorConfig,
    MotionSensors,
    Sonar,
    SonarConfig,
    build_scene,
)

# The camera's optical frame (x right, y down, z forward) relative to the ghost's (x forward, y left, z up)
_OPTICAL_ROTATION = (-0.5, 0.5, -0.5, 0.5)


def _point(position) -> Point:
    return Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))


def _yaw_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class HauntedHouseNode(Node):
    def __init__(self) -> None:
        super().__init__("haunted_house")
        default_house = os.path.join(get_package_share_directory("haunted_house"), "config", "haunted_house.yaml")
        self.declare_parameter("house_file", default_house)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("camera_rate_hz", 10.0)
        self.declare_parameter("motion_rate_hz", 25.0)
        self.declare_parameter("spectator_rate_hz", 10.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("map_resolution_m", 0.1)
        # Negative for a different game every reset; set it to replay the same shuffles and noise
        self.declare_parameter("seed", -1)
        # Publish the floor plan on /map and /haunted_house/obstacles
        self.declare_parameter("publish_floor_plan", True)
        # Turn off to draw your own ghost
        self.declare_parameter("draw_ghost", True)

        house_file = self.get_parameter("house_file").value
        self.dt = 1.0 / self.get_parameter("update_rate_hz").value
        self.motion_every = max(1, round(self.get_parameter("update_rate_hz").value / self.get_parameter("motion_rate_hz").value))
        self.cmd_vel_timeout = Duration(seconds=self.get_parameter("cmd_vel_timeout_s").value)
        self.draw_ghost = self.get_parameter("draw_ghost").value
        self.map_frame = "map"
        self.base_frame = "base_link"

        seed = self.get_parameter("seed").value
        self.rng = random.Random(None if seed < 0 else seed)
        self.np_rng = np.random.default_rng(None if seed < 0 else seed)

        self.game: Game = load_game(house_file)
        self.sonar = Sonar(SonarConfig(), self.np_rng)
        self.camera = Camera(CameraConfig(), self.np_rng)
        self.motion = MotionSensors(MotionSensorConfig(), self.np_rng)
        self.command = Command()
        self.command_time: Time | None = None
        self.ticks = 0
        self.sonar_buffer: list[float] = []
        self.sonar_first_beam = 0
        self.sonar_stamp = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # What the ghost's brain may use
        self.velocity_pub = self.create_publisher(TwistStamped, "ghost/velocity", 10)
        self.imu_pub = self.create_publisher(Imu, "ghost/imu", 10)
        self.sonar_pub = self.create_publisher(LaserScan, "sonar/scan", 10)
        self.depth_pub = self.create_publisher(Image, "camera/depth", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "camera/camera_info", 10)
        self.detections_pub = self.create_publisher(Detection2DArray, "camera/detections", 10)
        self.overlay_pub = self.create_publisher(ImageMarker, "camera/detections_overlay", 10)
        self.state_pub = self.create_publisher(GameState, "haunted_house/state", 10)
        self.start_pub = self.create_publisher(PoseStamped, "haunted_house/start_pose", latched)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", latched)
        self.obstacles_pub = self.create_publisher(BoxArray, "haunted_house/obstacles", latched)
        # For spectators only
        self.truth_pub = self.create_publisher(PoseStamped, "spectator/ghost_pose", 10)
        self.people_pub = self.create_publisher(TargetArray, "spectator/people", 10)
        self.candies_pub = self.create_publisher(TargetArray, "spectator/candies", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "spectator/markers", 10)

        self.create_subscription(TwistStamped, "cmd_vel", self.on_cmd_vel, 10)
        self.create_service(Trigger, "haunted_house/reset", self.on_reset)

        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_sensor_mounts()
        self.publish_floor_plan = self.get_parameter("publish_floor_plan").value
        self.map_msg = self.make_map(self.get_parameter("map_resolution_m").value)
        self.obstacles_msg = self.make_obstacles()

        self.new_game()
        self.create_timer(self.dt, self.update)
        self.create_timer(1.0 / self.get_parameter("camera_rate_hz").value, self.publish_camera)
        self.create_timer(1.0 / self.get_parameter("spectator_rate_hz").value, self.publish_world)
        # Republish the latched topics now and then, for anything that missed them
        self.create_timer(2.0, self.publish_house)
        self.publish_house()

        self.get_logger().info(
            f"Loaded {house_file}: {len(self.game.people)} people and {len(self.game.candies)} candies. "
            f"The game starts in {self.game.rules.start_delay_s:.0f} s"
        )

    # ----------------------------------------------------------------------------------------------
    # Game

    def new_game(self) -> None:
        self.game.reset(self.rng)
        self.motion.new_game()
        self.command = Command()
        self.command_time = None

    def on_cmd_vel(self, msg: TwistStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f"Ignoring cmd_vel in frame '{msg.header.frame_id}': commands must be in '{self.base_frame}'",
                throttle_duration_sec=5.0,
            )
            return
        linear = msg.twist.linear
        self.command = Command((linear.x, linear.y, linear.z), msg.twist.angular.z)
        self.command_time = self.get_clock().now()

    def on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.new_game()
        self.publish_house()
        self.get_logger().info(f"Reset: the game starts in {self.game.rules.start_delay_s:.0f} s")
        response.success = True
        response.message = "The house is haunted again"
        return response

    def update(self) -> None:
        now = self.get_clock().now()
        fresh = self.command_time is not None and now - self.command_time <= self.cmd_vel_timeout
        phase = self.game.phase

        for catch in self.game.step(self.dt, self.command if fresh else Command()):
            verb = "Haunted" if catch.is_person else "Ate"
            self.get_logger().info(f"{verb} {catch.target.name}: +{catch.target.points}, score {self.game.score}")

        if phase == Phase.WAITING and self.game.phase == Phase.RUNNING:
            self.get_logger().info(f"Go! {self.game.rules.duration_s:.0f} seconds on the clock")
        if phase == Phase.RUNNING and self.game.phase == Phase.FINISHED:
            self.get_logger().info(
                f"Game over! Final score {self.game.score}: {self.game.people_haunted} people haunted, "
                f"{self.game.candies_eaten} candies eaten, {self.game.bonks} bonks"
            )
            self.publish_world()

        self.ticks += 1
        self.update_sonar(now)
        if self.ticks % self.motion_every == 0:
            self.publish_motion(now)

    # ----------------------------------------------------------------------------------------------
    # Sensors

    def update_sonar(self, now: Time) -> None:
        first, ranges = self.sonar.advance(self.game.ghost, build_scene(self.game), self.dt)
        if ranges.size == 0:
            return
        if not self.sonar_buffer:
            self.sonar_first_beam = first
            self.sonar_stamp = now
        self.sonar_buffer.extend(float(r) for r in ranges)
        # Send a tenth of a turn at a time, so the brain gets beams soon after they are measured
        if len(self.sonar_buffer) >= self.sonar.config.beams // 10:
            config = self.sonar.config
            increment = 2.0 * math.pi / config.beams
            scan = LaserScan()
            scan.header.stamp = self.sonar_stamp.to_msg()
            scan.header.frame_id = "sonar_link"
            scan.angle_min = self.sonar_first_beam * increment
            scan.angle_increment = increment
            scan.angle_max = scan.angle_min + (len(self.sonar_buffer) - 1) * increment
            scan.time_increment = 1.0 / (config.beams * config.revolutions_per_s)
            scan.scan_time = 1.0 / config.revolutions_per_s
            scan.range_min = config.min_range
            scan.range_max = config.max_range
            scan.ranges = self.sonar_buffer
            self.sonar_pub.publish(scan)
            self.sonar_buffer = []

    def publish_motion(self, now: Time) -> None:
        stamp = now.to_msg()
        velocity, yaw_rate = self.motion.velocity(self.game.ghost)
        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = (float(v) for v in velocity)
        twist.twist.angular.z = float(yaw_rate)
        self.velocity_pub.publish(twist)

        imu = Imu()
        imu.header = twist.header
        q = _yaw_quaternion(self.motion.heading(self.game.ghost))
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        imu.angular_velocity.z = float(yaw_rate)
        heading_variance = math.radians(self.motion.config.heading_noise_stddev_deg) ** 2
        imu.orientation_covariance = [heading_variance, 0.0, 0.0, 0.0, heading_variance, 0.0, 0.0, 0.0, heading_variance]
        # No accelerometer
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)

    def publish_camera(self) -> None:
        stamp = self.get_clock().now().to_msg()
        scene = build_scene(self.game)
        config = self.camera.config
        depth = self.camera.render(self.game.ghost, scene)

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "camera_optical"
        image.height, image.width = depth.shape
        image.encoding = "32FC1"
        image.is_bigendian = 0
        image.step = image.width * 4
        image.data = depth.astype("<f4").tobytes()
        self.depth_pub.publish(image)

        f = config.focal_length
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = image.height, image.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [f, 0.0, config.width / 2.0, 0.0, f, config.height / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, config.width / 2.0, 0.0, 0.0, f, config.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)

        detections = Detection2DArray()
        detections.header = image.header
        overlay = ImageMarker()
        overlay.header = image.header
        overlay.ns = "detections"
        overlay.type = ImageMarker.LINE_LIST
        overlay.action = ImageMarker.ADD
        overlay.scale = 1.0
        for found in self.camera.detect(self.game.ghost, scene):
            left, top, right, bottom = found.box
            detection = Detection2D()
            detection.header = image.header
            detection.bbox.center.position.x = (left + right) / 2.0
            detection.bbox.center.position.y = (top + bottom) / 2.0
            detection.bbox.size_x = right - left
            detection.bbox.size_y = bottom - top
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = found.kind
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)
            detections.detections.append(detection)

            color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0) if found.kind == "person" else ColorRGBA(r=0.3, g=1.0, b=0.3, a=1.0)
            corners = [(left, top), (right, top), (right, bottom), (left, bottom)]
            for i in range(4):
                for u, v in (corners[i], corners[(i + 1) % 4]):
                    overlay.points.append(Point(x=u, y=v, z=0.0))
                    overlay.outline_colors.append(color)
        self.detections_pub.publish(detections)
        self.overlay_pub.publish(overlay)

    def publish_sensor_mounts(self) -> None:
        sonar = TransformStamped()
        sonar.header.stamp = self.get_clock().now().to_msg()
        sonar.header.frame_id = self.base_frame
        sonar.child_frame_id = "sonar_link"
        sonar.transform.rotation.w = 1.0

        camera = TransformStamped()
        camera.header = sonar.header
        camera.child_frame_id = "camera_optical"
        offset = self.camera.config.offset
        camera.transform.translation.x, camera.transform.translation.y, camera.transform.translation.z = offset
        r = camera.transform.rotation
        r.x, r.y, r.z, r.w = _OPTICAL_ROTATION
        self.static_tf.sendTransform([sonar, camera])

    # ----------------------------------------------------------------------------------------------
    # Spectators, and the house

    def publish_world(self) -> None:
        stamp = self.get_clock().now().to_msg()
        game = self.game

        truth = PoseStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = self.map_frame
        truth.pose.position = _point(game.ghost.position)
        o = truth.pose.orientation
        o.x, o.y, o.z, o.w = _yaw_quaternion(game.ghost.yaw)
        self.truth_pub.publish(truth)

        for publisher, targets in ((self.people_pub, game.people), (self.candies_pub, game.candies)):
            msg = TargetArray()
            msg.header = truth.header
            msg.targets = [Target(id=t.id, name=t.name, position=_point(t.position), points=t.points) for t in targets]
            publisher.publish(msg)

        state = GameState()
        state.header = truth.header
        state.phase = int(game.phase)
        state.time_remaining_s = game.time_remaining_s
        state.score = game.score
        state.candies_eaten = game.candies_eaten
        state.people_haunted = game.people_haunted
        state.candies_left = len(game.candies)
        state.people_left = len(game.people)
        state.bonks = game.bonks
        self.state_pub.publish(state)

        self.markers_pub.publish(build_markers(game, self.map_frame, stamp, self.draw_ghost))

    def publish_house(self) -> None:
        stamp = self.get_clock().now().to_msg()
        start = PoseStamped()
        start.header.stamp = stamp
        start.header.frame_id = self.map_frame
        start.pose.position = _point(self.game.ghost_start.position)
        o = start.pose.orientation
        o.x, o.y, o.z, o.w = _yaw_quaternion(self.game.ghost_start.yaw)
        self.start_pub.publish(start)

        if self.publish_floor_plan:
            self.map_msg.header.stamp = stamp
            self.obstacles_msg.header.stamp = stamp
            self.map_pub.publish(self.map_msg)
            self.obstacles_pub.publish(self.obstacles_msg)

    def make_map(self, resolution: float) -> OccupancyGrid:
        origin_x, origin_y, width, height, cells = self.game.house.occupancy_grid(resolution)
        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = cells
        return grid

    def make_obstacles(self) -> BoxArray:
        msg = BoxArray()
        msg.header.frame_id = self.map_frame
        msg.boxes = [Box(name=b.name, min=_point(b.min), max=_point(b.max)) for b in self.game.house.obstacles]
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HauntedHouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
