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
"""The ghost's sensors: a scanning sonar, a depth camera that spots people and candy, and velocity and heading.

Everything here is plain numpy, so it can be tested without ROS. Positions are in the map frame unless
a name says otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from haunted_house.game import Game
from haunted_house.ghost import GhostState

# Half the width of a person, of their head, and of a piece of candy, as the sensors see them (m)
PERSON_HALF_WIDTH = 0.2
HEAD_HALF_SIZE = 0.125
CANDY_HALF_SIZE = 0.1


@dataclass(frozen=True)
class SonarConfig:
    # Revolutions per second, and beams per revolution
    revolutions_per_s: float = 1.0
    beams: int = 360
    min_range: float = 0.1
    max_range: float = 15.0
    noise_stddev: float = 0.02


@dataclass(frozen=True)
class CameraConfig:
    width: int = 128
    height: int = 72
    horizontal_fov_deg: float = 120.0
    max_range: float = 12.0
    # Where the camera sits in the ghost's frame: forward, left, up (m)
    offset: tuple[float, float, float] = (0.25, 0.0, 0.1)
    noise_stddev: float = 0.01

    @property
    def focal_length(self) -> float:
        """In pixels, the same horizontally and vertically."""
        return (self.width / 2.0) / math.tan(math.radians(self.horizontal_fov_deg) / 2.0)


@dataclass(frozen=True)
class MotionSensorConfig:
    """The ghost's velocity (like a DVL) and heading (like a compass)."""

    velocity_noise_stddev: float = 0.02
    # Each game draws a fixed velocity bias up to this size on each axis, so dead reckoning drifts
    max_velocity_bias: float = 0.015
    heading_noise_stddev_deg: float = 1.0
    yaw_rate_noise_stddev: float = 0.01


@dataclass
class Scene:
    """Every box a sensor can hit, with what each one is."""

    box_min: np.ndarray  # (M, 3)
    box_max: np.ndarray  # (M, 3)
    # For each box: "obstacle", "person" or "candy"
    kinds: list[str]
    # For people and candy, the index of the target in game.people or game.candies; -1 for obstacles
    target_index: list[int]


def build_scene(game: Game) -> Scene:
    house = game.house
    lows, highs, kinds, indices = [], [], [], []
    for box in house.obstacles:
        lows.append(box.min)
        highs.append(box.max)
        kinds.append("obstacle")
        indices.append(-1)
    # The floor and ceiling, as slabs a little bigger than the house
    (x0, x1), (y0, y1) = house.x_range, house.y_range
    for low_z, high_z in ((-1.0, 0.0), (house.ceiling_z, house.ceiling_z + 1.0)):
        lows.append((x0 - 1.0, y0 - 1.0, low_z))
        highs.append((x1 + 1.0, y1 + 1.0, high_z))
        kinds.append("obstacle")
        indices.append(-1)
    for i, person in enumerate(game.people):
        x, y, head_z = person.position
        lows.append((x - PERSON_HALF_WIDTH, y - PERSON_HALF_WIDTH, 0.0))
        highs.append((x + PERSON_HALF_WIDTH, y + PERSON_HALF_WIDTH, head_z + 0.15))
        kinds.append("person")
        indices.append(i)
    for i, candy in enumerate(game.candies):
        x, y, z = candy.position
        lows.append((x - CANDY_HALF_SIZE, y - CANDY_HALF_SIZE, z - CANDY_HALF_SIZE))
        highs.append((x + CANDY_HALF_SIZE, y + CANDY_HALF_SIZE, z + CANDY_HALF_SIZE))
        kinds.append("candy")
        indices.append(i)
    return Scene(np.array(lows, dtype=float), np.array(highs, dtype=float), kinds, indices)


def raycast(origin: np.ndarray, directions: np.ndarray, scene: Scene, skip: int = -1) -> tuple[np.ndarray, np.ndarray]:
    """Cast rays from one origin. A ray is origin + t * direction, for t >= 0.

    Returns, for each ray, the t of its first hit (inf if none) and the index of the box it hit
    (-1 if none). The box at index `skip` is ignored.
    """
    count = len(directions)
    keep = np.arange(len(scene.kinds)) != skip
    low = (scene.box_min[keep] - origin).astype(np.float32)
    high = (scene.box_max[keep] - origin).astype(np.float32)
    safe = np.where(np.abs(directions) < 1e-9, 1e-9, directions).astype(np.float32)
    inverse = 1.0 / safe

    # The slab method, one axis at a time to keep the arrays small
    t_near = np.zeros((count, low.shape[0]), dtype=np.float32)
    t_far = np.full((count, low.shape[0]), np.inf, dtype=np.float32)
    for axis in range(3):
        t1 = np.outer(inverse[:, axis], low[:, axis])
        t2 = np.outer(inverse[:, axis], high[:, axis])
        np.maximum(t_near, np.minimum(t1, t2), out=t_near)
        np.minimum(t_far, np.maximum(t1, t2), out=t_far)
    t_near[t_far < t_near] = np.inf

    nearest = t_near.argmin(axis=1)
    t = t_near[np.arange(count), nearest].astype(float)
    box = np.flatnonzero(keep)[nearest]
    return t, np.where(np.isfinite(t), box, -1)


def _yaw_matrix(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Turns a direction in the camera's optical frame (x right, y down, z forward) into the ghost's frame
# (x forward, y left, z up)
OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


class Sonar:
    """A mechanical scanning sonar: one beam at a time, sweeping a full circle around the ghost.

    Beams are horizontal, from the ghost's center, at angles counterclockwise from its nose.
    """

    def __init__(self, config: SonarConfig, rng: np.random.Generator) -> None:
        self.config = config
        self.rng = rng
        self.position = 0.0  # in beams, counting up forever
        self.angles = np.arange(config.beams) * (2.0 * math.pi / config.beams)

    def advance(self, ghost: GhostState, scene: Scene, dt: float) -> tuple[int, np.ndarray]:
        """Fire every beam the sweep passes during dt, from where the ghost is now.

        Returns the index of the first beam fired, and the measured ranges (inf for no echo).
        """
        start = int(math.floor(self.position + 1e-9))
        self.position += self.config.beams * self.config.revolutions_per_s * dt
        count = int(math.floor(self.position + 1e-9)) - start
        if count <= 0:
            return start % self.config.beams, np.empty(0)
        indices = (start + np.arange(count)) % self.config.beams
        body = np.stack([np.cos(self.angles[indices]), np.sin(self.angles[indices]), np.zeros(count)], axis=1)
        directions = body @ _yaw_matrix(ghost.yaw).T
        t, _ = raycast(np.array(ghost.position), directions, scene)
        ranges = t + self.rng.normal(0.0, self.config.noise_stddev, count)
        ranges = np.where((ranges > self.config.max_range) | ~np.isfinite(t), np.inf, np.maximum(ranges, self.config.min_range))
        return int(indices[0]), ranges


@dataclass(frozen=True)
class Detection:
    """Something the camera recognizes: where it is in the image, but not how far away."""

    kind: str  # "person" or "candy"
    # Bounding box in pixels: left, top, right, bottom. Around a person's head, or a whole candy
    box: tuple[float, float, float, float]


class Camera:
    """A forward-facing depth camera. Every pixel measures depth along the camera's axis, all at once."""

    def __init__(self, config: CameraConfig, rng: np.random.Generator) -> None:
        self.config = config
        self.rng = rng
        f = config.focal_length
        u, v = np.meshgrid(np.arange(config.width) + 0.5, np.arange(config.height) + 0.5)
        # Unit depth along the optical axis, so a ray's t is the pixel's depth
        optical = np.stack([(u - config.width / 2.0) / f, (v - config.height / 2.0) / f, np.ones_like(u)], axis=-1)
        self.body_rays = optical.reshape(-1, 3) @ OPTICAL_TO_BODY.T

    def origin(self, ghost: GhostState) -> np.ndarray:
        return np.array(ghost.position) + _yaw_matrix(ghost.yaw) @ np.array(self.config.offset)

    def render(self, ghost: GhostState, scene: Scene) -> np.ndarray:
        """A depth image in meters, row by row from the top left. inf where nothing is in range."""
        directions = self.body_rays @ _yaw_matrix(ghost.yaw).T
        depth, _ = raycast(self.origin(ghost), directions, scene)
        depth = depth + self.rng.normal(0.0, self.config.noise_stddev, depth.shape)
        depth = np.where(depth > self.config.max_range, np.inf, depth)
        return depth.reshape(self.config.height, self.config.width).astype(np.float32)

    def detect(self, ghost: GhostState, scene: Scene) -> list[Detection]:
        """People and candy in view and not hidden behind something.

        Something counts as in view when the point the ghost must reach (a person's head, a candy's
        center) is inside the image, within range, and has a clear line of sight.
        """
        config = self.config
        f = config.focal_length
        origin = self.origin(ghost)
        to_optical = (_yaw_matrix(ghost.yaw) @ OPTICAL_TO_BODY).T
        detections = []
        for index, kind in enumerate(scene.kinds):
            if kind == "obstacle":
                continue
            low, high = scene.box_min[index], scene.box_max[index]
            center = (low + high) / 2.0
            aim = np.array([center[0], center[1], high[2] - 0.15]) if kind == "person" else center
            offset = aim - origin
            distance = float(np.linalg.norm(offset))
            optical = to_optical @ offset
            if optical[2] <= 0.05 or distance > config.max_range:
                continue
            u = f * optical[0] / optical[2] + config.width / 2.0
            v = f * optical[1] / optical[2] + config.height / 2.0
            if not (0.0 <= u < config.width and 0.0 <= v < config.height):
                continue
            # Line of sight to the aim point, ignoring the target's own box
            t, _ = raycast(origin, offset[None, :], scene, skip=index)
            if t[0] < 1.0 - 0.05 / max(distance, 1e-6):
                continue
            if kind == "person":
                # The camera spots faces: the box is around the head, where the ghost has to reach
                low, high = aim - HEAD_HALF_SIZE, aim + HEAD_HALF_SIZE
            detections.append(Detection(kind, self._bounding_box(origin, to_optical, low, high)))
        return detections

    def _bounding_box(self, origin, to_optical, low, high) -> tuple[float, float, float, float]:
        config = self.config
        f = config.focal_length
        corners = np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
        optical = (corners - origin) @ to_optical.T
        optical = optical[optical[:, 2] > 0.05]
        u = f * optical[:, 0] / optical[:, 2] + config.width / 2.0
        v = f * optical[:, 1] / optical[:, 2] + config.height / 2.0
        return (
            float(np.clip(u.min(), 0.0, config.width)),
            float(np.clip(v.min(), 0.0, config.height)),
            float(np.clip(u.max(), 0.0, config.width)),
            float(np.clip(v.max(), 0.0, config.height)),
        )


class MotionSensors:
    """Velocity in the ghost's frame, and heading, each with noise. The velocity also has a per-game bias."""

    def __init__(self, config: MotionSensorConfig, rng: np.random.Generator) -> None:
        self.config = config
        self.rng = rng
        self.new_game()

    def new_game(self) -> None:
        self.bias = self.rng.uniform(-self.config.max_velocity_bias, self.config.max_velocity_bias, 3)

    def velocity(self, ghost: GhostState) -> tuple[np.ndarray, float]:
        """Measured velocity (forward, left, up) and yaw rate."""
        noise = self.rng.normal(0.0, self.config.velocity_noise_stddev, 3)
        yaw_rate = ghost.yaw_rate + self.rng.normal(0.0, self.config.yaw_rate_noise_stddev)
        return np.array(ghost.velocity) + self.bias + noise, yaw_rate

    def heading(self, ghost: GhostState) -> float:
        """Measured yaw, counterclockwise from east (the map's x axis)."""
        noisy = ghost.yaw + math.radians(self.rng.normal(0.0, self.config.heading_noise_stddev_deg))
        return math.atan2(math.sin(noisy), math.cos(noisy))
