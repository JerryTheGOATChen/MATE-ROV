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
"""The ghost's motion: it follows body-frame velocity commands with a lag, like the ROV."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from haunted_house.house import House, Vec3


@dataclass(frozen=True)
class GhostModel:
    """How the ghost responds to commands. The defaults match the ROV's kinematic simulator."""

    # Time constant of the lag from commanded to actual velocity (s)
    response_time: float = 0.3
    # Limits for surge (forward), sway (left) and heave (up), m/s
    max_linear_speed: Vec3 = (1.0, 0.6, 0.6)
    # Limit for yaw rate, rad/s
    max_yaw_rate: float = 1.5
    # The ghost is a sphere this size as far as walls and furniture are concerned (m)
    radius: float = 0.3


@dataclass(frozen=True)
class Command:
    """A velocity command in the ghost's own frame: x forward, y left, z up, and yaw rate."""

    linear: Vec3 = (0.0, 0.0, 0.0)
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class GhostState:
    position: Vec3
    yaw: float
    # Velocity in the ghost's own frame, and yaw rate
    velocity: Vec3 = (0.0, 0.0, 0.0)
    yaw_rate: float = 0.0
    # Whether the last step ran into something
    bonked: bool = field(default=False, compare=False)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def body_to_world(velocity: Vec3, yaw: float) -> Vec3:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (cos_yaw * velocity[0] - sin_yaw * velocity[1], sin_yaw * velocity[0] + cos_yaw * velocity[1], velocity[2])


def world_to_body(velocity: Vec3, yaw: float) -> Vec3:
    return body_to_world(velocity, -yaw)


def step(state: GhostState, command: Command, model: GhostModel, house: House, dt: float) -> GhostState:
    """Advance the ghost by dt seconds under a command.

    The walls are lined with salt, which ghosts cannot cross: when a move would touch an obstacle,
    the ghost slides along it, keeping only the parts of the move that stay clear.
    """
    target = tuple(_clamp(value, limit) for value, limit in zip(command.linear, model.max_linear_speed))
    target_yaw_rate = _clamp(command.yaw_rate, model.max_yaw_rate)
    blend = 1.0 - math.exp(-dt / model.response_time) if model.response_time > 0.0 else 1.0

    velocity = tuple(v + blend * (t - v) for v, t in zip(state.velocity, target))
    yaw_rate = state.yaw_rate + blend * (target_yaw_rate - state.yaw_rate)

    # Move along the current heading, one axis at a time so the ghost slides along what it hits
    world_velocity = list(body_to_world(velocity, state.yaw))
    position = list(state.position)
    bonked = False
    for axis in range(3):
        moved = list(position)
        moved[axis] += world_velocity[axis] * dt
        if house.collides(tuple(moved), model.radius):
            world_velocity[axis] = 0.0
            bonked = True
        else:
            position = moved

    return replace(
        state,
        position=tuple(position),
        yaw=_normalize_angle(state.yaw + yaw_rate * dt),
        velocity=world_to_body(tuple(world_velocity), state.yaw),
        yaw_rate=yaw_rate,
        bonked=bonked,
    )
