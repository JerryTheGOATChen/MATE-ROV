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
"""People walking through the house on fixed loops, known only to the simulator."""

from __future__ import annotations

import math
from dataclasses import dataclass

from haunted_house.house import Vec3


@dataclass(frozen=True)
class Walker:
    """A person who walks a closed loop of waypoints at a steady speed, in straight lines.

    They start at the first waypoint, walk to each in turn, then back to the first and around again.
    """

    name: str
    path: tuple[tuple[float, float], ...]
    speed: float
    # Height of their head, where the ghost has to reach them
    head_z: float

    def __post_init__(self) -> None:
        if len(self.path) < 2:
            raise ValueError(f"{self.name}: a path needs at least two waypoints")
        if self.speed <= 0.0:
            raise ValueError(f"{self.name}: speed must be positive")

    def segments(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        return [(self.path[i], self.path[(i + 1) % len(self.path)]) for i in range(len(self.path))]

    @property
    def loop_length(self) -> float:
        return sum(math.dist(a, b) for a, b in self.segments())

    def position_at(self, t: float) -> Vec3:
        """Where the person's head is t seconds after the game starts."""
        distance = (self.speed * t) % self.loop_length
        for a, b in self.segments():
            length = math.dist(a, b)
            if distance <= length and length > 0.0:
                fraction = distance / length
                return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]), self.head_z)
            distance -= length
        return (self.path[0][0], self.path[0][1], self.head_z)
