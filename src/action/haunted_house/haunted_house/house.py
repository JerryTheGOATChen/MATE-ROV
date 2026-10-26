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
"""The house's geometry: axis-aligned boxes the ghost cannot fly through."""

from __future__ import annotations

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Box:
    """An axis-aligned box in the map frame."""

    name: str
    min: Vec3
    max: Vec3

    def distance_to(self, point: Vec3) -> float:
        """Distance from a point to the box's surface, or 0 inside it."""
        squared = 0.0
        for value, low, high in zip(point, self.min, self.max):
            gap = max(low - value, 0.0, value - high)
            squared += gap * gap
        return math.sqrt(squared)


@dataclass(frozen=True)
class House:
    """The house: its extent, height, obstacles and room labels."""

    x_range: tuple[float, float]
    y_range: tuple[float, float]
    ceiling_z: float
    walls: tuple[Box, ...]
    furniture: tuple[Box, ...]
    rooms: tuple[tuple[str, float, float], ...] = ()

    @property
    def obstacles(self) -> tuple[Box, ...]:
        return self.walls + self.furniture

    def collides(self, point: Vec3, radius: float) -> bool:
        """Whether a sphere at the point touches an obstacle, the floor or the ceiling."""
        if point[2] < radius or point[2] > self.ceiling_z - radius:
            return True
        return any(box.distance_to(point) < radius for box in self.obstacles)

    def occupancy_grid(self, resolution: float, margin: float = 0.5) -> tuple[float, float, int, int, list[int]]:
        """A 2D grid of the house seen from above: 100 where any obstacle stands, at any height, else 0.

        Returns the grid's origin x, y, its width and height in cells, and the cells row by row from
        the origin, as nav_msgs/OccupancyGrid lays them out.
        """
        origin_x = self.x_range[0] - margin
        origin_y = self.y_range[0] - margin
        width = int(math.ceil((self.x_range[1] - self.x_range[0] + 2 * margin) / resolution))
        height = int(math.ceil((self.y_range[1] - self.y_range[0] + 2 * margin) / resolution))
        cells = [0] * (width * height)
        for box in self.obstacles:
            # Every cell whose center lies inside the box's footprint
            first_col = max(0, int(math.ceil((box.min[0] - origin_x) / resolution - 0.5)))
            last_col = min(width - 1, int(math.floor((box.max[0] - origin_x) / resolution - 0.5)))
            first_row = max(0, int(math.ceil((box.min[1] - origin_y) / resolution - 0.5)))
            last_row = min(height - 1, int(math.floor((box.max[1] - origin_y) / resolution - 0.5)))
            for row in range(first_row, last_row + 1):
                for col in range(first_col, last_col + 1):
                    cells[row * width + col] = 100
        return origin_x, origin_y, width, height, cells
