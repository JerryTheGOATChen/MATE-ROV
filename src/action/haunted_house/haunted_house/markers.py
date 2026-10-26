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
"""Foxglove-friendly markers of the house, the people, the candies and the ghost."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from haunted_house.game import Game, Phase

_WALL = ColorRGBA(r=0.22, g=0.16, b=0.30, a=1.0)
_FURNITURE = ColorRGBA(r=0.40, g=0.24, b=0.12, a=0.95)
_FLOOR = ColorRGBA(r=0.08, g=0.06, b=0.10, a=1.0)
_GHOST = ColorRGBA(r=0.95, g=0.95, b=1.0, a=0.85)
_EYES = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
_TEXT = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
_CANDY = (
    ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0),
    ColorRGBA(r=0.6, g=0.2, b=0.9, a=1.0),
    ColorRGBA(r=0.2, g=0.9, b=0.3, a=1.0),
    ColorRGBA(r=1.0, g=0.9, b=0.2, a=1.0),
)
_PEOPLE = (
    ColorRGBA(r=0.85, g=0.3, b=0.3, a=1.0),
    ColorRGBA(r=0.3, g=0.5, b=0.9, a=1.0),
    ColorRGBA(r=0.9, g=0.8, b=0.3, a=1.0),
    ColorRGBA(r=0.4, g=0.8, b=0.5, a=1.0),
    ColorRGBA(r=0.8, g=0.4, b=0.8, a=1.0),
    ColorRGBA(r=0.4, g=0.8, b=0.8, a=1.0),
)
_SKIN = ColorRGBA(r=0.95, g=0.8, b=0.65, a=1.0)


class MarkerBuilder:
    def __init__(self, frame_id: str, stamp: Time) -> None:
        self.frame_id = frame_id
        self.stamp = stamp
        self.array = MarkerArray()
        # Clear what was drawn last time, so reached targets disappear. It needs a frame too, or
        # Foxglove lists an unnamed frame it cannot place
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        self.array.markers.append(clear)

    def add(self, ns: str, marker_type: int, position, scale, color: ColorRGBA, text: str = "") -> None:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.stamp
        marker.ns = ns
        marker.id = len(self.array.markers)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = (float(s) for s in scale)
        marker.color = color
        marker.text = text
        self.array.markers.append(marker)

    def box(self, ns: str, low, high, color: ColorRGBA) -> None:
        center = [(a + b) / 2 for a, b in zip(low, high)]
        size = [b - a for a, b in zip(low, high)]
        self.add(ns, Marker.CUBE, center, size, color)


def build_markers(game: Game, frame_id: str, stamp: Time, draw_ghost: bool) -> MarkerArray:
    """Everything as it really is, for spectators. The ghost's brain must not read these."""
    house = game.house
    builder = MarkerBuilder(frame_id, stamp)

    (x0, x1), (y0, y1) = house.x_range, house.y_range
    builder.box("floor", (x0, y0, -0.05), (x1, y1, 0.0), _FLOOR)
    for wall in house.walls:
        builder.box("walls", wall.min, wall.max, _WALL)
    for item in house.furniture:
        builder.box("furniture", item.min, item.max, _FURNITURE)
    for name, x, y in house.rooms:
        builder.add("rooms", Marker.TEXT_VIEW_FACING, (x, y, 0.05), (0, 0, 0.4), _TEXT, text=name)

    for candy in game.candies:
        builder.add("candies", Marker.SPHERE, candy.position, (0.2, 0.2, 0.2), _CANDY[candy.id % len(_CANDY)])

    for person in game.people:
        x, y, head_z = person.position
        body_height = head_z - 0.15
        color = _PEOPLE[person.id % len(_PEOPLE)]
        builder.add("people", Marker.CYLINDER, (x, y, body_height / 2), (0.4, 0.4, body_height), color)
        builder.add("people", Marker.SPHERE, (x, y, head_z), (0.25, 0.25, 0.25), _SKIN)
        builder.add("people", Marker.TEXT_VIEW_FACING, (x, y, head_z + 0.35), (0, 0, 0.25), _TEXT, text=person.name)

    if draw_ghost:
        # Where the ghost really is: a head, a sheet, and eyes facing the way it faces
        x, y, z = game.ghost.position
        cos_yaw, sin_yaw = math.cos(game.ghost.yaw), math.sin(game.ghost.yaw)

        def at(forward: float, left: float, up: float):
            return (x + cos_yaw * forward - sin_yaw * left, y + sin_yaw * forward + cos_yaw * left, z + up)

        builder.add("ghost", Marker.SPHERE, at(0.0, 0.0, 0.1), (0.5, 0.5, 0.5), _GHOST)
        builder.add("ghost", Marker.CYLINDER, at(0.0, 0.0, -0.15), (0.55, 0.55, 0.4), _GHOST)
        builder.add("ghost", Marker.SPHERE, at(0.22, 0.09, 0.17), (0.08, 0.08, 0.12), _EYES)
        builder.add("ghost", Marker.SPHERE, at(0.22, -0.09, 0.17), (0.08, 0.08, 0.12), _EYES)

    builder.add(
        "scoreboard",
        Marker.TEXT_VIEW_FACING,
        ((x0 + x1) / 2, y1 + 1.0, house.ceiling_z + 0.5),
        (0, 0, 0.6),
        _TEXT,
        text=scoreboard(game),
    )
    return builder.array


def scoreboard(game: Game) -> str:
    if game.phase == Phase.WAITING:
        return f"Get ready... {game.time_remaining_s:.0f}"
    seconds = int(game.time_remaining_s + 0.999)
    text = f"Score {game.score}   {seconds // 60}:{seconds % 60:02d}"
    if game.phase == Phase.FINISHED:
        text = f"Final score {game.score}!"
    return text
