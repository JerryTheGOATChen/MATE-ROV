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
"""Loads a house file: the layout, the people's paths, the candies and the rules."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from haunted_house.game import Game, Rules
from haunted_house.ghost import GhostModel, GhostState
from haunted_house.house import Box, House
from haunted_house.people import Walker


def _vec(values: Any, length: int, what: str) -> tuple[float, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"{what} must be a list of {length} numbers, got {values!r}")
    return tuple(float(v) for v in values)


def game_from_dict(data: dict[str, Any]) -> Game:
    rules_data = data.get("rules", {})
    rules = Rules(
        duration_s=float(rules_data.get("duration_s", Rules.duration_s)),
        start_delay_s=float(rules_data.get("start_delay_s", Rules.start_delay_s)),
        catch_radius_m=float(rules_data.get("catch_radius_m", Rules.catch_radius_m)),
        candy_points=int(rules_data.get("candy_points", Rules.candy_points)),
        person_points=int(rules_data.get("person_points", Rules.person_points)),
        candy_jitter_m=float(rules_data.get("candy_jitter_m", Rules.candy_jitter_m)),
        random_people_start=bool(rules_data.get("random_people_start", Rules.random_people_start)),
    )

    house_data = data["house"]
    ceiling_z = float(house_data["ceiling_z"])
    walls = tuple(
        Box(f"Wall {i + 1}", (x0, y0, 0.0), (x1, y1, ceiling_z))
        for i, (x0, y0, x1, y1) in enumerate(_vec(w, 4, "a wall") for w in house_data.get("walls", []))
    )
    furniture = tuple(
        Box(f["name"], _vec(f["min"], 3, f"{f['name']} min"), _vec(f["max"], 3, f"{f['name']} max"))
        for f in house_data.get("furniture", [])
    )
    rooms = tuple((r["name"], *_vec(r["at"], 2, f"{r['name']} at")) for r in house_data.get("rooms", []))
    house = House(
        x_range=_vec(house_data["x_range"], 2, "x_range"),
        y_range=_vec(house_data["y_range"], 2, "y_range"),
        ceiling_z=ceiling_z,
        walls=walls,
        furniture=furniture,
        rooms=rooms,
    )

    ghost_data = data.get("ghost", {})
    defaults = GhostModel()
    model = GhostModel(
        response_time=float(ghost_data.get("response_time_s", defaults.response_time)),
        max_linear_speed=_vec(ghost_data.get("max_linear_speed", defaults.max_linear_speed), 3, "max_linear_speed"),
        max_yaw_rate=float(ghost_data.get("max_yaw_rate", defaults.max_yaw_rate)),
        radius=float(ghost_data.get("radius_m", defaults.radius)),
    )
    x, y, z, yaw = _vec(ghost_data.get("start", [0.0, 0.0, 1.5, 0.0]), 4, "ghost start")
    ghost_start = GhostState(position=(x, y, z), yaw=yaw)
    if house.collides(ghost_start.position, model.radius):
        raise ValueError("The ghost's start is inside an obstacle")

    walkers = tuple(
        Walker(
            name=p["name"],
            path=tuple(_vec(point, 2, f"{p['name']} waypoint") for point in p["path"]),
            speed=float(p["speed"]),
            head_z=float(p["head_z"]),
        )
        for p in data.get("people", [])
    )
    candies = tuple(_vec(c, 3, "a candy") for c in data.get("candies", []))

    return Game(house, walkers, candies, ghost_start, model, rules)


def load_game(path: str | Path) -> Game:
    with open(path, encoding="utf-8") as f:
        return game_from_dict(yaml.safe_load(f))
