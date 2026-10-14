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
"""Missions: waypoints to navigate to, and the criteria a run must meet to pass.

Missions are loaded from YAML. See the package README for the file format.
"""

import dataclasses
import math
from pathlib import Path

import yaml

OUTCOMES = ("succeeded", "failed", "canceled")  # as nav2_simple_commander reports them


class MissionError(ValueError):
    """A mission file is malformed."""


@dataclasses.dataclass(frozen=True)
class Waypoint:
    """A pose to reach. z is up, so depth is negative."""

    x: float
    y: float
    z: float
    yaw: float  # rad


@dataclasses.dataclass(frozen=True)
class Obstacle:
    """A vertical cylinder, spanning every depth, that the vehicle's center must stay out of."""

    x: float
    y: float
    radius: float  # m, the obstacle's radius plus the vehicle's half-width


@dataclasses.dataclass(frozen=True)
class Criteria:
    """What a mission run must achieve to pass."""

    timeout: float  # s to reach the expected outcome
    xy_tolerance: float  # m, final horizontal error
    z_tolerance: float  # m, final vertical error
    yaw_tolerance: float  # rad, final heading error
    max_path_deviation: float  # m the vehicle may stray from the planned path
    max_speed: tuple[
        float, float, float, float
    ]  # commanded surge, sway, heave (m/s), yaw rate (rad/s)


@dataclasses.dataclass(frozen=True)
class Mission:
    """Navigate to one waypoint, or through several in order."""

    name: str
    waypoints: tuple[Waypoint, ...]
    criteria: Criteria
    expect: str = "succeeded"  # outcome that passes, one of OUTCOMES
    cancel_after: float | None = (
        None  # s after sending the goal; set when expecting "canceled"
    )
    obstacles: tuple[Obstacle, ...] = ()  # to stay clear of throughout the run


@dataclasses.dataclass(frozen=True)
class MissionSet:
    """Missions to run in order, each starting where the previous one ended."""

    frame_id: str
    missions: tuple[Mission, ...]


_CRITERIA_KEYS = {
    "timeout_s",
    "xy_tolerance_m",
    "z_tolerance_m",
    "yaw_tolerance_deg",
    "max_path_deviation_m",
    "max_speed",
}
_SPEED_AXES = ("x", "y", "z", "yaw")


def load_missions(path: Path) -> MissionSet:
    """Load a mission file. Raises MissionError if it is malformed."""
    with open(path, encoding="utf-8") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            raise MissionError(f"invalid YAML: {e}") from e
    return parse_missions(data)


def parse_missions(data: object) -> MissionSet:
    """Build missions from parsed YAML. Raises MissionError if it is malformed."""
    data = _mapping(data, "mission file", required={"frame_id", "defaults", "missions"})
    defaults = _mapping(data["defaults"], "defaults", required=_CRITERIA_KEYS)

    missions = data["missions"]
    if not isinstance(missions, list) or not missions:
        raise MissionError("missions must be a non-empty list")
    parsed = tuple(_parse_mission(entry, defaults) for entry in missions)

    names = [mission.name for mission in parsed]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise MissionError(
            f"mission names must be unique, repeated: {', '.join(duplicates)}"
        )

    return MissionSet(frame_id=str(data["frame_id"]), missions=parsed)


def _parse_mission(entry: object, defaults: dict) -> Mission:
    entry = _mapping(
        entry,
        "mission",
        required={"name", "waypoints"},
        optional={"criteria", "expect", "cancel_after_s", "obstacles"},
    )
    name = str(entry["name"])

    waypoints = entry["waypoints"]
    if not isinstance(waypoints, list) or not waypoints:
        raise MissionError(f"{name}: waypoints must be a non-empty list")

    overrides = _mapping(
        entry.get("criteria", {}), f"{name}: criteria", optional=_CRITERIA_KEYS
    )
    criteria = _parse_criteria({**defaults, **overrides}, name)

    expect = entry.get("expect", "succeeded")
    if expect not in OUTCOMES:
        raise MissionError(
            f"{name}: expect must be one of {', '.join(OUTCOMES)}, not '{expect}'"
        )

    cancel_after = entry.get("cancel_after_s")
    if (cancel_after is not None) != (expect == "canceled"):
        raise MissionError(
            f"{name}: cancel_after_s is required exactly when expecting 'canceled'"
        )
    if cancel_after is not None:
        cancel_after = _positive(cancel_after, f"{name}: cancel_after_s")

    obstacles = entry.get("obstacles", [])
    if not isinstance(obstacles, list):
        raise MissionError(f"{name}: obstacles must be a list")

    return Mission(
        name=name,
        waypoints=tuple(_parse_waypoint(waypoint, name) for waypoint in waypoints),
        criteria=criteria,
        expect=expect,
        cancel_after=cancel_after,
        obstacles=tuple(_parse_obstacle(obstacle, name) for obstacle in obstacles),
    )


def _parse_waypoint(entry: object, mission_name: str) -> Waypoint:
    context = f"{mission_name}: waypoint"
    entry = _mapping(entry, context, required={"x", "y", "z"}, optional={"yaw_deg"})
    return Waypoint(
        x=_number(entry["x"], f"{context} x"),
        y=_number(entry["y"], f"{context} y"),
        z=_number(entry["z"], f"{context} z"),
        yaw=math.radians(_number(entry.get("yaw_deg", 0.0), f"{context} yaw_deg")),
    )


def _parse_obstacle(entry: object, mission_name: str) -> Obstacle:
    context = f"{mission_name}: obstacle"
    entry = _mapping(entry, context, required={"x", "y", "radius_m"})
    return Obstacle(
        x=_number(entry["x"], f"{context} x"),
        y=_number(entry["y"], f"{context} y"),
        radius=_positive(entry["radius_m"], f"{context} radius_m"),
    )


def _parse_criteria(values: dict, mission_name: str) -> Criteria:
    context = f"{mission_name}: criteria"
    max_speed = _mapping(
        values["max_speed"], f"{context} max_speed", required=set(_SPEED_AXES)
    )
    return Criteria(
        timeout=_positive(values["timeout_s"], f"{context} timeout_s"),
        xy_tolerance=_positive(values["xy_tolerance_m"], f"{context} xy_tolerance_m"),
        z_tolerance=_positive(values["z_tolerance_m"], f"{context} z_tolerance_m"),
        yaw_tolerance=math.radians(
            _positive(values["yaw_tolerance_deg"], f"{context} yaw_tolerance_deg")
        ),
        max_path_deviation=_positive(
            values["max_path_deviation_m"], f"{context} max_path_deviation_m"
        ),
        max_speed=tuple(
            _positive(max_speed[axis], f"{context} max_speed {axis}")
            for axis in _SPEED_AXES
        ),
    )


def _mapping(
    value: object,
    context: str,
    required: set = frozenset(),
    optional: set = frozenset(),
) -> dict:
    """Check that `value` is a mapping with all `required` keys and no keys outside `required | optional`."""
    if not isinstance(value, dict):
        raise MissionError(f"{context} must be a mapping")
    missing = required - value.keys()
    if missing:
        raise MissionError(f"{context} is missing {', '.join(sorted(missing))}")
    unknown = value.keys() - required - optional
    if unknown:
        raise MissionError(
            f"{context} has unknown keys: {', '.join(sorted(map(str, unknown)))}"
        )
    return value


def _number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionError(f"{context} must be a number")
    return float(value)


def _positive(value: object, context: str) -> float:
    number = _number(value, context)
    if number <= 0.0:
        raise MissionError(f"{context} must be positive")
    return number
