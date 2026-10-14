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
"""Scoring a mission run against the mission's criteria."""

import dataclasses
import math
from collections.abc import Sequence

from rov_mission_runner.mission import Mission, Obstacle

Point = tuple[float, float, float]
Command = tuple[
    float, float, float, float
]  # surge, sway, heave (m/s), yaw rate (rad/s)

# Commands at exactly a limit must pass despite floating-point rounding
_LIMIT_SLACK = 1e-6


@dataclasses.dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float  # rad


@dataclasses.dataclass
class Telemetry:
    """What was observed during one mission run."""

    outcome: str = "not run"  # succeeded, failed, canceled, rejected or unknown
    duration: float = 0.0  # s
    final_pose: Pose | None = None  # when the run ended
    planned_path: list[Point] = dataclasses.field(
        default_factory=list
    )  # the latest plan
    positions: list[Point] = dataclasses.field(
        default_factory=list
    )  # sampled during the run
    plan_start: int = (
        0  # index of the first position sampled while following the latest plan
    )
    earlier_deviation: float = 0.0  # m, the most the vehicle strayed from earlier plans
    max_command: Command = (0.0, 0.0, 0.0, 0.0)  # largest magnitude on each axis
    last_command: Command | None = None

    def record_plan(self, path: list[Point]) -> None:
        """Start following a new plan. Navigation replans as it goes, so the positions sampled so
        far are scored against the plan they were following."""
        if self.planned_path:
            self.earlier_deviation = self.path_deviation()
        self.plan_start = len(self.positions)
        self.planned_path = path

    def path_deviation(self) -> float:
        """The most the vehicle strayed from the plan it was following, in m."""
        return max(
            self.earlier_deviation,
            _max_deviation(self.positions[self.plan_start :], self.planned_path),
        )

    def record_command(self, command: Command) -> None:
        self.max_command = tuple(
            max(peak, abs(value)) for peak, value in zip(self.max_command, command)
        )
        self.last_command = command

    def stopped(self) -> bool:
        """Whether the last velocity command was zero."""
        return self.last_command is not None and all(
            abs(value) <= _LIMIT_SLACK for value in self.last_command
        )


@dataclasses.dataclass(frozen=True)
class Check:
    """One acceptance criterion and how the run measured up."""

    name: str
    passed: bool
    measured: str


@dataclasses.dataclass(frozen=True)
class MissionReport:
    mission: str
    checks: tuple[Check, ...]

    @property
    def passed(self) -> bool:
        return all(check.passed for check in self.checks)


def score(mission: Mission, telemetry: Telemetry) -> MissionReport:
    """Check a run against every criterion that applies to its mission."""
    criteria = mission.criteria
    checks = [
        Check(
            "outcome",
            telemetry.outcome == mission.expect,
            f"{telemetry.outcome} (expected {mission.expect})",
        ),
        Check(
            "duration",
            telemetry.duration <= criteria.timeout,
            f"{telemetry.duration:.1f} s (limit {criteria.timeout:g} s)",
        ),
    ]
    if mission.expect == "succeeded":
        checks += _final_pose_checks(mission, telemetry.final_pose)
    checks.append(_path_deviation_check(telemetry, criteria.max_path_deviation))
    checks.append(_commanded_speed_check(telemetry.max_command, criteria.max_speed))
    if mission.obstacles:
        checks.append(_clearance_check(mission.obstacles, telemetry.positions))
    if mission.expect == "canceled":
        checks.append(_stopped_check(telemetry))
    return MissionReport(mission.name, tuple(checks))


def format_report(reports: Sequence[MissionReport]) -> str:
    lines = []
    for report in reports:
        lines.append(f"[{'PASS' if report.passed else 'FAIL'}] {report.mission}")
        for check in report.checks:
            lines.append(
                f"    {'ok  ' if check.passed else 'FAIL'}  {check.name:<17} {check.measured}"
            )
    passed = sum(report.passed for report in reports)
    lines.append(f"{passed}/{len(reports)} missions passed")
    return "\n".join(lines)


def distance_to_polyline(point: Point, polyline: Sequence[Point]) -> float:
    """Shortest distance from `point` to a path of straight segments through `polyline`."""
    if len(polyline) == 1:
        return math.dist(point, polyline[0])

    closest = math.inf
    for start, end in zip(polyline, polyline[1:]):
        segment = [e - s for s, e in zip(start, end)]
        length_sq = sum(component**2 for component in segment)
        offset = [p - s for s, p in zip(start, point)]
        t = (
            0.0
            if length_sq == 0.0
            else min(
                max(sum(o * c for o, c in zip(offset, segment)) / length_sq, 0.0), 1.0
            )
        )
        closest = min(
            closest, math.dist(point, [s + t * c for s, c in zip(start, segment)])
        )
    return closest


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_difference(a: float, b: float) -> float:
    """Signed difference a - b, wrapped to [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _final_pose_checks(mission: Mission, pose: Pose | None) -> list[Check]:
    if pose is None:
        return [Check("final pose", False, "unknown: no transform to the robot")]

    goal = mission.waypoints[-1]
    criteria = mission.criteria
    xy_error = math.hypot(pose.x - goal.x, pose.y - goal.y)
    z_error = abs(pose.z - goal.z)
    yaw_error = abs(angle_difference(pose.yaw, goal.yaw))
    return [
        Check(
            "final xy error",
            xy_error <= criteria.xy_tolerance,
            f"{xy_error:.3f} m (limit {criteria.xy_tolerance:g} m)",
        ),
        Check(
            "final z error",
            z_error <= criteria.z_tolerance,
            f"{z_error:.3f} m (limit {criteria.z_tolerance:g} m)",
        ),
        Check(
            "final yaw error",
            yaw_error <= criteria.yaw_tolerance,
            f"{math.degrees(yaw_error):.1f} deg (limit {math.degrees(criteria.yaw_tolerance):g} deg)",
        ),
    ]


def _path_deviation_check(telemetry: Telemetry, limit: float) -> Check:
    if not telemetry.planned_path:
        return Check("path deviation", False, "no plan received")
    deviation = telemetry.path_deviation()
    return Check(
        "path deviation", deviation <= limit, f"{deviation:.3f} m (limit {limit:g} m)"
    )


def _commanded_speed_check(peak: Command, limits: Command) -> Check:
    within = all(value <= limit + _LIMIT_SLACK for value, limit in zip(peak, limits))
    return Check(
        "commanded speed",
        within,
        f"surge {peak[0]:.2f}/{limits[0]:g}, sway {peak[1]:.2f}/{limits[1]:g}, "
        f"heave {peak[2]:.2f}/{limits[2]:g} m/s, yaw {peak[3]:.2f}/{limits[3]:g} rad/s",
    )


def _stopped_check(telemetry: Telemetry) -> Check:
    if telemetry.last_command is None:
        return Check("stop command", False, "no velocity command received")
    stopped = telemetry.stopped()
    return Check(
        "stop command",
        stopped,
        "zero velocity" if stopped else f"last command {telemetry.last_command}",
    )


def _clearance_check(
    obstacles: Sequence[Obstacle], positions: Sequence[Point]
) -> Check:
    # How far the vehicle's center stayed outside the nearest obstacle; negative means inside
    clearance = min(
        (
            math.hypot(x - obstacle.x, y - obstacle.y) - obstacle.radius
            for x, y, _ in positions
            for obstacle in obstacles
        ),
        default=math.inf,
    )
    return Check(
        "obstacle clearance", clearance >= 0.0, f"{clearance:.2f} m (limit 0 m)"
    )


def _max_deviation(positions: Sequence[Point], path: Sequence[Point]) -> float:
    return max(
        (distance_to_polyline(position, path) for position in positions), default=0.0
    )
