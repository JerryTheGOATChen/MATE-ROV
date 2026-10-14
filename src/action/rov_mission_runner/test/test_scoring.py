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
import dataclasses
import math

import pytest

from rov_mission_runner.mission import Criteria, Mission, Obstacle, Waypoint
from rov_mission_runner.scoring import (
    Pose,
    Telemetry,
    angle_difference,
    distance_to_polyline,
    format_report,
    score,
    yaw_from_quaternion,
)

CRITERIA = Criteria(
    timeout=60.0,
    xy_tolerance=0.25,
    z_tolerance=0.2,
    yaw_tolerance=math.radians(10),
    max_path_deviation=0.35,
    max_speed=(0.5, 0.3, 0.3, 0.6),
)
TRANSIT = Mission(
    name="transit",
    waypoints=(Waypoint(4.0, 0.0, -1.0, math.pi / 2),),
    criteria=CRITERIA,
)
CANCEL = dataclasses.replace(
    TRANSIT, name="cancel", expect="canceled", cancel_after=3.0
)


def good_run() -> Telemetry:
    telemetry = Telemetry(
        outcome="succeeded",
        duration=12.0,
        final_pose=Pose(3.9, 0.1, -1.05, math.pi / 2 + 0.05),
        planned_path=[(0.0, 0.0, 0.0), (4.0, 0.0, -1.0)],
        positions=[(0.0, 0.0, 0.0), (2.0, 0.1, -0.5), (3.9, 0.1, -1.05)],
    )
    telemetry.record_command((0.5, -0.3, -0.3, 0.6))
    telemetry.record_command((0.0, 0.0, 0.0, 0.0))
    return telemetry


def failed_checks(mission: Mission, telemetry: Telemetry) -> list[str]:
    return [
        check.name for check in score(mission, telemetry).checks if not check.passed
    ]


def test_deviation_is_scored_against_the_plan_being_followed():
    # Replanning from the vehicle's position: each sample is near the plan it followed, but the
    # first is far from the last plan
    telemetry = good_run()
    telemetry.positions = []
    telemetry.record_plan([(0.0, 0.0, 0.0), (4.0, 0.0, -1.0)])
    telemetry.positions.append((0.0, 0.1, 0.0))
    telemetry.record_plan([(3.0, 0.0, -0.75), (4.0, 0.0, -1.0)])
    telemetry.positions.append((3.9, 0.1, -1.0))

    assert failed_checks(TRANSIT, telemetry) == []

    # Straying from an earlier plan fails, even if a later plan passes through that position
    telemetry = good_run()
    telemetry.positions = []
    telemetry.record_plan([(0.0, 0.0, 0.0), (4.0, 0.0, -1.0)])
    telemetry.positions.append((2.0, 1.0, -0.5))
    telemetry.record_plan([(2.0, 1.0, -0.5), (4.0, 0.0, -1.0)])

    assert failed_checks(TRANSIT, telemetry) == ["path deviation"]


def test_clearance_is_checked_when_a_mission_lists_obstacles():
    # The good run passes (2.0, 0.1) on its way along y = 0
    clear = dataclasses.replace(TRANSIT, obstacles=(Obstacle(2.0, 1.0, 0.6),))
    hit = dataclasses.replace(TRANSIT, obstacles=(Obstacle(2.0, 0.5, 0.6),))

    assert failed_checks(clear, good_run()) == []
    assert failed_checks(hit, good_run()) == ["obstacle clearance"]
    assert "obstacle clearance" not in [
        check.name for check in score(TRANSIT, good_run()).checks
    ]


def test_distance_to_polyline():
    path = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 0.0, -2.0)]

    assert distance_to_polyline((1.0, 0.0, 0.0), path) == 0.0
    assert distance_to_polyline((1.0, 0.5, 0.0), path) == pytest.approx(0.5)
    assert distance_to_polyline((2.5, 0.0, -1.0), path) == pytest.approx(0.5)
    assert distance_to_polyline((-1.0, 0.0, 0.0), path) == pytest.approx(1.0)
    assert distance_to_polyline((0.0, 0.0, 3.0), [(0.0, 0.0, 0.0)]) == pytest.approx(
        3.0
    )


def test_angles():
    assert yaw_from_quaternion(0.0, 0.0, math.sin(0.4), math.cos(0.4)) == pytest.approx(
        0.8
    )
    assert angle_difference(3.0, -3.0) == pytest.approx(6.0 - 2.0 * math.pi)
    assert angle_difference(-3.0, 3.0) == pytest.approx(2.0 * math.pi - 6.0)


def test_a_good_run_passes():
    report = score(TRANSIT, good_run())

    assert report.passed
    assert [check.name for check in report.checks] == [
        "outcome",
        "duration",
        "final xy error",
        "final z error",
        "final yaw error",
        "path deviation",
        "commanded speed",
    ]


@pytest.mark.parametrize(
    ("change", "failed"),
    [
        ({"outcome": "failed"}, "outcome"),
        ({"duration": 61.0}, "duration"),
        ({"final_pose": Pose(3.7, 0.0, -1.0, math.pi / 2)}, "final xy error"),
        ({"final_pose": Pose(4.0, 0.0, -1.3, math.pi / 2)}, "final z error"),
        ({"final_pose": Pose(4.0, 0.0, -1.0, math.pi / 2 + 0.2)}, "final yaw error"),
        ({"final_pose": None}, "final pose"),
        ({"positions": [(2.0, 0.5, -0.5)]}, "path deviation"),
        ({"planned_path": []}, "path deviation"),
        ({"max_command": (0.51, 0.0, 0.0, 0.0)}, "commanded speed"),
        ({"max_command": (0.0, 0.0, 0.0, 0.7)}, "commanded speed"),
    ],
)
def test_each_criterion_can_fail_a_run(change, failed):
    assert failed_checks(TRANSIT, dataclasses.replace(good_run(), **change)) == [failed]


def test_a_canceled_run_must_end_with_a_stop_command():
    canceled = dataclasses.replace(
        good_run(), outcome="canceled", final_pose=Pose(1.0, 0.0, -0.2, 0.0)
    )
    assert failed_checks(CANCEL, canceled) == []

    canceled.record_command((0.2, 0.0, 0.0, 0.0))
    assert failed_checks(CANCEL, canceled) == ["stop command"]

    canceled.last_command = None
    assert failed_checks(CANCEL, canceled) == ["stop command"]


def test_report_lists_each_mission_and_failure():
    report = format_report([score(TRANSIT, good_run()), score(CANCEL, good_run())])

    assert "[PASS] transit" in report
    assert "[FAIL] cancel" in report
    assert "FAIL  outcome" in report
    assert report.endswith("1/2 missions passed")
