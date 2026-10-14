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
import copy
import math

import pytest
import yaml

from rov_mission_runner.mission import MissionError, load_missions, parse_missions

DEFAULTS = {
    "timeout_s": 60,
    "xy_tolerance_m": 0.25,
    "z_tolerance_m": 0.2,
    "yaw_tolerance_deg": 10,
    "max_path_deviation_m": 0.35,
    "max_speed": {"x": 0.5, "y": 0.3, "z": 0.3, "yaw": 0.6},
}
DIVE = {"name": "dive", "waypoints": [{"x": 0, "y": 0, "z": -2}]}


def mission_file(*missions):
    return {
        "frame_id": "map",
        "defaults": copy.deepcopy(DEFAULTS),
        "missions": list(missions),
    }


def test_defaults_apply_to_every_mission():
    mission = parse_missions(mission_file(DIVE)).missions[0]

    assert mission.criteria.timeout == 60.0
    assert mission.criteria.xy_tolerance == 0.25
    assert mission.criteria.max_speed == (0.5, 0.3, 0.3, 0.6)
    assert mission.expect == "succeeded"
    assert mission.cancel_after is None


def test_a_mission_can_override_criteria():
    survey = {
        **DIVE,
        "name": "survey",
        "criteria": {"timeout_s": 120, "max_path_deviation_m": 0.5},
    }
    mission = parse_missions(mission_file(survey)).missions[0]

    assert mission.criteria.timeout == 120.0
    assert mission.criteria.max_path_deviation == 0.5
    assert mission.criteria.xy_tolerance == 0.25


def test_angles_are_given_in_degrees():
    turn = {
        "name": "turn",
        "waypoints": [
            {"x": 1, "y": 2, "z": -1, "yaw_deg": 90},
            {"x": 1, "y": 3, "z": -1},
        ],
    }
    mission_set = parse_missions(mission_file(turn))
    mission = mission_set.missions[0]

    assert mission_set.frame_id == "map"
    assert mission.waypoints[0].yaw == pytest.approx(math.pi / 2)
    assert mission.waypoints[1].yaw == 0.0
    assert mission.criteria.yaw_tolerance == pytest.approx(math.radians(10))


def test_a_cancel_mission_has_a_cancel_time():
    cancel = {**DIVE, "name": "cancel", "expect": "canceled", "cancel_after_s": 3}
    mission = parse_missions(mission_file(cancel)).missions[0]

    assert mission.expect == "canceled"
    assert mission.cancel_after == 3.0


def test_a_mission_can_list_obstacles():
    around = {**DIVE, "obstacles": [{"x": 5, "y": 3, "radius_m": 0.6}]}
    mission = parse_missions(mission_file(around)).missions[0]

    assert len(mission.obstacles) == 1
    assert (mission.obstacles[0].x, mission.obstacles[0].y) == (5.0, 3.0)
    assert mission.obstacles[0].radius == 0.6
    assert parse_missions(mission_file(DIVE)).missions[0].obstacles == ()


@pytest.mark.parametrize(
    ("missions", "message"),
    [
        ([], "non-empty list"),
        ([{"name": "dive", "waypoints": []}], "non-empty list"),
        ([{"name": "dive"}], "missing waypoints"),
        ([{**DIVE, "speed": 1}], "unknown keys: speed"),
        ([{"name": "dive", "waypoints": [{"x": 0, "y": 0}]}], "missing z"),
        (
            [{"name": "dive", "waypoints": [{"x": 0, "y": 0, "z": "deep"}]}],
            "z must be a number",
        ),
        ([{**DIVE, "criteria": {"timeout_s": -1}}], "timeout_s must be positive"),
        ([{**DIVE, "criteria": {"max_speed": {"x": 1}}}], "missing y, yaw, z"),
        ([{**DIVE, "expect": "crashed"}], "expect must be one of"),
        ([{**DIVE, "expect": "canceled"}], "cancel_after_s is required"),
        ([{**DIVE, "cancel_after_s": 3}], "cancel_after_s is required"),
        ([DIVE, DIVE], "repeated: dive"),
        ([{**DIVE, "obstacles": {"x": 1}}], "obstacles must be a list"),
        ([{**DIVE, "obstacles": [{"x": 1, "y": 1}]}], "missing radius_m"),
    ],
)
def test_malformed_missions_are_rejected(missions, message):
    with pytest.raises(MissionError, match=message):
        parse_missions(mission_file(*missions))


def test_incomplete_defaults_are_rejected():
    data = mission_file(DIVE)
    del data["defaults"]["timeout_s"]

    with pytest.raises(MissionError, match="defaults is missing timeout_s"):
        parse_missions(data)


def test_load_missions_reads_a_yaml_file(tmp_path):
    path = tmp_path / "missions.yaml"
    path.write_text(yaml.safe_dump(mission_file(DIVE)))

    assert load_missions(path).missions[0].name == "dive"


def test_load_missions_reports_invalid_yaml(tmp_path):
    path = tmp_path / "missions.yaml"
    path.write_text("missions: [unclosed")

    with pytest.raises(MissionError, match="invalid YAML"):
        load_missions(path)
