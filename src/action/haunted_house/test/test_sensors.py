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
import math

import numpy as np
import pytest

from haunted_house.config import load_game
from haunted_house.game import Game, Rules
from haunted_house.ghost import GhostModel, GhostState
from haunted_house.house import Box, House
from haunted_house.people import Walker
from haunted_house.sensors import (
    Camera,
    CameraConfig,
    MotionSensorConfig,
    MotionSensors,
    Sonar,
    SonarConfig,
    build_scene,
    raycast,
)

# A 10 x 10 m room, 3 m tall, centered on the origin
ROOM = House(
    x_range=(-5.0, 5.0),
    y_range=(-5.0, 5.0),
    ceiling_z=3.0,
    walls=(
        Box("east", (5.0, -5.0, 0.0), (5.2, 5.0, 3.0)),
        Box("west", (-5.2, -5.0, 0.0), (-5.0, 5.0, 3.0)),
        Box("north", (-5.0, 5.0, 0.0), (5.0, 5.2, 3.0)),
        Box("south", (-5.0, -5.2, 0.0), (5.0, -5.0, 3.0)),
    ),
    furniture=(),
)
QUIET_SONAR = SonarConfig(noise_stddev=0.0)
QUIET_CAMERA = CameraConfig(noise_stddev=0.0)


def make_game(walkers=(), candies=()) -> Game:
    return Game(ROOM, tuple(walkers), tuple(candies), GhostState(position=(0.0, 0.0, 1.5), yaw=0.0), GhostModel(), Rules())


def ghost_at(x, y, z=1.5, yaw=0.0) -> GhostState:
    return GhostState(position=(x, y, z), yaw=yaw)


def test_raycast_finds_the_nearest_box():
    scene = build_scene(make_game())
    t, hit = raycast(np.array([0.0, 0.0, 1.5]), np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 2.0, 0.0]]), scene)
    assert t == pytest.approx([5.0, 1.5, 2.5])
    assert scene.kinds[hit[0]] == "obstacle"


class TestSonar:
    def test_sweeps_a_full_circle_once_a_second(self):
        sonar = Sonar(QUIET_SONAR, np.random.default_rng(0))
        scene = build_scene(make_game())
        fired = [sonar.advance(ghost_at(0.0, 0.0), scene, 0.02)[1].size for _ in range(50)]
        assert sum(fired) == 360

    def test_measures_distance_around_the_ghost_turned_with_it(self):
        sonar = Sonar(QUIET_SONAR, np.random.default_rng(0))
        scene = build_scene(make_game())
        ranges = np.concatenate([sonar.advance(ghost_at(1.0, 0.0, yaw=math.pi / 2), scene, 0.02)[1] for _ in range(50)])
        # Beam 0 points along the ghost's nose, which faces north; beam 90 points west
        assert ranges[0] == pytest.approx(5.0)
        assert ranges[90] == pytest.approx(6.0)
        assert ranges[270] == pytest.approx(4.0)

    def test_sees_people(self):
        walker = Walker("p", path=((2.0, 0.0), (2.0, 1.0)), speed=0.01, head_z=1.6)
        sonar = Sonar(QUIET_SONAR, np.random.default_rng(0))
        _, ranges = sonar.advance(ghost_at(0.0, 0.0), build_scene(make_game(walkers=[walker])), 0.003)
        assert ranges[0] == pytest.approx(1.8)


class TestCamera:
    def test_depth_is_along_the_camera_axis(self):
        camera = Camera(QUIET_CAMERA, np.random.default_rng(0))
        depth = camera.render(ghost_at(0.0, 0.0), build_scene(make_game()))
        assert depth.shape == (72, 128)
        # The wall is 5 m from the ghost and 4.75 m from the camera, at every pixel that sees it
        assert depth[36, 64] == pytest.approx(4.75, abs=1e-4)
        assert depth[36, 30] == pytest.approx(4.75, abs=1e-4)

    def test_spots_people_and_candy_in_view(self):
        walker = Walker("p", path=((3.0, 0.5), (3.0, 1.5)), speed=0.01, head_z=1.6)
        game = make_game(walkers=[walker], candies=[(2.0, -1.0, 1.5), (-2.0, 0.0, 1.5)])
        detections = Camera(QUIET_CAMERA, np.random.default_rng(0)).detect(ghost_at(0.0, 0.0), build_scene(game))
        # The candy behind the ghost is out of view
        assert sorted(d.kind for d in detections) == ["candy", "person"]
        candy = next(d for d in detections if d.kind == "candy")
        # To the right of center, since it is to the ghost's right
        assert (candy.box[0] + candy.box[2]) / 2 > 64

    def test_cannot_see_through_walls(self):
        house = House(ROOM.x_range, ROOM.y_range, 3.0, ROOM.walls + (Box("screen", (1.0, -2.0, 0.0), (1.2, 2.0, 3.0)),), ())
        game = Game(house, (), ((3.0, 0.0, 1.5),), GhostState(position=(0.0, 0.0, 1.5), yaw=0.0), GhostModel(), Rules())
        assert Camera(QUIET_CAMERA, np.random.default_rng(0)).detect(ghost_at(0.0, 0.0), build_scene(game)) == []


def test_velocity_is_biased_and_heading_is_noisy():
    sensors = MotionSensors(MotionSensorConfig(), np.random.default_rng(3))
    ghost = GhostState(position=(0.0, 0.0, 1.5), yaw=0.5, velocity=(0.5, 0.0, 0.0))
    velocities = np.array([sensors.velocity(ghost)[0] for _ in range(2000)])
    assert velocities.mean(axis=0) == pytest.approx(np.array([0.5, 0.0, 0.0]) + sensors.bias, abs=0.003)
    assert np.any(np.abs(sensors.bias) > 0.0)
    headings = [sensors.heading(ghost) for _ in range(500)]
    assert np.mean(headings) == pytest.approx(0.5, abs=0.01)


def test_the_packaged_house_renders_fast_enough():
    import time

    game = load_game(__file__.replace("test/test_sensors.py", "config/haunted_house.yaml"))
    camera = Camera(CameraConfig(), np.random.default_rng(0))
    start = time.perf_counter()
    for _ in range(5):
        scene = build_scene(game)
        camera.render(game.ghost, scene)
        camera.detect(game.ghost, scene)
    # The camera runs at 10 Hz; leave plenty of room for the rest of the simulator
    assert (time.perf_counter() - start) / 5 < 0.06
