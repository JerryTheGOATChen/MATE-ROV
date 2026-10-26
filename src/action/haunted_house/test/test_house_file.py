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
"""Checks the packaged house is fair: people walk clear of everything, and every candy can be reached."""

import itertools
import random
import math
import os

import pytest

from haunted_house.config import load_game
from haunted_house.game import Phase
from haunted_house.ghost import Command

DEFAULT_HOUSE_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "haunted_house.yaml")
# Point HOUSE_FILE at another house to check it too
HOUSE_FILE = os.environ.get("HOUSE_FILE", DEFAULT_HOUSE_FILE)


@pytest.fixture(scope="module")
def game():
    return load_game(HOUSE_FILE)


@pytest.mark.skipif(HOUSE_FILE != DEFAULT_HOUSE_FILE, reason="only for the packaged house")
def test_loads(game):
    assert len(game.people) == 6
    assert len(game.candies) == 20
    assert game.rules.duration_s == 120.0


def test_people_walk_clear_of_walls_and_furniture(game):
    # Anything below head height is in the way of a person
    for walker in game.walkers:
        steps = int(walker.loop_length / 0.05)
        for i in range(steps):
            x, y, head_z = walker.position_at(i * 0.05 / walker.speed)
            for box in game.house.obstacles:
                if box.min[2] < head_z:
                    footprint_distance = box.distance_to((x, y, min(max(box.min[2], 0.0), box.max[2])))
                    assert footprint_distance >= 0.25, f"{walker.name} walks into {box.name} at ({x:.2f}, {y:.2f})"


def test_every_candy_can_be_reached(game):
    house, radius, reach = game.house, game.model.radius, game.rules.catch_radius_m
    offsets = [i * 0.05 for i in range(-10, 11)]
    for candy in game.candies:
        cx, cy, cz = candy.position
        reachable = any(
            math.dist((cx + dx, cy + dy, cz + dz), candy.position) <= reach - 0.05
            and not house.collides((cx + dx, cy + dy, cz + dz), radius)
            for dx, dy, dz in itertools.product(offsets, repeat=3)
        )
        assert reachable, f"{candy.name} at {candy.position} cannot be reached"


def test_candies_are_not_inside_obstacles(game):
    for candy in game.candies:
        for box in game.house.obstacles:
            assert box.distance_to(candy.position) > 0.0, f"{candy.name} is inside {box.name}"


def test_a_ghost_that_stays_still_scores_nothing():
    game = load_game(HOUSE_FILE)
    dt = 0.02
    while game.phase != Phase.FINISHED:
        game.step(dt, Command())
    assert game.score == 0


def test_shuffling_keeps_candy_reachable_and_moves_people():
    game = load_game(HOUSE_FILE)
    rng = random.Random(7)
    starts = set()
    for _ in range(10):
        game.reset(rng)
        for candy, original in zip(game.candies, game.candy_positions):
            assert game.can_reach(candy.position), f"{candy.name} moved somewhere unreachable"
            assert math.dist(candy.position[:2], original[:2]) <= game.rules.candy_jitter_m * math.sqrt(2) + 1e-9
        starts.add(round(game.people[0].start_offset_s, 3))
    assert len(starts) > 1
