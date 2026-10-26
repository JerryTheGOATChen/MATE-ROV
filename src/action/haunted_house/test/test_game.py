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

import pytest

from haunted_house.game import Game, Phase, Rules
from haunted_house.ghost import Command, GhostModel, GhostState, step
from haunted_house.house import Box, House
from haunted_house.people import Walker

# An empty 10 x 10 m room, 3 m tall, with one wall across x = 2
ROOM = House(
    x_range=(-5.0, 5.0),
    y_range=(-5.0, 5.0),
    ceiling_z=3.0,
    walls=(Box("wall", (2.0, -5.0, 0.0), (2.2, 5.0, 3.0)),),
    furniture=(Box("table", (-3.0, -1.0, 0.0), (-2.0, 1.0, 0.8)),),
)
INSTANT = GhostModel(response_time=0.0)
DT = 0.02


def fly(state: GhostState, command: Command, seconds: float, model: GhostModel = INSTANT) -> GhostState:
    for _ in range(round(seconds / DT)):
        state = step(state, command, model, ROOM, DT)
    return state


def make_game(walkers=(), candies=(), start=(0.0, 0.0, 1.5), rules=Rules(start_delay_s=0.0)) -> Game:
    return Game(ROOM, tuple(walkers), tuple(candies), GhostState(position=start, yaw=0.0), INSTANT, rules)


def run(game: Game, command: Command, seconds: float) -> None:
    for _ in range(round(seconds / DT)):
        game.step(DT, command)


class TestHouse:
    def test_distance_to_box(self):
        box = Box("b", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        assert box.distance_to((0.5, 0.5, 0.5)) == 0.0
        assert box.distance_to((2.0, 0.5, 0.5)) == pytest.approx(1.0)
        assert box.distance_to((2.0, 2.0, 0.5)) == pytest.approx(math.sqrt(2.0))

    def test_collides_with_floor_ceiling_and_obstacles(self):
        assert ROOM.collides((0.0, 0.0, 0.2), 0.3)
        assert ROOM.collides((0.0, 0.0, 2.8), 0.3)
        assert ROOM.collides((1.8, 0.0, 1.5), 0.3)
        assert not ROOM.collides((0.0, 0.0, 1.5), 0.3)
        # The table is low, so the ghost can pass over it
        assert ROOM.collides((-2.5, 0.0, 0.9), 0.3)
        assert not ROOM.collides((-2.5, 0.0, 1.2), 0.3)

    def test_occupancy_grid_marks_every_obstacle_seen_from_above(self):
        origin_x, origin_y, width, height, cells = ROOM.occupancy_grid(0.1, margin=0.0)
        assert (origin_x, origin_y, width, height) == (-5.0, -5.0, 100, 100)

        def cell(x, y):
            return cells[int((y - origin_y) / 0.1) * width + int((x - origin_x) / 0.1)]

        assert cell(2.1, 0.0) == 100
        assert cell(-2.5, 0.0) == 100
        assert cell(0.0, 0.0) == 0


class TestGhost:
    def test_follows_commands_in_its_own_frame(self):
        facing_north = GhostState(position=(0.0, 0.0, 1.5), yaw=math.pi / 2)
        moved = fly(facing_north, Command(linear=(0.5, 0.0, 0.0)), 1.0)
        assert moved.position == pytest.approx((0.0, 0.5, 1.5), abs=1e-6)

    def test_respects_speed_limits(self):
        moved = fly(GhostState(position=(0.0, 0.0, 1.5), yaw=0.0), Command(linear=(5.0, 0.0, 0.0), yaw_rate=9.0), 1.0)
        assert moved.velocity[0] <= INSTANT.max_linear_speed[0] + 1e-9
        assert moved.yaw_rate == pytest.approx(INSTANT.max_yaw_rate)

    def test_lags_behind_commands(self):
        state = step(GhostState(position=(0.0, 0.0, 1.5), yaw=0.0), Command((1.0, 0.0, 0.0)), GhostModel(), ROOM, DT)
        assert 0.0 < state.velocity[0] < 0.2

    def test_slides_along_walls_instead_of_passing_through(self):
        start = GhostState(position=(1.0, 0.0, 1.5), yaw=0.0)
        # Heading east and north at once: the wall stops the eastward part only
        moved = fly(start, Command(linear=(0.5, 0.5, 0.0)), 4.0)
        assert moved.position[0] < 2.0 - INSTANT.radius + 1e-6
        assert moved.position[1] == pytest.approx(2.0, abs=1e-6)
        assert moved.bonked

    def test_stays_between_floor_and_ceiling(self):
        start = GhostState(position=(0.0, 0.0, 1.5), yaw=0.0)
        assert fly(start, Command(linear=(0.0, 0.0, -0.6)), 5.0).position[2] >= INSTANT.radius
        assert fly(start, Command(linear=(0.0, 0.0, 0.6)), 5.0).position[2] <= ROOM.ceiling_z - INSTANT.radius


class TestWalker:
    walker = Walker("w", path=((0.0, 0.0), (2.0, 0.0), (2.0, 2.0)), speed=1.0, head_z=1.6)

    def test_walks_its_loop_in_straight_lines(self):
        assert self.walker.position_at(0.0) == pytest.approx((0.0, 0.0, 1.6))
        assert self.walker.position_at(1.0) == pytest.approx((1.0, 0.0, 1.6))
        assert self.walker.position_at(3.0) == pytest.approx((2.0, 1.0, 1.6))
        # Back to the start along the diagonal, then around again
        loop = 4.0 + math.sqrt(8.0)
        assert self.walker.position_at(loop) == pytest.approx((0.0, 0.0, 1.6), abs=1e-9)
        assert self.walker.position_at(loop + 1.0) == pytest.approx((1.0, 0.0, 1.6))

    def test_rejects_bad_paths(self):
        with pytest.raises(ValueError):
            Walker("w", path=((0.0, 0.0),), speed=1.0, head_z=1.6)
        with pytest.raises(ValueError):
            Walker("w", path=((0.0, 0.0), (1.0, 0.0)), speed=0.0, head_z=1.6)


class TestGame:
    def test_counts_down_before_starting_and_freezes_the_ghost(self):
        game = make_game(candies=[(3.0, 3.0, 1.0)], rules=Rules(start_delay_s=1.0))
        run(game, Command(linear=(1.0, 0.0, 0.0)), 0.5)
        assert game.phase == Phase.WAITING
        assert game.ghost.position == (0.0, 0.0, 1.5)
        run(game, Command(), 0.6)
        assert game.phase == Phase.RUNNING

    def test_eating_candy_scores_and_removes_it(self):
        game = make_game(candies=[(1.0, 0.0, 1.5), (-4.0, -4.0, 1.0)])
        run(game, Command(linear=(1.0, 0.0, 0.0)), 1.0)
        assert game.score == 1
        assert game.candies_eaten == 1
        assert [c.id for c in game.candies] == [1]

    def test_haunting_a_person_scores_ten_and_removes_them(self):
        # Someone walks north across the ghost's way as it heads east
        walker = Walker("victim", path=((1.5, -1.0), (1.5, 4.0)), speed=0.5, head_z=1.5)
        game = make_game(walkers=[walker], candies=[(-4.0, -4.0, 1.0)])
        run(game, Command(linear=(1.0, 0.0, 0.0)), 1.5)
        assert game.score == 10
        assert game.people_haunted == 1
        assert game.people == []

    def test_people_must_be_reached_at_head_height(self):
        walker = Walker("tall", path=((0.0, 0.0), (0.0, 4.0)), speed=0.01, head_z=2.6)
        game = make_game(walkers=[walker], candies=[(-4.0, -4.0, 1.0)])
        run(game, Command(), 1.0)
        assert game.score == 0

    def test_ends_when_time_runs_out(self):
        game = make_game(candies=[(-4.0, -4.0, 1.0)], rules=Rules(duration_s=2.0, start_delay_s=0.0))
        run(game, Command(), 2.5)
        assert game.phase == Phase.FINISHED
        assert game.time_remaining_s == 0.0
        # The ghost is frozen once the game is over
        run(game, Command(linear=(1.0, 0.0, 0.0)), 1.0)
        assert game.ghost.position == (0.0, 0.0, 1.5)

    def test_ends_early_when_everything_is_collected(self):
        game = make_game(candies=[(1.0, 0.0, 1.5)])
        run(game, Command(linear=(1.0, 0.0, 0.0)), 1.0)
        assert game.phase == Phase.FINISHED
        assert game.time_remaining_s > 100.0

    def test_counts_bonks(self):
        game = make_game(candies=[(-4.0, -4.0, 1.0)], start=(1.0, 0.0, 1.5))
        run(game, Command(linear=(1.0, 0.0, 0.0)), 2.0)
        assert game.bonks == 1

    def test_reset_puts_everything_back(self):
        game = make_game(candies=[(1.0, 0.0, 1.5), (-4.0, -4.0, 1.0)])
        run(game, Command(linear=(1.0, 0.0, 0.0)), 1.0)
        game.reset()
        assert game.score == 0
        assert len(game.candies) == 2
        assert game.ghost.position == (0.0, 0.0, 1.5)
