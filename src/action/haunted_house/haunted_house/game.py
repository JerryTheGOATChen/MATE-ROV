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
"""The game: a ghost has two minutes to haunt people and eat candy in a haunted house."""

from __future__ import annotations

import itertools
import math
import random
from dataclasses import dataclass, field
from enum import IntEnum

from haunted_house.ghost import Command, GhostModel, GhostState, step
from haunted_house.house import House, Vec3
from haunted_house.people import Walker


@dataclass(frozen=True)
class Rules:
    duration_s: float = 120.0
    # Countdown before the clock starts, after launch and after every reset
    start_delay_s: float = 3.0
    # How close the ghost's center must come to a candy, or to a person's head
    catch_radius_m: float = 0.5
    candy_points: int = 1
    person_points: int = 10
    # Each game, every candy moves up to this far from its spot in the house file, in x and y (m)
    candy_jitter_m: float = 0.25
    # Each game, every person starts somewhere random along their loop
    random_people_start: bool = True


class Phase(IntEnum):
    WAITING = 0
    RUNNING = 1
    FINISHED = 2


@dataclass
class Target:
    id: int
    name: str
    points: int
    position: Vec3
    # People walk; candies have none
    walker: Walker | None = None
    # Where along their loop a person started, in seconds of walking
    start_offset_s: float = 0.0


@dataclass(frozen=True)
class Catch:
    """A target the ghost reached this step."""

    target: Target
    is_person: bool


@dataclass
class Game:
    house: House
    walkers: tuple[Walker, ...]
    candy_positions: tuple[Vec3, ...]
    ghost_start: GhostState
    model: GhostModel = field(default_factory=GhostModel)
    rules: Rules = field(default_factory=Rules)

    def __post_init__(self) -> None:
        self.reset()

    def reset(self, rng: random.Random | None = None) -> None:
        """Put everything back and count down to a new game.

        With a random generator, the candy and the people's starting points are shuffled a little,
        as the rules say. Without one, everything is exactly as in the house file.
        """
        self.phase = Phase.WAITING
        self.countdown_s = self.rules.start_delay_s
        self.elapsed_s = 0.0
        self.ghost = self.ghost_start
        self.score = 0
        self.candies_eaten = 0
        self.people_haunted = 0
        self.bonks = 0
        self.people = []
        for i, walker in enumerate(self.walkers):
            offset = rng.uniform(0.0, walker.loop_length / walker.speed) if rng and self.rules.random_people_start else 0.0
            self.people.append(
                Target(i, walker.name, self.rules.person_points, walker.position_at(offset), walker, offset)
            )
        self.candies = [
            Target(i, _candy_name(i), self.rules.candy_points, self._place_candy(p, rng))
            for i, p in enumerate(self.candy_positions)
        ]

    @property
    def time_remaining_s(self) -> float:
        if self.phase == Phase.WAITING:
            return self.countdown_s
        return max(0.0, self.rules.duration_s - self.elapsed_s)

    def step(self, dt: float, command: Command) -> list[Catch]:
        """Advance the game by dt seconds, with the ghost under the given command.

        Returns what the ghost reached during the step.
        """
        if self.phase == Phase.WAITING:
            self.countdown_s -= dt
            if self.countdown_s <= 0.0:
                self.phase = Phase.RUNNING
            return []
        if self.phase == Phase.FINISHED:
            return []

        self.elapsed_s = min(self.elapsed_s + dt, self.rules.duration_s)
        was_bonked = self.ghost.bonked
        self.ghost = step(self.ghost, command, self.model, self.house, dt)
        if self.ghost.bonked and not was_bonked:
            self.bonks += 1

        for person in self.people:
            person.position = person.walker.position_at(person.start_offset_s + self.elapsed_s)

        catches = [Catch(p, True) for p in self.people if self._reaches(p)]
        catches += [Catch(c, False) for c in self.candies if self._reaches(c)]
        for catch in catches:
            self.score += catch.target.points
            if catch.is_person:
                self.people.remove(catch.target)
                self.people_haunted += 1
            else:
                self.candies.remove(catch.target)
                self.candies_eaten += 1

        if self.elapsed_s >= self.rules.duration_s or not (self.people or self.candies):
            self.phase = Phase.FINISHED
        return catches

    def _reaches(self, target: Target) -> bool:
        return math.dist(self.ghost.position, target.position) <= self.rules.catch_radius_m

    def _place_candy(self, position: Vec3, rng: random.Random | None) -> Vec3:
        """The candy's spot, moved a little at random if there is a generator, keeping it reachable."""
        jitter = self.rules.candy_jitter_m
        if rng is None or jitter <= 0.0:
            return position
        for _ in range(20):
            moved = (position[0] + rng.uniform(-jitter, jitter), position[1] + rng.uniform(-jitter, jitter), position[2])
            if self.can_reach(moved):
                return moved
        return position

    def can_reach(self, position: Vec3) -> bool:
        """Whether the ghost can get close enough to a point to collect it, without touching anything."""
        if any(box.distance_to(position) < 0.02 for box in self.house.obstacles):
            return False
        offsets = [i * 0.1 for i in range(-4, 5)]
        for dx, dy, dz in itertools.product(offsets, repeat=3):
            point = (position[0] + dx, position[1] + dy, position[2] + dz)
            if math.dist(point, position) <= self.rules.catch_radius_m - 0.05 and not self.house.collides(
                point, self.model.radius
            ):
                return True
        return False


_CANDY_NAMES = ("Candy corn", "Chocolate bar", "Lollipop", "Gummy worm", "Toffee", "Caramel apple", "Jelly bean")


def _candy_name(index: int) -> str:
    return f"{_CANDY_NAMES[index % len(_CANDY_NAMES)]} {index + 1}"
