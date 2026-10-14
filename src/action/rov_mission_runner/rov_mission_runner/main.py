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
"""Run the missions in a file through Nav2 and print a scored report.

Exits 0 when every mission passes, 1 when any fails, and 2 when the mission file is unusable.
"""

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from rov_mission_runner.mission import MissionError, load_missions
from rov_mission_runner.runner import MissionRunner
from rov_mission_runner.scoring import format_report, score

EXIT_PASSED = 0
EXIT_FAILED = 1
EXIT_USAGE = 2


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv if argv is None else argv
    args = _parse_args(remove_ros_args(argv)[1:])

    try:
        mission_set = load_missions(args.missions)
    except (OSError, MissionError) as e:
        print(f"Cannot load {args.missions}: {e}", file=sys.stderr)
        return EXIT_USAGE

    unknown = sorted(
        set(args.only) - {mission.name for mission in mission_set.missions}
    )
    if unknown:
        print(
            f"No such missions in {args.missions}: {', '.join(unknown)}",
            file=sys.stderr,
        )
        return EXIT_USAGE
    missions = [
        mission
        for mission in mission_set.missions
        if not args.only or mission.name in args.only
    ]

    # Without rclpy's signal handlers, Ctrl-C raises KeyboardInterrupt while the context is still
    # up, so the runner can cancel the active goal before exiting
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    runner = MissionRunner(mission_set.frame_id, args.truth_topic)
    try:
        runner.wait_for_nav2()
        reports = []
        for mission in missions:
            runner.logger.info(f"Running mission '{mission.name}'")
            reports.append(score(mission, runner.run(mission)))
            runner.logger.info(
                f"Mission '{mission.name}': {'PASS' if reports[-1].passed else 'FAIL'}"
            )

        print(format_report(reports), flush=True)
        return EXIT_PASSED if all(report.passed for report in reports) else EXIT_FAILED
    except KeyboardInterrupt:
        runner.logger.warning("Interrupted")
        return EXIT_FAILED
    finally:
        runner.close()
        rclpy.try_shutdown()


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="mission_runner",
        description="Run navigation missions through Nav2 and score each against its acceptance criteria.",
    )
    parser.add_argument("missions", type=Path, help="mission file (YAML)")
    parser.add_argument(
        "--only", nargs="+", default=[], metavar="NAME", help="run only these missions"
    )
    parser.add_argument(
        "--truth-topic",
        metavar="TOPIC",
        help="score on a simulator's ground-truth odometry (nav_msgs/Odometry) instead of TF",
    )
    return parser.parse_args(args)


if __name__ == "__main__":
    sys.exit(main())
