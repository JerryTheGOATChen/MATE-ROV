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
"""System test: Nav2 flies the simulated ROV through the pool missions in Gazebo.

Launches sim.launch.yaml with Gazebo headless, then runs every mission in
missions/pool_missions.yaml through rov_mission_runner, scored on the ground truth. The runner
exits non-zero if any criterion fails, and its report is in the test output.
"""

import os
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

# Generous: the missions take about 90 s, plus Gazebo's startup
MISSIONS_TIMEOUT_S = 300.0


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("rov_gazebo_sim")
    simulated_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(share, "launch", "sim.launch.yaml"))
    )
    mission_runner = Node(
        package="rov_mission_runner",
        executable="mission_runner",
        arguments=[
            os.path.join(share, "missions", "pool_missions.yaml"),
            "--truth-topic",
            "/sim/ground_truth",
        ],
        output="screen",
    )
    test_description = launch.LaunchDescription(
        [simulated_stack, mission_runner, launch_testing.actions.ReadyToTest()]
    )
    return test_description, {"mission_runner": mission_runner}


class TestMissions(unittest.TestCase):
    def test_mission_runner_finishes(self, proc_info, mission_runner):
        proc_info.assertWaitForShutdown(
            process=mission_runner, timeout=MISSIONS_TIMEOUT_S
        )


@launch_testing.post_shutdown_test()
class TestAcceptance(unittest.TestCase):
    def test_every_mission_passes(self, proc_info, mission_runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=mission_runner)
