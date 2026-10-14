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
"""System acceptance test: Nav2 flies the simulated ROV through the acceptance missions.

Launches sim.launch.yaml, the action module with this simulator in place of the vehicle, then
runs every mission in missions/sim_acceptance.yaml through rov_mission_runner, which exits
non-zero if any acceptance criterion fails. Its report is in the test output.
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

# Generous: the missions take about 70 s, plus startup
MISSIONS_TIMEOUT_S = 240.0

# The simulator logs this once configured with config/kinematic_sim.yaml. The criteria in the
# README assume that config's water current.
SIM_CONFIGURED = "water current [0.03 -0.02 0.02] m/s"


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("rov_kinematic_sim")
    simulated_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(share, "launch", "sim.launch.yaml"))
    )
    mission_runner = Node(
        package="rov_mission_runner",
        executable="mission_runner",
        arguments=[os.path.join(share, "missions", "sim_acceptance.yaml")],
        output="screen",
    )
    test_description = launch.LaunchDescription(
        [simulated_stack, mission_runner, launch_testing.actions.ReadyToTest()]
    )
    return test_description, {"mission_runner": mission_runner}


class TestMissions(unittest.TestCase):
    def test_simulator_loads_its_config(self, proc_output):
        proc_output.assertWaitFor(SIM_CONFIGURED, timeout=30.0)

    def test_mission_runner_finishes(self, proc_info, mission_runner):
        proc_info.assertWaitForShutdown(
            process=mission_runner, timeout=MISSIONS_TIMEOUT_S
        )


@launch_testing.post_shutdown_test()
class TestAcceptance(unittest.TestCase):
    def test_every_mission_passes(self, proc_info, mission_runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=mission_runner)
