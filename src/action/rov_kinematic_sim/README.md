# rov_kinematic_sim

A lightweight simulated ROV that stands in for the vehicle and its localization, so navigation can be developed and tested without the robot or a physics simulator. It follows velocity commands with a realistic lag, drifts with a steady disturbance, and floats at the surface.

## Usage

To run Nav2 on the simulator, use an action DevContainer. Stop the deployed action container first, because its Nav2 servers have the same names:

```bash
./watod -m action down
./watod -m action:dev up
./watod -t action_bringup_dev
```

In the DevContainer, build the workspace and launch the action module with the simulator:

```bash
colcon build
source install/setup.bash
ros2 launch rov_kinematic_sim sim.launch.yaml
```

Then, from a second shell in the DevContainer (`./watod -t action_bringup_dev`, then `source install/setup.bash`), send a goal or fly the acceptance missions:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: -1.0}, orientation: {w: 1.0}}}}"
ros2 run rov_mission_runner mission_runner $(ros2 pkg prefix --share rov_kinematic_sim)/missions/sim_acceptance.yaml
```

To watch, connect Foxglove to the bridge address that `watod up` prints, add a 3D panel with its display frame set to `map`, and turn on `/plan` and `/odom`. On Docker Desktop, that address only works with host networking enabled (Settings, Resources, Network).

`kinematic_sim.launch.yaml` runs the simulator alone: the simulator, a lifecycle manager that activates it, and an identity `map -> odom` transform in place of localization. Pass `params_file:=<file>` to change the vehicle or the disturbance.

For sensor noise, the DVL, the sonar and obstacles, use the Gazebo pool in [`rov_gazebo_sim`](../../simulation/rov_gazebo_sim) instead.

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `cmd_vel` (subscribed) | `geometry_msgs/TwistStamped` | Velocity in the vehicle frame: x forward, y left, z up, angular z for yaw rate. Commands in any frame other than `base_frame` are ignored |
| `odom` (published) | `nav_msgs/Odometry` | Ground-truth pose, and velocity over ground in the vehicle frame |
| `odom -> base_link` (TF) | transform | Same pose as `odom` |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate_hz` | 50.0 | Simulation and publishing rate |
| `cmd_vel_timeout_s` | 0.5 | The vehicle stops thrusting if no command arrives for this long |
| `odom_frame`, `base_frame` | `odom`, `base_link` | Frames of the odometry and transform |
| `initial_pose` | `[0, 0, 0, 0]` | x, y, z (m) and yaw (rad) when configured |
| `response_time_s` | 0.3 | Time constant of the lag from commanded to actual velocity |
| `max_linear_speed` | `[1.0, 0.6, 0.6]` | Thruster limits for surge, sway and heave, m/s |
| `max_yaw_rate` | 1.0 | Thruster limit for yaw rate, rad/s |
| `water_current` | `[0, 0, 0]` | Steady drift in the world frame, m/s |
| `surface_z` | 0.0 | The vehicle cannot rise above this height |

The packaged configuration sets a drift of a few cm/s, which stands in for tether drag and positive buoyancy, so navigation is always tested against a disturbance.

## Acceptance criteria

Nav2 on the ROV is accepted when every check below passes. `./watod -m action test action` runs them all in Docker. The system checks fly [`missions/sim_acceptance.yaml`](missions/sim_acceptance.yaml) through the full Nav2 stack on this simulator, which drifts at about 4 cm/s throughout to stand in for tether drag and buoyancy.

| Requirement | Checked by | Passes when |
|-------------|------------|-------------|
| Nav2 loads the 3D plugins and starts | `test_plugin_loading` in `rov_nav2_plugins`, and the acceptance test | All four plugins load by name and Nav2 becomes active |
| Reaches 3D goals with the right heading | Missions `dive`, `transit_and_turn` and `return_and_surface` | Final error is at most 0.25 m horizontally, 0.2 m vertically and 10° in heading, within 30 s |
| Follows multi-waypoint routes without stopping | Mission `survey` | The same final errors, within 60 s |
| Stays on the planned path | Every mission | The vehicle stays within 0.35 m of the plan |
| Respects the vehicle's limits | Every mission | `cmd_vel` never exceeds 0.5 m/s surge, 0.3 m/s sway or heave, or 0.6 rad/s yaw |
| Gives up on an unreachable goal rather than hanging | Mission `unreachable_goal_fails` | Navigation fails within 30 s |
| Stops when navigation is canceled | Mission `cancel_stops_vehicle` | The goal is canceled and the last `cmd_vel` is zero |
| The control law converges and stays on the path despite lag and drift | Closed-loop unit tests in `rov_nav2_plugins` | Thresholds in `test_controller_math.cpp` |
| Routes around obstacles the sonar sees, but not the floor, with noisy localization | `test_pool_missions` in [`rov_gazebo_sim`](../../simulation/rov_gazebo_sim) | Every pool mission passes, scored on the ground truth |

The missions take about a minute. The runner prints each measurement next to its threshold, so the margins are visible in the test output.

The same runner works on the real ROV: write a mission file with waypoints in the pool's `map` frame and run it as above.
