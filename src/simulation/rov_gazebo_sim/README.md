# rov_gazebo_sim

A Gazebo pool with a simulated ROV, for testing the action module's Nav2 with sensor noise, localization and obstacles. The pool is loosely inspired by MATE ROV competition props: PVC frames and pipes, a wreck, a mooring line and a hanging instrument package. Nav2 runs unchanged, with the configuration the real ROV uses.

The ROV follows velocity commands and carries:
- an IMU
- a DVL and a depth sensor
- a 3D sonar that measures height, like the team's

`robot_localization`'s EKF fuses the DVL, depth and IMU into `/odom`, as the real vehicle's localization would.

## Usage

On macOS, native Linux, or Windows with WSL2, use the repository's Docker setup. From the
repository root:

```bash
cp watod-config.gazebo.sh watod-config.local.sh
./watod build
./watod up -d
```

This launches Gazebo Harmonic headlessly with the action stack. See the repository
[`README.md`](../../../README.md#gazebo-on-macos-linux-and-windows-with-wsl2) for the full host
instructions.

For native development on a ROS 2 Jazzy host, install the dependencies and build once from the
repository root:

```bash
rosdep install --from-paths src/simulation src/action src/wato_test src/infrastructure/wato_lifecycle_manager --ignore-src -y
colcon build --packages-up-to rov_gazebo_sim rov_mission_runner
```

Add `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` to the build when `python3` on your `PATH` is not the system's, such as in a virtual environment.

Then, after `source install/setup.bash`, launch it and fly the pool missions from a second shell:

```bash
ros2 launch rov_gazebo_sim sim.launch.yaml gui:=true
ros2 run rov_mission_runner mission_runner $(ros2 pkg prefix --share rov_gazebo_sim)/missions/pool_missions.yaml --truth-topic /sim/ground_truth
```

`gazebo.launch.yaml` runs the simulator alone, in place of the vehicle and its localization, for use with another Nav2 launch.

| Launch argument | Default | Description |
|-----------------|---------|-------------|
| `gui` | false | Show the Gazebo window |
| `dvl_stddev` | 0 | DVL velocity noise (m/s) |
| `depth_stddev` | 0 | Depth sensor noise (m) |
| `gyro_stddev` | 0 | IMU turn rate noise (rad/s) |
| `sonar_stddev` | 0 | Sonar range noise (m) |

Noise values are Gaussian standard deviations, and noise is off by default. Docker users can import
the repository's ready-made [`foxglove/mate_rov_gazebo.json`](../../../foxglove/mate_rov_gazebo.json)
layout after connecting to the bridge address printed by `watod`. For a native host launch, run
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`, connect Foxglove to
`ws://localhost:8765`, and import the same layout. Its 3D panel uses the `map` display frame and
shows these topics:
- `/sim/foxglove_scene` (pool and competition props)
- `/sim/foxglove_rov` (ROV body, heading and traveled path)
- `/plan`
- `/sonar/points`
- `/occupied_cells_vis_array` (the OctoMap's occupied 3D cells)
- `/odom` (the EKF's estimate)
- `/sim/ground_truth` (where the ROV really is)

The layout also contains two Foxglove Teleop pads: one for forward/backward and lateral movement,
and one for depth and yaw. They publish ordinary `Twist` messages on dedicated Foxglove topics.
`foxglove_teleop_adapter.py` combines them, adds the required `base_link` header, clamps them to the
same limits as the controller, and forwards them to the existing `/cmd_vel` interface. Releasing a
button sends stop, and a 0.4 s watchdog sends stop if Foxglove disconnects. Do not operate the pads
while a Nav2 mission is active, because both are command sources for the same vehicle interface.

## The pool

Interior 20 x 12 m and 4 m deep, with the surface at z = 0 and the floor at z = -4. The ROV spawns at (0, 0, -0.3) facing east (+x).

| Prop | Where | Shape |
|------|-------|-------|
| Mooring line | (4, 2) | 0.3 m wide, floor to surface, with a buoy at the top |
| Coral restoration frame | (-5, -3) | 1.2 m PVC square on the floor, 0.5 m tall |
| Shipwreck | (-4, 3.5) | 2.4 x 1.0 x 0.9 m block on the floor |
| Gate | (7, -2) | Two PVC posts 1.5 m apart, crossbar at z = -1 |
| Instrument package | (0, -4) | 0.5 m cube hanging 1.5 m deep |
| Net wall | x = -7, from y = -0.5 to the north wall | Floor to surface, like the side of an aquaculture pen |

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `/cmd_vel` (subscribed) | `geometry_msgs/TwistStamped` | Velocity in `base_link`, as Nav2 sends it |
| `/foxglove/planar_cmd_vel` (subscribed) | `geometry_msgs/Twist` | Foxglove forward/backward and sway commands; adapted to `/cmd_vel` |
| `/foxglove/depth_yaw_cmd_vel` (subscribed) | `geometry_msgs/Twist` | Foxglove heave and yaw commands; adapted to `/cmd_vel` |
| `/odom` (published) | `nav_msgs/Odometry` | The EKF's estimate, also published as `odom -> base_link` |
| `/sonar/points` (published) | `sensor_msgs/PointCloud2` | 3D sonar returns in `sonar_link`: 90° wide, 40° tall, 0.3 to 10 m, at 5 Hz |
| `/sensors/dvl`, `/sensors/depth`, `/sensors/imu` (published) | `nav_msgs/Odometry`, `sensor_msgs/Imu` | The EKF's inputs |
| `/sim/ground_truth` (published) | `nav_msgs/Odometry` | Where the ROV really is, for scoring only |
| `map -> odom`, `base_link -> sonar_link` | TF | Static: localization is the identity, and the sonar sits 0.25 m ahead of the center |

## Missions

[`missions/pool_missions.yaml`](missions/pool_missions.yaml) flies five missions, scored on the ground truth:

| Mission | Checks |
|---------|--------|
| `leave_the_surface` | Diving and moving at once, on the EKF's estimate |
| `around_the_mooring` | The sonar sees the mooring line on the straight route, and the ROV goes around it: its center stays 0.45 m from the line's |
| `near_the_floor` | Crossing the pool to 1 m above the floor. The sonar sees the floor ahead, and the costmap must ignore it |
| `around_the_net` | The net wall blocks the straight route and reaches the pool's side, so the ROV must find the long way around its end: the clearest view of the planner avoiding obstacles |
| `return` | Coming back to the start, around the net again from the costmap's memory, near the surface |

`colcon test --packages-select rov_gazebo_sim` flies them all with Gazebo headless, in about two minutes.
