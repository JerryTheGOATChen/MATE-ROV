# rov_mission_runner

Runs navigation missions through Nav2 and scores each run against acceptance criteria. Use it to check navigation in simulation (it drives the acceptance test in `rov_kinematic_sim`) and on the real ROV in the pool.

## Usage

With Nav2 running:

```bash
ros2 run rov_mission_runner mission_runner MISSIONS.yaml
ros2 run rov_mission_runner mission_runner MISSIONS.yaml --only dive survey
```

In a simulator, `--truth-topic TOPIC` scores the vehicle's true pose from the simulator's ground-truth odometry (`nav_msgs/Odometry`, in the missions' frame) instead of TF, so localization errors count against the run. The runner also logs how far localization was from the truth at the end of each mission.

It waits for Nav2 to become active, runs the missions in order, and prints a report:

```text
[PASS] dive
    ok    outcome           succeeded (expected succeeded)
    ok    duration          10.4 s (limit 30 s)
    ok    final xy error    0.172 m (limit 0.25 m)
    ...
6/6 missions passed
```

It exits with 0 if every mission passes, 1 if any fails, and 2 if the mission file is invalid. Ctrl-C cancels the active goal before exiting.

## Mission files

```yaml
frame_id: map  # frame of every waypoint

defaults:  # criteria for every mission; each mission may override any of them
  timeout_s: 30  # time to reach the expected outcome
  xy_tolerance_m: 0.25  # final error from the last waypoint, checked when the goal succeeds
  z_tolerance_m: 0.2
  yaw_tolerance_deg: 10
  max_path_deviation_m: 0.35  # how far the vehicle may stray from the planned path
  max_speed: {x: 0.5, y: 0.3, z: 0.3, yaw: 0.6}  # largest allowed cmd_vel: m/s, and rad/s for yaw

missions:
  - name: dive
    waypoints:
      - {x: 0.0, y: 0.0, z: -2.0, yaw_deg: 0}  # z is up, so this is 2 m deep

  - name: survey  # several waypoints are flown as one path, without stopping
    criteria: {timeout_s: 60}
    waypoints:
      - {x: 3.0, y: 0.0, z: -1.5, yaw_deg: 0}
      - {x: 3.0, y: 3.0, z: -1.5, yaw_deg: 90}

  - name: cancel_stops_vehicle
    expect: canceled  # or failed; succeeded by default
    cancel_after_s: 3.0
    waypoints:
      - {x: 6.0, y: 0.0, z: -1.0}

  - name: around_pillar
    waypoints:
      - {x: 8.0, y: 3.0, z: -1.5}
    obstacles:  # vertical cylinders the vehicle's center must stay out of
      - {x: 5.0, y: 3.0, radius_m: 0.6}  # the obstacle's radius plus the vehicle's half-width

  - name: over_block
    waypoints:
      - {x: 8.0, y: 0.0, z: -3.5}
    obstacles:
      # z_min and z_max bound an obstacle that doesn't span every depth: its bottom and top, plus
      # the vehicle's half-height. Either may be left out.
      - {x: 5.0, y: 0.0, radius_m: 0.9, z_min: -4.1, z_max: -2.9}
```

Each mission starts wherever the previous one ended.

## Criteria

| Check | Passes when |
|-------|-------------|
| outcome | Nav2 reports the expected outcome |
| duration | The outcome arrives within `timeout_s`. The runner cancels the goal at the timeout |
| final xy, z and yaw error | Only when expecting success: the robot's pose when the goal succeeds is within tolerance of the last waypoint |
| path deviation | The robot stays within `max_path_deviation_m` of the path the planner published. When Nav2 replans, each position is checked against the plan it was following |
| commanded speed | No `cmd_vel` exceeds `max_speed` on any axis |
| stop command | Only when expecting cancellation: the last `cmd_vel` after canceling is zero |
| obstacle clearance | Only for missions with `obstacles`: the robot never enters one, measured in 3D for obstacles with `z_min` or `z_max` |

The runner reads the robot's pose from TF (`frame_id` to `base_link`), or from `--truth-topic`, the plan from `plan`, and commands from `cmd_vel` (`geometry_msgs/TwistStamped`). If Nav2's commands are remapped, remap the runner's too, such as `--ros-args -r cmd_vel:=/nav2/cmd_vel`.
