# rov_nav2_plugins

Nav2 plugins that navigate the ROV in 3D: x, y, depth and heading. They are modeled on the `orca_nav2` plugins from [Orca4](https://github.com/clydemcqueen/orca4), which navigate a BlueROV2 in open water, plus a planner that routes around obstacles the sonar sees.

Nav2's stock plugins are 2D. They ignore depth, so they report success at the wrong depth and treat a dive as no progress.

## Plugins

- [OctomapPlanner3D](src/planner/octomap_planner_3d.md) -- Global planner: searches a true 3D OctoMap and routes beside, above or below obstacles
- [PurePursuitController3D](src/controller/pure_pursuit_controller_3d.md) -- Controller: 3D pure pursuit commanding surge, sway, heave and yaw rate within velocity and acceleration limits
- [GoalChecker3D](src/goal_checker/goal_checker_3d.md) -- Goal checker: horizontal, vertical and heading errors all within tolerance at once
- [ProgressChecker3D](src/progress_checker/progress_checker_3d.md) -- Progress checker: moving in 3D or turning counts as progress

Each plugin's page lists its parameters. [PLUGINS.md](PLUGINS.md) covers how Nav2 calls the plugins and how to add one, and [DEVELOPING.md](DEVELOPING.md) maps the source folders.

## Usage

Select the plugins in the Nav2 parameter file, as [`action_bringup/config/nav2.yaml`](../action_bringup/config/nav2.yaml) does for the ROV:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["OctomapPlanner3D"]
    OctomapPlanner3D:
      plugin: "rov_nav2_plugins::OctomapPlanner3D"
      pointcloud_topic: "/sonar/points"
      map_frame: "map"
      map_resolution: 0.1
      sensor_model.max_range: 8.0

controller_server:
  ros__parameters:
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "rov_nav2_plugins::ProgressChecker3D"
    goal_checker:
      plugin: "rov_nav2_plugins::GoalChecker3D"
    FollowPath:
      plugin: "rov_nav2_plugins::PurePursuitController3D"
```

Set each plugin's parameters under its name, such as `FollowPath.lookahead_dist`.

Goals are ordinary `NavigateToPose` and `NavigateThroughPoses` goals with a meaningful z: z is up (REP 103), so a goal 2 m deep has `z: -2.0`. Only the yaw of a goal's orientation is used.

The controller commands body-frame velocities in `cmd_vel`: x forward, y left, z up, and yaw rate.

## Assumptions

- The vehicle is holonomic in the horizontal plane (vectored thrusters, like a BlueROV2 Heavy) and holds itself level in roll and pitch.
- `OctomapPlanner3D` owns a persistent 3D `OcTree` and fills it directly from `/sonar/points` after transforming each cloud into `map`.
