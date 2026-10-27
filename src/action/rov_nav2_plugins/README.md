# rov_nav2_plugins

Nav2 plugins that navigate the ROV in 3D: x, y, depth and heading. They are modeled on the `orca_nav2` plugins from [Orca4](https://github.com/clydemcqueen/orca4), which navigate a BlueROV2 in open water, plus a planner that routes around obstacles the sonar sees.

Nav2's stock plugins are 2D. They ignore depth, so they report success at the wrong depth and treat a dive as no progress.

## Plugins

- [CostmapPlanner3D](src/planner/costmap_planner_3d.md) -- Global planner: routes around the costmap's obstacles with Nav2's Smac 2D planner, adding depth
- [PurePursuitController3D](src/controller/pure_pursuit_controller_3d.md) -- Controller: 3D pure pursuit commanding surge, sway, heave and yaw rate within velocity and acceleration limits
- [GoalChecker3D](src/goal_checker/goal_checker_3d.md) -- Goal checker: horizontal, vertical and heading errors all within tolerance at once
- [ProgressChecker3D](src/progress_checker/progress_checker_3d.md) -- Progress checker: moving in 3D or turning counts as progress
- [VoxelLayer3D](src/costmap/voxel_layer_3d.md) -- Costmap layer: 3D voxel grid of the sonar's returns, floor included, inflated in 3D; the 2D costmap gets only columns blocked at every depth

Each plugin's page lists its parameters. [PLUGINS.md](PLUGINS.md) covers how Nav2 calls the plugins and how to add one, and [DEVELOPING.md](DEVELOPING.md) maps the source folders.

## Usage

Select the plugins in the Nav2 parameter file, as [`action_bringup/config/nav2.yaml`](../action_bringup/config/nav2.yaml) does for the ROV:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["CostmapPlanner3D"]
    CostmapPlanner3D:
      plugin: "rov_nav2_plugins::CostmapPlanner3D"

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

Add the layer to a costmap's plugins:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["voxel_layer_3d"]
      voxel_layer_3d:
        plugin: "rov_nav2_plugins::VoxelLayer3D"
        topic: /sonar/points
```

Set each plugin's parameters under its name, such as `FollowPath.lookahead_dist`.

Goals are ordinary `NavigateToPose` and `NavigateThroughPoses` goals with a meaningful z: z is up (REP 103), so a goal 2 m deep has `z: -2.0`. Only the yaw of a goal's orientation is used.

The controller commands body-frame velocities in `cmd_vel`: x forward, y left, z up, and yaw rate.

## Assumptions

- The vehicle is holonomic in the horizontal plane (vectored thrusters, like a BlueROV2 Heavy) and holds itself level in roll and pitch.
- Obstacles come from the global costmap, which is 2D: the vehicle goes around them, never over or under. `action_bringup` fills the costmap from the sonar's point cloud. `VoxelLayer3D` keeps the 3D costs too, for a 3D planner; `CostmapPlanner3D` does not use them yet.
