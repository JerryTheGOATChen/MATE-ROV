# ROV Nav2 Plugins

Nav2 loads these plugins through pluginlib: `plugins.xml` exports each class against its Nav2 base class, and `costmap_plugins.xml` exports the costmap layer, which Nav2 loads through `nav2_costmap_2d` instead of `nav2_core`. The Nav2 servers create the ones named in their YAML configuration. The servers own all ROS communication, except that `VoxelLayer3D` subscribes to the sonar's point cloud and can publish its 3D costs for viewing. Only `CostmapPlanner3D` reads a costmap, the global costmap of the sonar's obstacles; the others use only the costmaps' frames.

## Plugin Types

| Type | Base Class | Server | Purpose |
|---|---|---|---|
| Planner | `nav2_core::GlobalPlanner` | `planner_server` | Turn a goal into a path. |
| Controller | `nav2_core::Controller` | `controller_server` | Turn the path and the vehicle's pose into a velocity command, every control period. |
| Goal checker | `nav2_core::GoalChecker` | `controller_server` | Decide when the vehicle has arrived. |
| Progress checker | `nav2_core::ProgressChecker` | `controller_server` | Fail navigation when the vehicle stops moving. |
| Costmap layer | `nav2_costmap_2d::Layer` | The costmap of `planner_server` or `controller_server` | Turn sensor data into costs. |

## Nav2 Interface

The base classes are defined in [`nav2_core`](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_core/include/nav2_core), and the layer's in [`nav2_costmap_2d`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/include/nav2_costmap_2d/layer.hpp). These tables give when the Jazzy servers call each method, and what these plugins do.

### Lifecycle

| Method | When Called | Purpose |
|---|---|---|
| `configure()` / `initialize()` | The server configures. The planner and controller have `configure()`; the checkers have `initialize()`. | Read parameters. The planner also loads and configures its 2D planner, and the controller takes its frames from the costmap. |
| `activate()` / `deactivate()` | The server activates or deactivates. | Nothing, except that `CostmapPlanner3D` passes them on to its 2D planner. |
| `cleanup()` | The server cleans up. | The controller drops its path, and `CostmapPlanner3D` unloads its 2D planner. |
| `onInitialize()` | The costmap configures, after `initialize()` has given the layer its node, TF buffer and costmap. | `VoxelLayer3D` reads parameters, subscribes to the cloud and sizes its grid to the costmap. |
| `matchSize()` | The costmap is resized. | `VoxelLayer3D` resizes its grid and forgets its voxels. |
| `activate()` / `deactivate()` (layer) | The costmap activates or deactivates. | `VoxelLayer3D` activates or deactivates its `voxel_costs` publisher. |

### Navigation Hooks

| Method | Plugin | When Called | Purpose |
|---|---|---|---|
| `createPlan()` | Planner | Each `ComputePathToPose` request, and once per leg of a `ComputePathThroughPoses` request. | Plan a route around the costmap's obstacles from the start to the goal. |
| `setPlan()` | Controller | A `FollowPath` goal starts, or a new path replaces the current one. | Follow the new path from its beginning, keeping the last command so acceleration stays limited. |
| `computeVelocityCommands()` | Controller | Every control period until the goal is reached. | Return the next `cmd_vel`. |
| `reset()` | Controller | The server publishes zero velocity: navigation succeeded, was canceled or failed. | Forget the path and the last command. |
| `setSpeedLimit()` | Controller | A message on the server's speed limit topic, from costmap filters. | Ignore it, with a warning the first time. |
| `isGoalReached()` | Goal checker | Every control period, with the vehicle's pose and the end of the path. | Check the horizontal, vertical and heading errors. |
| `getTolerances()` | Goal checker | When a controller asks; `PurePursuitController3D` does not. | Report the tolerances, with velocity unchecked. |
| `reset()` | Goal checker | The controller gets a new path. | Nothing: the checker keeps no state. |
| `check()` | Progress checker | Every control period, before the velocity command is computed. | Fail once the vehicle has gone too long without moving or turning enough. |
| `reset()` | Progress checker | A `FollowPath` goal starts. | Start timing again from the next pose. |
| `updateBounds()` | Costmap layer | Every costmap update, at `update_frequency`, before `updateCosts()`. | Insert the clouds received since the last update, decay old voxels and, if anything changed, recompute the 3D costs and mark the whole window changed. |
| `updateCosts()` | Costmap layer | Every costmap update, over the changed window of every layer. | Write each column's lowest cost into the 2D costmap, keeping the higher of it and the cost already there. |
| `reset()` | Costmap layer | A clear-costmap request, from a behavior tree recovery or a service. | Forget every voxel. |

## Plugin Documentation

- [CostmapPlanner3D](src/planner/costmap_planner_3d.md) -- Route around the sonar's obstacles with Nav2's Smac 2D planner, adding depth
- [PurePursuitController3D](src/controller/pure_pursuit_controller_3d.md) -- 3D pure pursuit commanding surge, sway, heave and yaw rate
- [GoalChecker3D](src/goal_checker/goal_checker_3d.md) -- Horizontal, vertical and heading errors all within tolerance at once
- [ProgressChecker3D](src/progress_checker/progress_checker_3d.md) -- Moving in 3D or turning counts as progress
- [VoxelLayer3D](src/costmap/voxel_layer_3d.md) -- 3D voxel grid of the sonar's returns, floor included, inflated in 3D

## Adding a Plugin

1. Create a class inheriting the Nav2 base class, in a new folder of its own under `include/rov_nav2_plugins/`, `src/` and `test/`. Keep the navigation math out of it, in one helper file in the same folder named `<folder>_math`, as described in [DEVELOPING.md](DEVELOPING.md).
1. Add `PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::YourPlugin, nav2_core::Controller)`, with your plugin's base class, at the bottom of the `.cpp`.
1. Add the source to the `rov_nav2_plugins` library in `CMakeLists.txt`.
1. Add the class to `plugins.xml`, or, for a costmap layer, to `costmap_plugins.xml` with base class `nav2_costmap_2d::Layer`:

   ```xml
   <class type="rov_nav2_plugins::YourPlugin" base_class_type="nav2_core::Controller">
     <description>Your plugin description</description>
   </class>
   ```

1. Add it to `test/test_plugin_loading.cpp`, and give it a page next to its source, named after it, like the other plugins'.
1. Select it in the Nav2 parameter file, under its server's plugin list.
