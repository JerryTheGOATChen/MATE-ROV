# Developing rov_nav2_plugins

## Structure

Each plugin has a folder, the same under `src/`, `include/rov_nav2_plugins/` and `test/`. The rule is one plugin and at most one helper per folder:

- The files named after the plugin are the Nav2 class, its tests and, under `src/`, its documentation page.
- All the navigation math the plugin calls goes in one helper file named after the folder, `<folder>_math`, tested by `test_<folder>_math`. A plugin with no math of its own has no helper.
- `common/pose4d` holds the one type every plugin shares.

| Folder | Nav2 plugin | Math it uses |
|--------|-------------|--------------|
| `common/` | | `pose4d`: x, y, z and yaw, with conversions to and from ROS messages |
| `planner/` | `costmap_planner_3d` | `planner_math`: `add_depth()` adds depth and yaw to a 2D route |
| `controller/` | `pure_pursuit_controller_3d` | `controller_math`: `Pursuit`, the control law below; `PathTracker`, a path parameterized by arc length, which tracks the vehicle's progress and interpolates the pose any distance ahead; and the speed profile and limit helpers |
| `goal_checker/` | `goal_checker_3d` | `pose4d` only |
| `progress_checker/` | `progress_checker_3d` | `pose4d` only |

The math is plain C++ with Eigen, independent of Nav2 and ROS communication, so it is fast to test exhaustively. The plugin classes adapt it to Nav2: they read parameters, handle frames and TF, convert messages, and report errors as Nav2 exceptions. Keep logic out of them. [PLUGINS.md](PLUGINS.md) covers when Nav2 calls them.

## Control law

Each control period, `Pursuit`:

1. Advances the vehicle's progress to the closest point on the path. It searches only one lookahead distance ahead and never moves backward, so a path that crosses itself (a survey loop) is followed in order.
2. Takes the lookahead point, `lookahead_dist` of path beyond the progress, and the path's heading there.
3. Heads straight for the lookahead point. The speed is the lowest of the speed limit in that direction, the speed from which the vehicle can still brake to a stop at the end of the path, and `approach_gain` times the remaining distance. The last term makes the final approach proportional, because the braking curve alone has unbounded gain at zero distance and makes a lagging vehicle chatter around the goal.
4. Turns toward the lookahead heading with the same speed profile.
5. Limits the change from the previous command to `max_accel` times the control period.

Velocity and acceleration limits are per body axis. When one axis's limit binds, the whole vector is scaled rather than that axis clipped, so the vehicle keeps heading straight at its target instead of veering off the path.

## Differences from Orca4

| | Orca4's `orca_nav2` | `rov_nav2_plugins` |
|-|---------------------|--------------------|
| Motion | Diff-drive: surge and yaw rate from path curvature, plus heave | Holonomic: surge, sway, heave and yaw rate |
| Heading | Faces the lookahead point; goal heading ignored | Tracks the path's heading; goal heading checked |
| Goal checker | Horizontal and vertical distance | Horizontal, vertical and heading, all at once |
| Progress checker | 3D distance | 3D distance or turning |
| Planner | Straight line; pose spacing from a truncated count; goal appended | Routes around the costmap's obstacles; always begins at the vehicle and ends exactly at the goal, including turns in place |
| Acceleration limits | Per axis, from a separate `tick_rate` | Direction-preserving, from the controller server's frequency |

## Testing

```bash
colcon test --packages-select rov_nav2_plugins
colcon test-result --verbose
```

- The algorithm tests include closed-loop runs of `Pursuit` against a simulated vehicle, both ideal and with response lag and a water current. Their thresholds sit just above measured performance, so a regression in tracking or arrival time fails them.
- The plugin tests drive each plugin through its Nav2 interface. `test_plugin_loading` loads each plugin by name through pluginlib, as the Nav2 servers do, which catches mistakes in `plugins.xml` and the export macros.
- There is no end-to-end test through the real Nav2 servers in the repository yet.

## Known limitations

- Parameters are read when the plugin's server configures; to change one, reconfigure the server. There are no dynamic parameters yet.
- Each plugin's own limitations, such as the controller ignoring Nav2 speed limits, are in the notes on its page, next to its source, such as [`src/controller/pure_pursuit_controller_3d.md`](src/controller/pure_pursuit_controller_3d.md).
