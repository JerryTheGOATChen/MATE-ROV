# action_bringup

Launch files and configuration for the action module (planning and control). The module runs Nav2 with the ROV's 3D plugins from [`rov_nav2_plugins`](../rov_nav2_plugins).

## Usage

With `action` in `ACTIVE_MODULES`, `watod up` starts `action.launch.yaml`. Nav2 then waits for the vehicle to provide the [vehicle interface](#vehicle-interface).

Once the vehicle is up, send a goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: -1.0}, orientation: {w: 1.0}}}}"
```

To watch, connect Foxglove to the bridge address that `watod up` prints, add a 3D panel with its display frame set to `map`, and turn on `/plan`, `/odom`, `/sonar/points`, `/global_costmap/costmap` and `/global_costmap/voxel_grid` (the costmap's 3D voxels).

| Launch argument | Default | Description |
|-----------------|---------|-------------|
| `autostart` | true | Configure and activate the nodes |
| `bond_enabled` | false | Bond-based crash detection for the module's own lifecycle nodes, managed by `wato_lifecycle_manager` (Nav2 has its own) |

`nav2.launch.yaml` also takes `use_sim_time` (default false), for a simulator that publishes `/clock`, such as Gazebo.

## Vehicle interface

To navigate the real ROV, the vehicle side (interfacing and localization) must provide:

| Interface | Type | Notes |
|-----------|------|-------|
| `/cmd_vel` (subscribe) | `geometry_msgs/TwistStamped` in `base_link` | Velocity in the vehicle frame: x forward, y left, z up, and yaw rate. Nav2 sends zero when navigation ends or is canceled. Stop if commands stop arriving |
| `odom -> base_link` | TF | Vehicle odometry, e.g. from the DVL, IMU and depth sensor |
| `map -> odom` | TF | Localization. Publish an identity transform if odometry is the only position source |
| `/odom` (publish) | `nav_msgs/Odometry` | Read by Nav2's controller server |
| `/sonar/points` (publish) | `sensor_msgs/PointCloud2` | Sonar returns at their real 3D positions, in the sonar's own frame: at the sonar, x along its center beam, with a transform to `map` (see [DEVELOPING.md](DEVELOPING.md#costmaps)). The costmap clears the sonar's field of view from that frame, set in `config/nav2.yaml`. Fills the global costmap that the planner routes around. Without it, the costmap stays empty and paths are nearly straight |

Frames follow REP 103 and REP 105. Because z is up, a depth of 2 m is z = -2.

Start the vehicle side first, or within a minute of the action module: Nav2 gives up if the vehicle's transforms do not appear within 60 s, and then has to be restarted. Raise `initial_transform_timeout` for both costmaps in `config/nav2.yaml` if the vehicle needs longer.
