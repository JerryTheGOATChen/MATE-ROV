# VoxelLayer3D

**Class:** `rov_nav2_plugins::VoxelLayer3D`

**Type:** Costmap layer (`nav2_costmap_2d::Layer`)

**XML:** `costmap_plugins.xml`

Keeps a 3D grid of the sonar's returns, the pool floor included, and inflates it in 3D, for a 3D planner. Nav2's voxel layers, and STVL, flatten every voxel into the 2D costmap, so the floor would make every cell lethal and has to be filtered out. This layer keeps everything in 3D instead:

- **3D grid.** Cubic voxels at the costmap's resolution, over the costmap's x, y window and from `min_z` to `max_z`. Each voxel is unknown, free or occupied. Each cloud marks the voxels between the sonar and each return free, then marks the return's voxel occupied. A voxel not seen for `voxel_decay`, free or occupied, is forgotten and becomes unknown again.
- **3D inflation.** Each voxel gets a cost from its distance to the nearest occupied voxel, with the same profile as Nav2's `InflationLayer`. Vertical distance is scaled by the costmap's inscribed radius over `robot_half_height`, so the clearance kept above the floor and below overhangs is the vehicle's half-height, not its half-width.
- **3D costs.** `cost_volume()` returns the latest costs, and which voxels are known, as an immutable snapshot, for a 3D planner in the same server.
- **No 2D costmap.** Nothing is written into the 2D costmap, which stays empty. Any projection of 3D obstacles into 2D either blocks every depth at an obstacle, so the floor blocks everything, or blocks none, so an obstacle that does not reach the surface disappears. The layer uses the costmap only for its window, frame, footprint and update loop.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Whether the layer takes in clouds and updates its costs. |
| `topic` | string | `/sonar/points` | The `sensor_msgs/PointCloud2` of returns, subscribed with sensor-data QoS. |
| `min_z` | double | `-4.4` | Bottom of the grid in the costmap's frame (meters). Set it below the floor, so floor returns stay in the grid. |
| `max_z` | double | `0.0` | Top of the grid in the costmap's frame (meters), such as the surface. |
| `obstacle_range` | double | `8.0` | Returns farther than this from the sonar are ignored (meters). |
| `voxel_decay` | double | `60.0` | A voxel not seen for this long, free or occupied, becomes unknown (seconds). |
| `robot_half_height` | double | `0.2` | The vehicle's vertical half-height (meters). |
| `inflation_radius` | double | `1.0` | Horizontal distance from an obstacle at which cost reaches zero (meters), as in Nav2's `InflationLayer`. |
| `cost_scaling_factor` | double | `3.0` | How fast cost falls off beyond the inscribed radius (1/meters), as in Nav2's `InflationLayer`. |
| `transform_tolerance` | double | `0.5` | How long a cloud waits for its transform into the costmap's frame before it is dropped (seconds). |
| `publish_voxel_costs` | bool | `false` | Publish the voxels with nonzero cost on `<costmap>/<layer name>/voxel_costs`, as a `PointCloud2` with the cost as `intensity`. Only published while subscribed. |

The inscribed radius is the costmap's own, from its `footprint` or `robot_radius`.

## Notes

- **Inflation.** The layer inflates in 3D itself. Nav2's `InflationLayer` in the same costmap would have nothing to inflate. `inflation_radius` below the inscribed radius is raised to it, as in Nav2, which leaves no cost gradient beyond the vehicle's body.
- **Viewing.** In Foxglove, add `voxel_costs` to a 3D panel, color by `intensity`, and set the point size to the costmap's resolution. The floor appears as a lethal slab with its inflation above it.
- **Clearing.** Clearing comes only from rays to real returns, and forgetting only from decay. Rays with no return are not in the cloud, so they clear nothing: water seen only out to the sonar's range without an echo stays unknown. A ray grazing the floor at a shallow angle clears the floor voxels it passes through. All rays of a cloud are cleared before its returns are marked, so floor the same cloud sees survives, but floor seen only by earlier clouds can be cleared.
- **Unknown space.** Unknown voxels are not obstacles, so their cost comes only from the obstacles seen near them. Whether to enter unknown water is the planner's choice, from `CostVolume::known()`. Most of the water column is unknown at any time, because the sonar sees only a band ahead of the vehicle, and voxels below the floor or inside obstacles are never seen.
- **Window.** The grid follows the costmap's size at configuration. It does not follow a rolling window, so use it in a fixed-window costmap such as the global costmap.
- **Cost.** Inserting a 30,800-return cloud and inflating the 220 x 140 x 44 pool grid take about 35 ms and 30 ms. The costs are recomputed only on updates where a cloud arrived or a voxel was forgotten.
