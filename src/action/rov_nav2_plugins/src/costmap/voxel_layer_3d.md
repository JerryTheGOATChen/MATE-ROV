# VoxelLayer3D

**Class:** `rov_nav2_plugins::VoxelLayer3D`

**Type:** Costmap layer (`nav2_costmap_2d::Layer`)

**XML:** `costmap_plugins.xml`

Keeps a 3D grid of the sonar's returns, the pool floor included, and inflates it in 3D. Nav2's voxel layers, and STVL, flatten every voxel into the 2D costmap, so the floor would make every cell lethal and has to be filtered out. This layer keeps the floor in 3D instead:

- **3D grid.** Cubic voxels at the costmap's resolution, over the costmap's x, y window and from `min_z` to `max_z`. Each cloud clears the voxels between the sonar and each return, then marks the return's voxel. A voxel not seen for `voxel_decay` is cleared.
- **3D inflation.** Each voxel gets a cost from its distance to the nearest marked voxel, with the same profile as Nav2's `InflationLayer`. Vertical distance is scaled by the costmap's inscribed radius over `robot_half_height`, so the clearance kept above the floor and below overhangs is the vehicle's half-height, not its half-width.
- **2D costmap.** Each cell gets its column's lowest cost: a cell is blocked only when every depth in it is, such as at the pool walls, and the floor leaves the 2D costmap free.
- **3D costs.** `cost_volume()` returns the latest 3D costs as an immutable snapshot, for a 3D planner in the same server.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Whether the layer updates and writes to the costmap. |
| `topic` | string | `/sonar/points` | The `sensor_msgs/PointCloud2` of returns, subscribed with sensor-data QoS. |
| `min_z` | double | `-4.4` | Bottom of the grid in the costmap's frame (meters). Set it at or below the floor. |
| `max_z` | double | `0.0` | Top of the grid in the costmap's frame (meters), such as the surface. |
| `obstacle_range` | double | `8.0` | Returns farther than this from the sonar are ignored (meters). |
| `voxel_decay` | double | `60.0` | A voxel not seen for this long is cleared (seconds). |
| `robot_half_height` | double | `0.2` | The vehicle's vertical half-height (meters). |
| `inflation_radius` | double | `1.0` | Horizontal distance from an obstacle at which cost reaches zero (meters), as in Nav2's `InflationLayer`. |
| `cost_scaling_factor` | double | `3.0` | How fast cost falls off beyond the inscribed radius (1/meters), as in Nav2's `InflationLayer`. |
| `transform_tolerance` | double | `0.5` | How long a cloud waits for its transform into the costmap's frame before it is dropped (seconds). |
| `publish_voxel_costs` | bool | `false` | Publish the voxels with nonzero cost on `<costmap>/<layer name>/voxel_costs`, as a `PointCloud2` with the cost as `intensity`. Only published while subscribed. |

The inscribed radius is the costmap's own, from its `footprint` or `robot_radius`.

## Notes

- **Inflation.** The layer inflates in 3D itself, so do not add Nav2's `InflationLayer` to the same costmap: it would inflate the 2D projection a second time.
- **Viewing.** In Foxglove, add `voxel_costs` to a 3D panel, color by `intensity`, and set the point size to the costmap's resolution. The floor appears as a lethal slab with its inflation above it.
- **Clearing.** Clearing comes only from rays to real returns and from decay. Rays with no return are not in the cloud, so they clear nothing. A ray grazing the floor at a shallow angle clears the floor voxels it passes through. All rays of a cloud are cleared before its returns are marked, so floor the same cloud sees survives, but floor seen only by earlier clouds can be cleared.
- **Unknown space.** Voxels never seen are free, as in Nav2's 2D layers without `track_unknown_space`.
- **Window.** The grid follows the costmap's size at configuration. It does not follow a rolling window, so use it in a fixed-window costmap such as the global costmap.
- **Cost.** Inserting a 30,800-return cloud and inflating the 220 x 140 x 44 pool grid take about 35 ms and 30 ms. The costs are recomputed only on updates where a cloud arrived or a voxel decayed.
