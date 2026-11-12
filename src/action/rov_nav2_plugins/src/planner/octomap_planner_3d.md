# OctomapPlanner3D

**Class:** `rov_nav2_plugins::OctomapPlanner3D`

This Nav2 global planner performs a 26-connected A* search through an OctoMap in x, y and z. It can route beside, above or below sonar obstacles instead of flattening every return into a 2D costmap.

The planner owns one persistent `octomap::OcTree`. It waits for the sonar cloud's transform into the
map frame, transforms every finite return, and passes the transformed endpoints and sensor origin to
OctoMap's batch point-cloud insertion. The library ray-traces free space, gives occupied endpoints
priority within a scan, applies the configured inverse sensor model, and prunes the tree.

For planning, it models the ROV as a sphere and derives a compact, inflated collision lattice from
the tree. The lattice is cached until a sonar update changes the map. Final path segments use the
same conservative cache, and unnecessary grid stair-steps are removed. Path heading is interpolated
from the start yaw to the goal yaw.

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `pointcloud_topic` | `/sonar/points` | Sonar `sensor_msgs/PointCloud2` input. |
| `map_frame` | `map` | Frame in which the persistent tree is stored. |
| `map_resolution` | `0.1` | OctoMap leaf resolution in metres. |
| `tf_queue_size` | `5` | Sonar clouds retained while waiting for TF. |
| `tf_filter_tolerance` | `5.0` | Maximum seconds a cloud waits for its map-frame transform. |
| `sensor_model.max_range` | `8.0` | Maximum occupied-return range; longer rays only clear free space to this range. |
| `sensor_model.hit` | `0.7` | Occupied update probability. |
| `sensor_model.miss` | `0.4` | Free update probability. |
| `sensor_model.min` | `0.12` | Minimum clamped occupancy probability. |
| `sensor_model.max` | `0.97` | Maximum clamped occupancy probability. |
| `sensor_model.occupancy` | `0.5` | Probability above which a node is occupied. |
| `compress_map` | `true` | Prune equivalent child nodes after every scan. |
| `visualization.enabled` | `true` | Publish occupied-cell markers when subscribed. |
| `visualization.publish_interval` | `1.0` | Minimum seconds between marker updates; `0` accepts every scan. |
| `visualization.topic` | `/occupied_cells_vis_array` | Optional occupied-cell marker topic. |
| `visualization.min_z` | `-4.0` | Lowest occupied leaf included in visualization. |
| `visualization.max_z` | `0.0` | Highest occupied leaf included in visualization. |
| `planning_resolution` | `0.25` | A* lattice spacing in metres. Smaller values give finer routes but cost more memory and time. |
| `collision_radius` | `0.45` | Spherical vehicle radius plus desired clearance, in metres. |
| `min_bounds` | `[-11, -7, -4]` | Minimum x, y and z planning bounds in the map frame. |
| `max_bounds` | `[11, 7, 0]` | Maximum x, y and z planning bounds in the map frame. |
| `unknown_is_occupied` | `false` | Whether unobserved OctoMap space is forbidden. |
| `max_expansions` | `250000` | Maximum A* states expanded for one plan. |

The point-cloud filter bounds `pointcloud_min_x`, `pointcloud_max_x`, `pointcloud_min_y`,
`pointcloud_max_y`, `pointcloud_min_z`, and `pointcloud_max_z` match `octomap_server` defaults and
can be used to discard transformed returns before insertion.

The planner fails a request until a valid sonar cloud has been inserted. It also fails when either
endpoint is occupied or outside the configured bounds, the map frame does not match the planner
frame, or no route is found within the expansion limit.
