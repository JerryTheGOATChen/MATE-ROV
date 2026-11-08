# OctomapPlanner3D

**Class:** `rov_nav2_plugins::OctomapPlanner3D`

This Nav2 global planner performs a 26-connected A* search through an OctoMap in x, y and z. It can route beside, above or below sonar obstacles instead of flattening every return into a 2D costmap.

The planner subscribes to the latest binary OctoMap with transient-local reliability. It models the ROV as a sphere, inflates occupied cells by that radius, validates the final path segments against the OctoMap, and removes unnecessary grid stair-steps. Path heading is interpolated from the start yaw to the goal yaw.

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `octomap_topic` | `/octomap_binary` | Binary `octomap_msgs/Octomap` input. |
| `map_update_interval` | `0.5` | Minimum seconds between full-tree decodes. Set to `0` to accept every map. |
| `planning_resolution` | `0.25` | A* lattice spacing in metres. Smaller values give finer routes but cost more memory and time. |
| `collision_radius` | `0.45` | Spherical vehicle radius plus desired clearance, in metres. |
| `min_bounds` | `[-11, -7, -4]` | Minimum x, y and z planning bounds in the map frame. |
| `max_bounds` | `[11, 7, 0]` | Maximum x, y and z planning bounds in the map frame. |
| `unknown_is_occupied` | `false` | Whether unobserved OctoMap space is forbidden. |
| `max_expansions` | `250000` | Maximum A* states expanded for one plan. |

The planner fails a request until an OctoMap has arrived. It also fails when either endpoint is occupied or outside the configured bounds, the map frame does not match the planner frame, or no route is found within the expansion limit.
