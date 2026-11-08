# VoxelPlanner3D

**Class:** `rov_nav2_plugins::VoxelPlanner3D`

**Type:** Planner (`nav2_core::GlobalPlanner`)

**XML:** `plugins.xml`

Routes over, under or around obstacles in 3D, through the costs [`VoxelLayer3D`](../costmap/voxel_layer_3d.md) keeps in the global costmap. Each plan reads the layer's latest costs and:

- **Searches** the voxel grid with A*, stepping to any of a voxel's 26 neighbors. A step costs its length times `1 + cost_weight * c / 252`, where c is the cost of the voxel it enters, so the route keeps away from inflated obstacles. Depth change counts `z_weight` times as much as horizontal travel. Water the sonar hasn't seen costs at least `unknown_cost`, so the route prefers water the sonar has seen. The route is the cheapest through the grid.
- **Keeps clear.** The route never enters a voxel within the vehicle's half-width, or half-height vertically, of an obstacle, and never cuts diagonally past the edge of one.
- **Shortens** the route through voxel centers into straight segments. A segment replaces a stretch of the route when it crosses only voxels the route could enter and costs no more than the stretch.
- **Adds headings**, turning evenly from the vehicle's heading to the goal's, or with `yaw_mode: face_travel`, facing along each segment. Going with 'face_travel' because it's more realistic

With nothing in the way, the route is a straight line.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `voxel_layer` | string | `voxel_layer_3d` | Name of the `VoxelLayer3D` in the planner server's costmap. |
| `cost_weight` | double | `2.0` | How much a voxel's cost adds to the cost of passing through it, like Smac's `cost_travel_multiplier`. `0` ignores costs, so the route hugs obstacles. |
| `unknown_cost` | int | `40` | The least cost of a voxel the sonar hasn't seen, from `0` to `252`. Higher keeps the route in water the sonar has seen. `0` treats unseen water as free. |
| `z_weight` | double | `1.5` | How many meters of travel one meter of depth change counts as. Higher prefers going around to going over or under. |
| `max_planning_time` | double | `1.0` | Time allowed for the search (seconds). |
| `yaw_mode` | string | `interpolate` | `interpolate`: turn evenly from the vehicle's heading to the goal's, in proportion to distance. `face_travel`: face along each segment no steeper than 45 degrees, turning in place between segments and at the ends, so the forward-looking sonar sees where the vehicle is going. |

## Notes

- **The layer.** The planner finds `VoxelLayer3D` by name when it configures, and fails if the costmap has none. The layer must be in the planner server's own costmap, the global costmap. Planning reads an immutable snapshot of the layer's costs, so it never waits for a costmap update.
- **Path ends.** The path always begins at the vehicle's pose and ends exactly at the goal. A goal at the vehicle's position gives a path that stays there, so the vehicle turns in place.
- **Close to obstacles.** A goal within the vehicle's size of an obstacle fails with `GoalOccupied`, so goals close to the floor must be at least `robot_half_height` above it. A vehicle that has drifted that close to an obstacle can still plan away from it; one inside an obstacle fails with `StartOccupied`.
- **Failures.** Start or goal outside the grid fail with `StartOutsideMapBounds` or `GoalOutsideMapBounds`; obstacles blocking every route with `NoValidPathCouldBeFound`; running out of time with `PlannerTimedOut`; a canceled request with `PlannerCancelled`. Before the sonar has seen anything, every voxel is unknown and free, so the route is a straight line.
- **Unseen water.** The sonar sees only a band ahead of the vehicle, so the route can pass through unseen water, where an obstacle may be. `navigate_to_pose_3d.xml` replans once a second, so the route moves as the sonar sees more. `navigate_through_poses_3d.xml` plans once (see its comment).
- **Frames.** The planner server transforms the start and goal into the global costmap's frame (`map`) before planning, and the path is in that frame.
- **Cost.** On the 220 x 140 x 44 pool grid, routes around the pool's obstacles take 5 to 125 ms on a desktop, a 22 m route corner to corner about 0.4 s, and finding there is no route, which searches the whole pool, about 0.45 s. The Raspberry Pi is several times slower, so long routes may need a larger `max_planning_time`.
