# CostmapPlanner3D

**Class:** `rov_nav2_plugins::CostmapPlanner3D`

**Type:** Planner (`nav2_core::GlobalPlanner`)

**XML:** `plugins.xml`

Routes around the obstacles in the global costmap in 3D. A stock Nav2 2D planner, Smac 2D by default, finds the horizontal route through the costmap. The plugin then adds depth and heading:

- **Depth** changes evenly along the horizontal route. With `z_before_xy`, it changes first, in place.
- **Heading** turns evenly from the vehicle's to the goal's, the shorter way around.

With nothing in the way, the route is nearly a straight line.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `xy_planner` | string | `nav2_smac_planner::SmacPlanner2D` | The 2D planner plugin. Its own parameters go under `<plugin name>.xy`, such as `CostmapPlanner3D.xy.max_planning_time`; see the [Smac 2D parameters](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html). Set its `tolerance` to 0 (see Failures). |
| `z_before_xy` | bool | `false` | Change depth first, then move horizontally. |

## Notes

- **Flat costmap.** The costmap is 2D, so an obstacle at any depth blocks its x, y. The vehicle goes around obstacles, never over or under them. Planning over and under needs a 3D map, such as OctoMap, and a 3D planner.
- **Path ends.** The path always begins at the vehicle's pose and ends exactly at the goal. A goal at the vehicle's position gives a path that stays there, so the vehicle turns in place.
- **Failures.** When obstacles block every route, planning fails with the 2D planner's error, usually `NoValidPathCouldBeFound`. A goal inside an obstacle fails with `GoalOccupied`, provided the 2D planner's `tolerance` is 0. With a tolerance, Smac ends its route at a free cell near the goal, and the path's last leg runs from there into the obstacle, because the path always ends exactly at the goal.
- **Frames.** The planner server transforms the start and goal into the global costmap's frame (`map`) before planning, and the path is in that frame.
- **Replanning.** `navigate_to_pose_3d.xml` replans once a second, so the route follows what the sonar has seen so far. `navigate_through_poses_3d.xml` plans once (see its comment).
