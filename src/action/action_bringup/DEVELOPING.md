# Developing action_bringup

## Launch structure

- `action.launch.yaml` is the module's entry point, run by docker compose. It includes `nav2.launch.yaml` in a group, because launch includes leak their arguments: Nav2's `params_file` would otherwise reach any other launch file that declares one.
- `nav2.launch.yaml` runs the Nav2 servers the ROV needs: controller, planner and BT navigator, configured by `config/nav2.yaml`.

Nav2 runs under `nav2_lifecycle_manager` rather than `wato_lifecycle_manager`. Nav2 servers bond with Nav2's manager on a shared topic, so if one crashes the others are brought down and the controller stops commanding the vehicle. `wato_lifecycle_manager` bonds on per-node topics that Nav2 servers do not use.

Nav2 components that are not launched: map server and AMCL (the vehicle localizes itself), behavior server (no recoveries), and velocity smoother and collision monitor (2D only).

## Costmaps

The global costmap holds the sonar's obstacles in 3D, in `rov_nav2_plugins`' [`VoxelLayer3D`](../rov_nav2_plugins/src/costmap/voxel_layer_3d.md). The layer marks every return as a voxel, the floor included, clears the water between the sonar and each return, and inflates the voxels in 3D. It remembers each voxel as unknown, free or occupied, so a planner can tell water the sonar has seen from water it hasn't. What the sonar no longer sees is forgotten after 200 s. The memory is long because the sonar looks only ahead while the vehicle also moves sideways and backward: with a short memory, the vehicle can back into an obstacle it has just gone around.

Nothing is written into the 2D costmap, which stays empty. Projecting 3D obstacles into 2D either blocks every depth at an obstacle, so the floor blocks everything, or blocks none, so a box on the floor disappears. The layer only borrows the costmap's window, frame, footprint and update loop, and `VoxelPlanner3D`, in the same planner server, reads its costs directly.

The inflation keeps the route well clear of obstacles, more than the vehicle's 0.35 m radius needs, because pure pursuit cuts inside the route's corners. Vertically the clearance scales with the vehicle's half-height: cost reaches zero 0.57 m above the floor.

Heights are z in the `map` frame, which is negative underwater. The grid runs from below the floor to the surface. It needs a sonar that measures height, like the team's: a sonar that can't would place floor echoes at the vehicle's depth, and the floor would become a wall at every depth the vehicle visits.

`VoxelPlanner3D` routes over, under or around obstacles through those costs. It prefers water the sonar has seen (`unknown_cost`) and going around to going over or under (`z_weight`), but it will cross unseen water, so the behavior tree's replanning once a second is what keeps the route clear of obstacles found on the way. In a closed-loop test of a route over a 0.9 m block, the hull stays 0.2 m clear with a vehicle that lags its commands by 0.5 s.

## Behavior trees

Neither tree has recovery behaviors: a failure goes back to whoever sent the goal, such as the pilot or a mission script.

- `navigate_to_pose_3d.xml` replans once a second, so the route avoids obstacles the sonar finds on the way.
- `navigate_through_poses_3d.xml` plans when the goals arrive or change. The planner server joins the legs through every waypoint into one path, so the vehicle does not stop at intermediate waypoints. It does not replan on the way: Nav2's `RemovePassedGoals`, needed to replan from the vehicle's position, drops a waypoint as soon as the vehicle comes near it, which cuts the corners of a survey.

## Current hacks

- The local costmap loads a single inflation layer with nothing to inflate. Nav2's controller server requires a costmap, the controller ignores it, and a ROS parameter file cannot express an empty plugin list.
- The global costmap is sized for a 20 x 12 m pool, 4 m deep. Resize it, and the voxel layer's `min_z`, for the competition pool.

## Testing

The package has no tests of its own. The plugins, including closed-loop runs of the controller against a simulated vehicle, are tested in [`rov_nav2_plugins`](../rov_nav2_plugins/DEVELOPING.md#testing). There is no end-to-end test through the Nav2 servers in the repository yet.
