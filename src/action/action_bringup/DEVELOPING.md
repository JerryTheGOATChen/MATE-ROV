# Developing action_bringup

## Launch structure

- `action.launch.yaml` is the module's entry point, run by docker compose. It includes `nav2.launch.yaml` in a group, because launch includes leak their arguments: Nav2's `params_file` would otherwise reach any other launch file that declares one.
- `nav2.launch.yaml` runs the Nav2 servers the ROV needs: controller, planner and BT navigator, configured by `config/nav2.yaml`.

Nav2 runs under `nav2_lifecycle_manager` rather than `wato_lifecycle_manager`. Nav2 servers bond with Nav2's manager on a shared topic, so if one crashes the others are brought down and the controller stops commanding the vehicle. `wato_lifecycle_manager` bonds on per-node topics that Nav2 servers do not use.

Nav2 components that are not launched: map server and AMCL (the vehicle localizes itself), behavior server (no recoveries), and velocity smoother and collision monitor (2D only).

## Costmaps

The global costmap holds the sonar's obstacles for `CostmapPlanner3D`. Its layer is the stock [spatio-temporal voxel layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) (STVL). STVL marks each return as a 3D voxel, then flattens the voxels into the 2D costmap, so an obstacle at any depth blocks its x, y. An obstacle the sonar no longer sees is forgotten after 60 s, and water the sonar sees without a return clears much sooner. The memory is long because the sonar looks only ahead while the vehicle also moves sideways and backward: with a short memory, the vehicle can back into an obstacle it has just gone around.

The inflation layer keeps the route well clear of obstacles, more than the vehicle's 0.35 m radius needs, because pure pursuit cuts inside the route's corners. In simulation the hull stays about 0.6 m from obstacles it goes around.

Heights are z in the `map` frame, which is negative underwater. Returns within 0.2 m of the floor, below z = -3.8 m, are ignored. A sonar that measures height, like the team's, sees the floor ahead of the vehicle, and those returns are the floor itself. A sonar that can't measure height would place floor echoes at the vehicle's depth, where this can't catch them. In testing with one, the floor became a wall that blocked every depth and never cleared.

The costmap is 2D on purpose for now. Going over and under obstacles needs a 3D planner that reads a 3D map, planned for once the team's 3D sonar is publishing. The controller, checkers and behavior trees are already 3D.

## Behavior trees

Neither tree has recovery behaviors: a failure goes back to whoever sent the goal, such as the pilot or a mission script.

- `navigate_to_pose_3d.xml` replans once a second, so the route avoids obstacles the sonar finds on the way.
- `navigate_through_poses_3d.xml` plans when the goals arrive or change. The planner server joins the legs through every waypoint into one path, so the vehicle does not stop at intermediate waypoints. It does not replan on the way: Nav2's `RemovePassedGoals`, needed to replan from the vehicle's position, drops a waypoint as soon as the vehicle comes near it, which cuts the corners of a survey.

## Current hacks

- The local costmap loads a single inflation layer with nothing to inflate. Nav2's controller server requires a costmap, the controller ignores it, and a ROS parameter file cannot express an empty plugin list.
- The global costmap is sized for a 20 x 12 m pool, 4 m deep. Resize it, and its floor height, for the competition pool.

## Testing

The package has no tests of its own. The plugins, including closed-loop runs of the controller against a simulated vehicle, are tested in [`rov_nav2_plugins`](../rov_nav2_plugins/DEVELOPING.md#testing). There is no end-to-end test through the Nav2 servers in the repository yet.
