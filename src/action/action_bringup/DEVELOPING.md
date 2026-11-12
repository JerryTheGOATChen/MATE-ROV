# Developing action_bringup

## Launch structure

- `action.launch.yaml` is the module's entry point, run by docker compose. It includes `nav2.launch.yaml` in a group, because launch includes leak their arguments: Nav2's `params_file` would otherwise reach any other launch file that declares one.
- `nav2.launch.yaml` runs the Nav2 servers the ROV needs: controller, planner and BT navigator, configured by `config/nav2.yaml`.

Nav2 runs under `nav2_lifecycle_manager` rather than `wato_lifecycle_manager`. Nav2 servers bond with Nav2's manager on a shared topic, so if one crashes the others are brought down and the controller stops commanding the vehicle. `wato_lifecycle_manager` bonds on per-node topics that Nav2 servers do not use.

Nav2 components that are not launched: map server and AMCL (the vehicle localizes itself), behavior server (no recoveries), and velocity smoother and collision monitor (2D only).

## OctoMap

`OctomapPlanner3D` owns one persistent `octomap::OcTree` and updates it directly from
`/sonar/points`. A TF message filter waits until each cloud can be transformed from the sonar frame
into `map`. OctoMap's batch point-cloud insertion ray-traces free space from the transformed sensor
origin to each return, marks in-range endpoints occupied, treats returns beyond maximum range as
free-only rays, applies the configured hit/miss probability model, and prunes the tree.

The plugin no longer launches `octomap_server` or publishes and decodes `/octomap_binary`. For A*,
it derives a compact 0.25 m collision lattice from the authoritative tree. The lattice is cached by
map revision, so multiple planning requests against the same sonar map reuse it; a sonar update
invalidates it. Planning takes a shared map lock and point-cloud insertion takes an exclusive lock.

`/occupied_cells_vis_array` remains available for Foxglove. Marker generation is limited to 1 Hz
and skipped when there are no subscribers. Set `visualization.enabled: false` in `config/nav2.yaml`
on a compute-constrained vehicle that does not need live occupied-cell visualization.

`OctomapPlanner3D` performs A* in x, y and z, so a route can pass beside, above or below an obstacle. It treats the ROV as a 0.45 m sphere, which includes clearance around its roughly 0.35 m half-width, and validates simplified path segments against the map. Heights use REP 103: z is up, so underwater positions are negative.

The current simulation uses `unknown_is_occupied: false`, allowing a route through water the sonar has not observed. Set it to true for conservative operation, but initial navigation will then require enough sensor coverage to connect the start and goal through known-free cells. The map resolution, sensor range, probability model, planning bounds and collision settings all live in `config/nav2.yaml`.

Nav2 still constructs empty local and global 2D costmaps because its planner and controller servers require `Costmap2DROS` objects for frame and lifecycle plumbing. They contain no obstacle source and are not used by the OctoMap planner.

## Behavior trees

Neither tree has recovery behaviors: a failure goes back to whoever sent the goal, such as the pilot or a mission script.

- `navigate_to_pose_3d.xml` replans once a second, so the route avoids obstacles the sonar finds on the way.
- `navigate_through_poses_3d.xml` plans when the goals arrive or change. The planner server joins the legs through every waypoint into one path, so the vehicle does not stop at intermediate waypoints. It does not replan on the way: Nav2's `RemovePassedGoals`, needed to replan from the vehicle's position, drops a waypoint as soon as the vehicle comes near it, which cuts the corners of a survey.

## Current hacks

- The local costmap loads a single inflation layer with nothing to inflate. Nav2's controller server requires a costmap, the controller ignores it, and a ROS parameter file cannot express an empty plugin list.
- The OctoMap planner bounds and Nav2's unused global-costmap window are sized for a 20 x 12 m pool, 4 m deep. Resize both for the competition pool.

## Testing

The package has no tests of its own. The plugins, including closed-loop runs of the controller against a simulated vehicle, are tested in [`rov_nav2_plugins`](../rov_nav2_plugins/DEVELOPING.md#testing). There is no end-to-end test through the Nav2 servers in the repository yet.
