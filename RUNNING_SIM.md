# Running Nav2 (watod) against the Gazebo pool sim

How to fly the action module's Nav2, running in its watod container, in the Gazebo pool sim, which runs on the host. This is a stopgap until the sim has its own watod module.

| Piece | Where it runs | Why |
|-------|---------------|-----|
| Nav2 (`action_bringup`) | `action_bringup_dev` container | Same as on the real ROV |
| Zenoh router, Foxglove bridge | `infrastructure_bringup` container | Started automatically by `watod up` |
| Gazebo sim (`rov_gazebo_sim`) | Host | No Docker image copies `src/simulation/`, and the sim needs Gazebo and a GPU |
| Mission runner (`rov_mission_runner`) | Host | Its missions file lives in `src/simulation/`, which the container doesn't mount |

They talk over Zenoh: the containers use `network_mode: host`, so a host process on the same RMW and domain joins the same network.

## One-time setup (host)

```bash
# Zenoh RMW, so the host can join the containers' ROS network
sudo apt install ros-jazzy-rmw-zenoh-cpp

# Foxglove app: the container's foxglove_bridge (3.6.0) only accepts the newer
# foxglove.sdk.v1 protocol, which foxglove-studio 3.2.1 doesn't speak
sudo apt update && sudo apt install --only-upgrade foxglove-studio
```

Build only the sim and the mission runner on the host:

```bash
cd ~/MATE-ROV
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/simulation src/action/rov_mission_runner \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

- No `rosdep install` is needed, because the sim's dependencies (`ros_gz_sim`, `ros_gz_bridge`, `robot_localization`, `tf2_ros`, `xacro`) are already on the host.
- `--base-paths` limits colcon to these packages. With `--packages-select`, colcon fails looking for `action_bringup` and the other packages in `src/`, which are built in the container instead.
- `-DPython3_EXECUTABLE` stops CMake from picking up pyenv's Python, which has no `catkin_pkg`.
- Rerun the build after changing anything under `src/simulation/`. This writes `build/`, `install/` and `log/` in the repo root, which git ignores and the container doesn't see.

## Host environment

Every host shell that talks to the sim or Nav2 needs:

```bash
cd ~/MATE-ROV
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=81
```

`81` is watod's default for this user: UID 1001 % 230. Check with `docker exec <container> env | grep ROS_DOMAIN_ID` if in doubt. Don't also source `~/ros2_jazzy_ws`, because its old `rov_navigation` would shadow packages.

## Running it

Run these in order, each in its own terminal.

**1. Containers.** `up`, not just `-t`, so the infrastructure container starts too, with the Zenoh router and Foxglove.
```bash
./watod -m action:dev up
```

**2. Nav2, in the container, with sim time.**
```bash
./watod -t action_bringup_dev
# inside the container
colcon build && source install/setup.bash
ros2 launch action_bringup nav2.launch.yaml use_sim_time:=true
```
Launch `nav2.launch.yaml`, not `action.launch.yaml`: `action.launch.yaml` doesn't pass `use_sim_time` through, so Nav2 would ignore Gazebo's clock.

**3. The sim, on the host** (with the host environment above).
```bash
ros2 launch rov_gazebo_sim gazebo.launch.yaml gui:=true
```
Use `gazebo.launch.yaml`, not `sim.launch.yaml`. `sim.launch.yaml` starts its own Nav2 alongside the container's.

**4. The missions, on the host** (with the host environment above).
```bash
ros2 run rov_mission_runner mission_runner \
  $(ros2 pkg prefix --share rov_gazebo_sim)/missions/pool_missions.yaml \
  --truth-topic /sim/ground_truth
```

## Watching in Foxglove

- Connect with **Foxglove WebSocket** to **`ws://localhost:20022`**. watod gives each user their own ports, so it isn't 8765. The port is watod's `FOXGLOVE_BRIDGE_PORT`.
- Don't start a `foxglove_bridge` on the host, because the infrastructure container already runs one.
- Import [`config/rov_config.json`](config/rov_config.json) as a layout (**Layouts → Import**). It's a 3D panel following `map` with these topics:
  - `/plan`
  - `/sonar/points`
  - `/global_costmap/voxel_grid`
  - `/global_costmap/costmap` (off by default)

  You can also add `/odom` (the EKF's estimate) and `/sim/ground_truth` (where the ROV really is).

The bridge's image has no `nav2_msgs` or `map_msgs`, so Foxglove can't decode `/behavior_tree_log`, the `*_raw` and `*_updates` costmap topics, or Nav2's services. None of the topics in the layout are affected.

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Host `ros2 topic list` shows no Nav2 topics | Is `infrastructure_bringup` running (`docker ps`)? Is the router listening on 7447 (`ss -ltn \| grep 7447`)? Are `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` exported in this shell? |
| `Package 'rov_mission_runner' not found` / `'rov_gazebo_sim' not found` | Rerun the host build, then `source install/setup.bash` |
| `colcon` fails with `No module named 'catkin_pkg'` | Add `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` |
| Foxglove shows no topics | It isn't connected. Check that you're on port 20022 and the app is up to date. `ss -tnp \| grep 20022` should show a connection |
| Mission runner waits forever for Nav2 | Nav2 isn't up or isn't reachable. Check step 2's output, then the first row of this table |
| Nav2 never gets TF or odom | Check that Nav2 was launched with `use_sim_time:=true` and the sim is running (`ros2 topic echo /clock --once`) |

## Going further

A `simulation` watod module would remove all the host setup: a Dockerfile with Gazebo Harmonic, a compose service with GPU and X11 access, and a mount of `src/simulation`. Then running the sim would be `./watod -m simulation action:dev up`.

### Getting Intellisense to work on VSCode
```
./watod -s action
```