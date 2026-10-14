# Developing rov_gazebo_sim

## Structure

Everything is stock Gazebo and ROS; the package has no code of its own.

- `worlds/mate_pool.sdf`: the pool and its props, all static. Gravity is off, standing in for a vehicle trimmed neutrally buoyant.
- `models/rov.sdf.xacro`: the ROV, spawned separately so xacro can set its sensor noise from launch arguments.
- `config/bridge.yaml`: the topics `ros_gz_bridge` carries between Gazebo and ROS.
- `config/ekf.yaml`: `robot_localization`'s EKF.
- `launch/gazebo.launch.yaml`: the simulator in place of the vehicle and its localization. `launch/sim.launch.yaml` adds `action_bringup`'s Nav2.

## Vehicle model

Gazebo's VelocityControl system moves the ROV at the commanded body velocity, with no lag or drag. The kinematic simulator in `rov_kinematic_sim` models the response lag and a water current, so the two simulators cover different things. A thruster-level model would need Gazebo's thruster, buoyancy and hydrodynamics systems, plus a velocity controller and thruster allocation for the real ROV's layout.

## Sensors

Gazebo has no DVL or depth sensor, so the ROV carries three of Gazebo's odometry publishers. Each publishes the vehicle's pose and velocity, with its own noise, and reports the noise as covariance.

| Publisher | Stands in for | What the EKF fuses |
|-----------|---------------|--------------------|
| `/sensors/dvl`, 10 Hz | DVL | Velocity in `base_link`: vx, vy, vz |
| `/sensors/depth`, 20 Hz | Depth sensor | z |
| `/sim/ground_truth`, 20 Hz, no noise | Nothing | Nothing; it is for scoring |

The IMU is Gazebo's, with noise on its turn rates. Gazebo doesn't add noise to an IMU's orientation, so the EKF's heading only drifts through the turn rates. Nothing measures x and y, so position drifts slowly, as it would with a real DVL.

The sonar is a GPU lidar with a 3D imaging sonar's field of view, so every return has a real height. Its floor returns land at z = -4, where `action_bringup`'s costmap ignores everything below -3.8 m. A sonar that can't measure height would put floor echoes at the vehicle's depth instead, which is why the costmap needs height-measuring data.

## Current hacks

- It lives under `src/simulation/`, which no Docker image copies, so it runs on the host. In a container it would need Gazebo in the image and a GPU for the sonar. A real simulation module would need its own Dockerfile and compose service, which is an infrastructure change.
- Sensor noise is off by default for now, so the missions test navigation alone. The noise launch arguments turn it back on; tuning the missions for noisy localization comes later.
- Keep colons out of comments in `models/rov.sdf.xacro`. The spawn launch file passes the generated model as a ROS parameter, which launch parses as YAML.

## Testing

```bash
colcon test --packages-select rov_gazebo_sim
colcon test-result --verbose
```

`test/test_pool_missions.py` is a launch test. It runs `sim.launch.yaml` with Gazebo headless, then flies `missions/pool_missions.yaml` through `rov_mission_runner`, scored on the ground truth. It runs in ROS domain 97 and its own Gazebo partition, so it cannot cross-talk with other tests or a running simulation.

When changing the pool, keep it inside `action_bringup`'s global costmap: 22 x 14 m centered on the origin, and the water column from -4 to 0 m.
