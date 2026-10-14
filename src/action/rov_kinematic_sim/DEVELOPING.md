# Developing rov_kinematic_sim

## Model

The vehicle has four degrees of freedom (x, y, z, yaw) and stays level. Each update:

1. The command is clamped to the thruster limits.
2. The velocity through the water approaches the command with a first-order lag (`response_time_s`). This lumps thruster spin-up, drag and added mass into one time constant, which is the dynamic that matters most to a path-following controller.
3. The vehicle moves at that velocity, rotated into the world frame, plus `water_current`.
4. If it would rise above `surface_z`, it stops at the surface.

`step()` in `rov_kinematics.hpp` is a pure function, so the model is unit tested without ROS. `KinematicSimNode` only handles parameters, the command timeout, and publishing.

The integration step is the nominal period, `1 / update_rate_hz`. That keeps runs repeatable; if the timer runs late under load, the vehicle simply moves slower than real time.

## What it does not model

Roll and pitch, thruster allocation, hydrodynamic coupling between axes, sensors and their noise, localization drift (the `map -> odom` transform is the identity), and obstacles. For those, use the Gazebo pool in [`rov_gazebo_sim`](../../simulation/rov_gazebo_sim). This simulator exists to test navigation logic quickly and repeatably, including in automated tests.

## Testing

```bash
colcon test --packages-select rov_kinematic_sim
colcon test-result --verbose
```

The node tests use their own namespace and frame names, so they cannot disturb other tests or a running stack.

`test/test_sim_acceptance.py` is a launch test: it launches `sim.launch.yaml`, runs `rov_mission_runner` on `missions/sim_acceptance.yaml`, and passes if the simulator loaded its config and every mission passes. It runs in ROS domain 98 so it cannot cross-talk with other tests or with a stack running in a DevContainer.

When tuning `action_bringup/config/nav2.yaml`, keep `max_speed` in `missions/sim_acceptance.yaml` equal to the controller's velocity limits, and the mission tolerances at least the goal checker's.
