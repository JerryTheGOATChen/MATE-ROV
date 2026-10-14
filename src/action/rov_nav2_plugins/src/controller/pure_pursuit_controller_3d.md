# PurePursuitController3D

**Class:** `rov_nav2_plugins::PurePursuitController3D`

**Type:** Controller (`nav2_core::Controller`)

**XML:** `plugins.xml`

Follows the path with 3D pure pursuit for a holonomic vehicle. Each control period it heads for the point `lookahead_dist` along the path ahead of the vehicle, and turns toward the path's heading there. It commands surge, sway, heave and yaw rate in the vehicle's frame (`cmd_vel` x forward, y left, z up, and angular z), within per-axis velocity and acceleration limits, and slows to a stop at the end of the path. The control law is in [DEVELOPING.md](../../DEVELOPING.md#control-law).

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `lookahead_dist` | double | `0.6` | Distance along the path ahead of the vehicle to steer toward (meters). Longer is smoother but cuts corners more. |
| `approach_gain` | double | `1.5` | Speed per meter (or radian) of remaining error on the final approach (1/s). Higher arrives sooner, but a sluggish vehicle may oscillate. |
| `max_vel_x` | double | `0.5` | Surge speed limit (m/s). |
| `max_vel_y` | double | `0.3` | Sway speed limit (m/s). |
| `max_vel_z` | double | `0.3` | Heave speed limit (m/s). |
| `max_vel_yaw` | double | `0.6` | Yaw rate limit (rad/s). |
| `max_accel_x` | double | `0.4` | Surge acceleration limit (m/s²). |
| `max_accel_y` | double | `0.3` | Sway acceleration limit (m/s²). |
| `max_accel_z` | double | `0.3` | Heave acceleration limit (m/s²). |
| `max_accel_yaw` | double | `0.6` | Yaw acceleration limit (rad/s²). |
| `transform_tolerance` | double | `0.1` | Time to wait for the transform of the vehicle's pose into the path's frame (seconds). |

The controller also reads the controller server's `controller_frequency` (default `20.0`) and uses its period for the acceleration limits.

## Notes

- The vehicle must be able to sway (vectored thrusters, like a BlueROV2 Heavy). A vehicle without lateral thrusters needs Orca4's diff-drive heading policy instead.
- The measured velocity Nav2 passes in is not used, because the controller server flattens odometry to 2D and drops heave. As in Orca4, acceleration is limited relative to the previous command instead.
- Nav2 speed limits are ignored, with a warning the first time one arrives. Nav2 sends them from costmap filters, which the ROV does not use.
- Following a path that was never set, or setting an empty one, fails with `InvalidPath`. A pose that cannot be transformed into the path's frame fails with `ControllerTFError`.
