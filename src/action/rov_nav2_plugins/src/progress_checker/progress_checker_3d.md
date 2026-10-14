# ProgressChecker3D

**Class:** `rov_nav2_plugins::ProgressChecker3D`

**Type:** Progress checker (`nav2_core::ProgressChecker`)

**XML:** `plugins.xml`

Fails navigation when the vehicle has neither moved `required_movement_radius` in 3D nor turned `required_movement_angle` within `movement_time_allowance`. Each time it moves or turns that far, timing starts again from its new pose. Nav2's own progress checkers measure movement in 2D, so they would count a dive as no progress.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `required_movement_radius` | double | `0.5` | Distance to move in 3D (meters). |
| `required_movement_angle` | double | `0.5` | Or angle to turn (radians). |
| `movement_time_allowance` | double | `10.0` | Time allowed to move or turn that much (seconds). |

## Notes

- A goal the vehicle cannot reach, such as one above the surface, fails about `movement_time_allowance` after the vehicle stalls.
- Time is measured on the controller server's clock, so it follows simulated time when `use_sim_time` is set.
