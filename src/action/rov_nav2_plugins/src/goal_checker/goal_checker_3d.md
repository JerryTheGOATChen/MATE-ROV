# GoalChecker3D

**Class:** `rov_nav2_plugins::GoalChecker3D`

**Type:** Goal checker (`nav2_core::GoalChecker`)

**XML:** `plugins.xml`

Reports the goal reached when the horizontal distance, the vertical distance and the heading error to the goal are all within tolerance. Velocity is not checked.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `xy_goal_tolerance` | double | `0.25` | Horizontal tolerance (meters). |
| `z_goal_tolerance` | double | `0.25` | Vertical tolerance (meters). |
| `yaw_goal_tolerance` | double | `0.25` | Heading tolerance (radians). Use `3.15` or more to ignore heading. |

## Notes

- Position and heading must be within tolerance at the same time. Unlike Nav2's stateful `SimpleGoalChecker`, reaching the position is not remembered while the vehicle turns, because an ROV can drift off position while it turns.
- Nav2 reports success the moment this check passes, while the vehicle may still be moving at about the controller's `approach_gain` times the tolerance; it then coasts to a stop. Tighten the tolerances if the vehicle must stop closer to the goal.
