# ghost_brain

**The hackathon starter: this is your package.** See [HACKATHON.md](../../../HACKATHON.md) for the rules.

A lifecycle node. Sensor messages feed two classes:
- `Localizer` works out where the ghost is from the start pose, velocity, heading and sonar. **Only the heading is filled in**, so the ghost thinks it never leaves the start.
- `Tracker` works out where the people and candy are from the camera's detections and depth. **It's empty**, so the ghost never knows about anything.

A control loop at 20 Hz then:
1. `choose_target()` picks the nearest target `Tracker` knows about.
2. `Planner::plan()` plans a route there. It runs again for every new target and every `replan_period_s`. **It's empty.**
3. `Controller::compute_velocity()` turns the route into a velocity on `/cmd_vel`. **It's empty.**

At the start of every game, it resets `Localizer` to the start pose and clears `Tracker`.

```bash
ros2 launch ghost_brain ghost_brain.launch.yaml
```

The launch file also starts a `wato_lifecycle_manager`, which configures and activates the node, as every lifecycle node in the real stack is managed.

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| Sensors, floor plan, start pose and game state (subscribed) | | See [HACKATHON.md](../../../HACKATHON.md#what-your-ghost-can-know) |
| `cmd_vel` (published) | `geometry_msgs/TwistStamped` | Velocity in `base_link`. Zero whenever the game isn't running |
| `ghost_brain/pose` (published) | `geometry_msgs/PoseStamped` | `Localizer`'s pose |
| `map -> base_link` | TF | The same pose, so Foxglove draws the sensors where the brain thinks the ghost is |
| `ghost_brain/path` (published) | `nav_msgs/Path` | The latest route |
| `ghost_brain/markers` (published) | `visualization_msgs/MarkerArray` | Tracked targets (orange people, green candy) and a ring around the current one. Add your own |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_rate_hz` | 20.0 | Control loop rate |
| `replan_period_s` | 0.5 | How often to replan toward the same target |
| `map_frame`, `base_frame` | `map`, `base_link` | Frames of the pose, and of the velocity commands |

## Tests

```bash
colcon test --packages-select ghost_brain && colcon test-result --verbose
./build/ghost_brain/test_sensing "[hackathon]"
./build/ghost_brain/test_planning "[hackathon]"
```

Hidden `[.hackathon]` tests check the localizer, tracker, planner and controller. They fail until you write those. Once they pass, delete the dot in their tag so CI runs them too.
