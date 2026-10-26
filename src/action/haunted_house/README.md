# haunted_house

The simulator and referee for the [Haunted House Hackathon](../../../HACKATHON.md). It runs the game, moves the ghost under `/cmd_vel`, walks the people along their hidden loops, and simulates the ghost's sensors. The truth goes on `/spectator/*` only.

```bash
ros2 launch haunted_house haunted_house.launch.yaml [house_file:=<file>] [seed:=<n>] [draw_ghost:=false]
```

## Interfaces

For the ghost's brain:

| Name | Type | Description |
|------|------|-------------|
| `cmd_vel` (subscribed) | `geometry_msgs/TwistStamped` | Velocity in `base_link`. Commands in other frames are ignored, and the ghost stops if they stop for 0.5 s |
| `ghost/velocity` | `geometry_msgs/TwistStamped` | Measured velocity in `base_link` and yaw rate, at 25 Hz: noise of 0.02 m/s, plus a bias of up to 0.015 m/s per axis drawn every game |
| `ghost/imu` | `sensor_msgs/Imu` | Measured heading in `orientation` (1° noise) and yaw rate, at 25 Hz |
| `sonar/scan` | `sensor_msgs/LaserScan` | 360 horizontal beams a second from the ghost's center, one revolution a second, 36 beams per message, in `sonar_link`. 0.02 m noise, 15 m range |
| `camera/depth` | `sensor_msgs/Image` | 128 × 72, 120° wide, `32FC1` depth along the optical axis, inf beyond 12 m, at 10 Hz, in `camera_optical` |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | Pinhole intrinsics, no distortion |
| `camera/detections` | `vision_msgs/Detection2DArray` | Pixel boxes around people's heads and candy in view and in line of sight, with `class_id` "person" or "candy". No positions or IDs |
| `camera/detections_overlay` | `visualization_msgs/ImageMarker` | The same boxes, for a Foxglove Image panel |
| `haunted_house/start_pose` | `geometry_msgs/PoseStamped` | The ghost's start. Latched |
| `map` | `nav_msgs/OccupancyGrid` | The floor plan from above. Latched |
| `haunted_house/obstacles` | `haunted_house_msgs/BoxArray` | Walls and furniture as 3D boxes. Latched |
| `haunted_house/state` | `haunted_house_msgs/GameState` | Phase, clock, score and counts, at 10 Hz |
| `haunted_house/reset` (service) | `std_srvs/Trigger` | Put everything back, shuffle, and count down to a new game |
| `base_link -> sonar_link`, `base_link -> camera_optical` | TF (static) | Where the sensors are mounted |

For spectators only:

| Name | Type | Description |
|------|------|-------------|
| `spectator/ghost_pose` | `geometry_msgs/PoseStamped` | Where the ghost really is |
| `spectator/people`, `spectator/candies` | `haunted_house_msgs/TargetArray` | Where every remaining target really is |
| `spectator/markers` | `visualization_msgs/MarkerArray` | Everything drawn for Foxglove, in `map` |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `house_file` | `config/haunted_house.yaml` | Layout, people, candies, ghost limits and rules |
| `seed` | -1 | Seed for the shuffles and sensor noise. Negative for a different game every reset |
| `update_rate_hz` | 50.0 | Simulation rate. The game clock advances by a fixed step per tick |
| `camera_rate_hz` | 10.0 | Depth image and detection rate |
| `motion_rate_hz` | 25.0 | Velocity and heading rate |
| `spectator_rate_hz` | 10.0 | Rate of the state and the spectator topics |
| `cmd_vel_timeout_s` | 0.5 | The ghost stops if commands stop for this long |
| `map_resolution_m` | 0.1 | Cell size of `map` |
| `publish_floor_plan` | true | Publish `map` and `haunted_house/obstacles`. Turn off for a SLAM challenge |
| `draw_ghost` | true | Draw the default ghost |

## How the sensors see

Every sensor ray-casts against boxes: the walls and furniture; the floor and ceiling; each person as a 0.4 m wide column up to just above their head; and each candy as a 0.2 m cube. A person or candy is detected when the point the ghost must reach (the head, or the candy's center) is in the image, within 12 m, and has a clear line of sight.

## House files

See [`config/haunted_house.yaml`](config/haunted_house.yaml) for the format. To make a new house for judging:
1. Copy that file and move things around.
2. Check it's fair: `HOUSE_FILE=<your copy> pytest test/test_house_file.py`. The test checks that people walk clear of everything, every candy can be reached even after shuffling, and a ghost that stays still scores nothing.

## Code

The game is plain Python and numpy, testable without ROS:
- `house.py`: geometry and collisions
- `ghost.py`: motion
- `people.py`: walkers
- `game.py`: rules, scoring and shuffling
- `sensors.py`: sonar, camera and motion sensors
- `config.py`: house files

`node.py` wraps them in ROS, and `markers.py` draws them.
