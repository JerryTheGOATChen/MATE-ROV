# The Haunted House Hackathon

A ghost has **two minutes** to haunt as many people and eat as much candy as it can in a haunted house.

- **Candy is worth 1 point.**
- **People are worth 10 points.**
- I have no clue if this is a balanced point distribution (I hope it is...)
- To collect candy, get the ghost's center within **0.5 m** of it.
- To haunt a person, get the ghost's center within **0.5m** of thier head. 
- The walls are lined with salt, and ghosts can't cross salt.

Your ghost flies like our ROV: in 3D, with velocity commands in its own frame and a short lag before it responds. Like the ROV, **nobody tells it where it is, or where anything else is.** It has a compass, a velocity sensor, a scanning sonar and a depth camera, and it knows where it started. You are responsible for the following:

1. **Localization:** where am I?
2. **Tracking:** where are the people and candy I've seen?
3. **Target picking:** what to go after next
4. **Path planner:** a route there that stays out of the walls
5. **Motion controller:** velocity commands that fly the route
6. **Anything fun:**

## Setup (once)

You need Docker, git and [Foxglove](https://foxglove.dev/download). On Windows, use WSL2 with Docker Desktop 

```bash
git clone https://github.com/JerryTheGOATChen/MATE-ROV.git && cd MATE-ROV
git checkout hackathon/halloween
git checkout -b team/<your-team-name>
echo 'export ACTIVE_MODULES="action:dev"' > watod-config.local.sh
./watod build
```

## Every session

```bash
./watod up                       # starts your DevContainer, plus Foxglove bridge and friends
./watod -t action_bringup_dev    # a shell in the DevContainer (open as many as you like)
```

Inside the DevContainer, in `/ws`:

```bash
colcon build --symlink-install
source install/setup.bash

# Shell 1: the haunted house. Leave it running
ros2 launch haunted_house haunted_house.launch.yaml

# Shell 2: your ghost. Ctrl+C, rebuild and relaunch it as often as you like
ros2 launch ghost_brain ghost_brain.launch.yaml

# Start a fresh game whenever you want, also available in the Foxglove UI
ros2 service call /haunted_house/reset std_srvs/srv/Trigger
```

Your code in `src/action/` on your machine is mounted into the container, so edit with any editor you like. Run git outside the container.

### Watching in Foxglove

Open a foxglove connection to the bridge address that `./watod up` prints, for example `ws://localhost:20002`. 

The quickest setup is the ready-made layout: **Layouts → Import from file** → [`foxglove_layout.json`](foxglove_layout.json) in the repo root. It has:
- a 3D view and a top-down map, with the truth, the sonar, and your brain's pose and tracked targets;
- the camera's depth image with its detection boxes;
- a New game button, which calls the reset service;
- the game clock and score, plus the house's log of every catch;
- a session leaderboard of your best finished games. Games reset before time is up don't count, and the leaderboard empties when you reload the layout.

To set it up by hand instead, add a 3D panel, set its display frame to `map`, and turn on these topics:

| Topic | What it is |
|-------|------------|
| `/spectator/markers` | The house, and where the people, candy and ghost really are |
| `/sonar/scan` | The sonar, drawn where your brain thinks the ghost is |
| `/ghost_brain/path` | Your planned route |
| `/ghost_brain/markers` | What your tracker knows, and anything else your brain draws |
| `/map` | The occupancy grid your planner reads |

The sonar and camera are drawn at the pose your brain publishes. When the sonar's echoes line up with the walls, your localization works. An Image panel on `/camera/depth`, with `/camera/detections_overlay` turned on, shows what the camera sees.

A Raw Messages panel on `/haunted_house/state` shows the score and the clock.

## What your ghost can know

| Topic | Type | What's in it |
|-------|------|--------------|
| `/haunted_house/start_pose` | `geometry_msgs/PoseStamped` | Where the ghost starts every game |
| `/ghost/velocity` | `geometry_msgs/TwistStamped` | Velocity in the ghost's own frame, and turn rate, at 25 Hz with simulated noise and bias|
| `/ghost/imu` | `sensor_msgs/Imu` | Heading (yaw) in `orientation`, and turn rate, at 25 Hz |
| `/sonar/scan` | `sensor_msgs/LaserScan` | A horizontal beam sweeping a full circle once a second, sent a tenth of a turn at a time. Each range is the distance to the nearest thing, people included, in that direction |
| `/camera/depth` | `sensor_msgs/Image` | 128 × 72 depth image (meters, 32-bit floats) across the 120° in front, all at once, at 10 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | The camera's focal length and center |
| `/camera/detections` | `vision_msgs/Detection2DArray` | A box around each person's head and each candy the camera can see, and which it is. Does not tell you where in the world it is, or which candy/person it is. this will need to be done by you |
| `/map` | `nav_msgs/OccupancyGrid` | The floor plan from above at 0.1 m per cell: 100 where anything stands (including things that can be flown over or under), and 0 where it's clear |
| `/haunted_house/obstacles` | `haunted_house_msgs/BoxArray` | Every wall and piece of furniture as a 3D box |
| `/haunted_house/state` | `haunted_house_msgs/GameState` | Phase (waiting, running or finished), time left, score, counts, bonks |

The sonar is at the ghost's center. The camera is 0.25 m forward and 0.1 m up, which is also in TF as `base_link -> camera_optical`. Both publish the transform from `base_link`, so publish your own `map -> base_link` (the starter does) and Foxglove draws them where your brain thinks the ghost is.

**Off limits:** `/spectator/*` has the truth: the ghost's real pose, and where the people and candy are. Use it to check your work in Foxglove, but your brain must not subscribe to it.

### Driving the ghost

Publish `geometry_msgs/TwistStamped` on `/cmd_vel` with `header.frame_id: base_link`:

| Field | Meaning | Limit |
|-------|---------|-------|
| `twist.linear.x` | Forward | 1.0 m/s |
| `twist.linear.y` | Left (sideways) | 0.6 m/s |
| `twist.linear.z` | Up | 0.6 m/s |
| `twist.angular.z` | Turn left | 1.5 rad/s |

The ghost takes about 0.3 s to reach a commanded velocity. It stops if commands stop for 0.5 s. It's frozen during the countdown and after the final whistle.

## The house

16 × 12 m and 3 m tall, with the floor at z = 0. A grand hall runs down the middle, with a library and kitchen to the west and a ballroom and nursery to the east, all joined by 1.2 m doors. The ghost starts at the south end of the grand hall, facing north. The ghost is a 0.3 m sphere, so it fits through the doors but not by much.

Some furniture can be flown **over** (the dining table, the coffin, the piano) or **under** (the chandeliers).

The layout, the people's loops, the candy and the rules are all in [`haunted_house.yaml`](src/action/haunted_house/config/haunted_house.yaml). The final judging runs a different house file that nobody has seen, so a brain hard-coded for this house will do badly.

## Where to write code

Everything is in [`src/action/ghost_brain`](src/action/ghost_brain):

| File | What's there |
|------|--------------|
| `src/localizer.cpp` | `Localizer`: **yours to write.** Only the heading is filled in |
| `src/tracker.cpp` | `Tracker::on_detections()`: **empty, yours to write** |
| `src/planner.cpp` | `Planner::plan()`: **empty, yours to write** |
| `src/controller.cpp` | `Controller::compute_velocity()`: **empty, yours to write** |
| `src/ghost_brain_node.cpp` | The node: subscriptions, the control loop, `choose_target()`, and markers to draw |
| `test/test_sensing.cpp`, `test/test_planning.cpp` | Tests. Hidden `[.hackathon]` tests check each of the four |

The header comments in each `.hpp` list ideas, from simplest to fanciest. A good order: dead reckoning, then placing one detection, then flying to it. Then get smarter.

Test as you go:

```bash
colcon build --packages-select ghost_brain && colcon test --packages-select ghost_brain && colcon test-result --verbose
./build/ghost_brain/test_sensing "[hackathon]"    # the hidden localizer and tracker tests
./build/ghost_brain/test_planning "[hackathon]"   # the hidden planner and controller tests
```

You can rewrite anything in `ghost_brain`: add files, classes or parameters, even a second node. Three rules: don't edit `haunted_house`, drive the ghost only through `/cmd_vel`, and don't read `/spectator/*`.

## Ideas once it works

- **Localization:** dead reckoning drifts about a meter a minute. Correct it with the sonar against `/map`, or with a particle filter.
- **Exploring:** with nothing in view, spin to look around, or head for rooms you haven't seen yet.
- **Prediction:** estimate each person's velocity from a few sightings and intercept them.
- **3D:** use `/haunted_house/obstacles` and fly over the table instead of around it.
- **Style:** turn off the default ghost with `draw_ghost:=false` on the house launch, then draw your own on `/ghost_brain/markers` in the `base_link` frame so it follows your ghost. Add a trail, glowing eyes, a "BOO!" text marker, or a spin when you score.

## Judging

Each team's brain runs once on the secret house. The judge starts the brain first, then launches the house with the same `seed:=<n>` for everyone, so every team's first game has the same candy and the same people's starting points. The highest score wins. A tie goes to the fewest bonks.

