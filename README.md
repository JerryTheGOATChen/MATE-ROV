# WATonomous Monorepo Template

A starting point for WATonomous robotics projects. It contains the infrastructure from [wato_monorepo](https://github.com/WATonomous/wato_monorepo) without any robot-specific code. That includes the `watod` CLI, multi-stage Docker builds, docker compose profiles, the Zenoh router, Foxglove bridge, log viewer, unit testing with `wato_test`, pre-commit, and CI.

Every module builds, runs, and passes its tests out of the box. Each one contains a bringup package and a single example lifecycle node that you replace with real code.

## Modules

| Module | Container | Purpose | Contents |
|--------|-----------|---------|----------|
| `infrastructure` | `infrastructure_bringup` | Shared services, always started | Zenoh router, Foxglove bridge, TF relays, topic healthchecker, `wato_lifecycle_manager`, `vision_msgs_markers` |
| `interfacing` | `interfacing_bringup` | Hardware drivers, sensors, CAN, actuation | `pid_msgs` |
| `perception` | `perception_bringup` (CUDA) | Detection, tracking, sensor fusion | |
| `world_modeling` | `world_modeling_bringup` | Localization, mapping, prediction | |
| `action` | `action_bringup` | Planning and control | Nav2 with 3D plugins (`rov_nav2_plugins`) |

The **log viewer** web UI runs next to infrastructure. It shows container logs and topic health from `topic_healthchecker`.

## Quickstart

```bash
cp watod-config.sh watod-config.local.sh   # then set ACTIVE_MODULES
./watod build                              # build images
./watod up                                 # start ACTIVE_MODULES + infrastructure
./watod test                               # run colcon tests in each module
./watod -t interfacing_bringup             # open a shell in a container
./watod down
```

Append `:dev` to a module (for example, `ACTIVE_MODULES="perception:dev"`) to get a DevContainer with `src/<module>` mounted for live editing. See [DEVELOPING.md](DEVELOPING.md) for the full guide.

## Gazebo on macOS, Linux, and Windows with WSL2

The repository includes a shared Docker configuration for Gazebo Harmonic. This keeps Gazebo and
ROS 2 inside the project image, so group members do not need to install them directly on their
computers. It supports Docker Desktop on macOS, Docker Engine on native Linux, and Docker Desktop
with WSL2 integration on Windows. Native Windows PowerShell is not supported by the Bash-based
`watod` launcher.

1. Install and start Docker Desktop on macOS/Windows, or Docker Engine with the Compose plugin on
   Linux. On Windows, enable Docker Desktop's WSL2 integration and run these commands inside the
   WSL2 terminal. Keeping the repository in the WSL filesystem rather than `/mnt/c` is recommended.
2. From the repository root, create your ignored local configuration from the tracked example:

   ```bash
   cp watod-config.gazebo.sh watod-config.local.sh
   ```

3. Build and start the headless simulator:

   ```bash
   ./watod build
   ./watod up -d
   ```

4. Check the services with `./watod ps`, or connect Foxglove to the address printed by `watod`.
   Stop everything with `./watod down`.

### Foxglove visualization

Connect Foxglove to `ws://localhost:<FOXGLOVE_BRIDGE_PORT>` (port `11002` with the example local
setup in this checkout). Then open the **Layouts** menu, choose **Import from file**, and select
[`foxglove/mate_rov_gazebo.json`](foxglove/mate_rov_gazebo.json).

The shared layout contains a `map`-frame 3D pool view with the pool walls and competition props,
the moving ROV and its trail, planned path, odometry, ground-truth pose, sonar
points and OctoMap occupied cells. It also includes plots comparing estimated and true depth, body
velocity, IMU turn rates, and a ground-truth message inspector. If a developer uses a different
`BASE_PORT`, use the Foxglove URL printed by `watod up` instead of port `11002`.

The Foxglove scene is a clean ROS-marker reconstruction of Gazebo's world, not a second physics
simulation. Gazebo remains responsible for motion, collisions and sensor generation. To record a
run as an MCAP file for later Foxglove playback, open a second terminal and run:

```bash
./watod bag record -a -o mate_rov_run
```

Stop recording with `Ctrl+C`. The recording is saved under `bags/mate_rov_run/` and can be opened
directly in Foxglove with the same layout.

The example deliberately contains no username, home-directory path, Docker socket path, or fixed
port. `watod` discovers those values for each developer. Personal changes belong in
`watod-config.local.sh`, which Git ignores. Linux users running the physical ROV instead of Gazebo
can continue using the normal `watod-config.sh` defaults.

The Gazebo configuration uses a `materov_gazebo_<user>` Compose project by default. Compose-managed
containers, networks, and volumes are therefore separate from older `watod_<user>` stacks on the
same Docker daemon. It also uses the separate 11000-11009 host-port range by default. Images are
tagged with the current Git branch as well, so building `cyrus` does not overwrite a groupmate's
`main` image. Run `./watod down` from the matching checkout and config to stop only that simulator
stack. Override `COMPOSE_PROJECT_NAME` or `BASE_PORT` in the ignored local config if those defaults
are already in use.

## Starting a new project from this template

1. Click **Use this template** on GitHub, or copy the repo.
2. You don't need to configure the registry or any secrets. Images publish to `ghcr.io/<owner>/<repo>`. CI takes that from the repo running the workflow, and local `watod` takes it from your git `origin` remote. CI logs in with the built-in `GITHUB_TOKEN`. To override the registry, set `REGISTRY_URL` in `watod-config.local.sh`.
3. Add your nodes as packages under `src/<module>/`. Then update `src/<module>/<module>_bringup/launch/<module>.launch.yaml` and list your lifecycle nodes in the lifecycle manager's `node_names` parameter.
4. Put the topics you care about in `src/infrastructure/infrastructure_bringup/config/topic_healthchecker.yaml`.
5. Add shared message packages as `src/<module>/<name>_msgs`. Copy them into `docker/infrastructure.Dockerfile` so Foxglove and `ros2 bag` can decode them, and into any other module Dockerfile that uses them.

### Adding a package

Create it under `src/<module>/`. The module Dockerfile already copies all of `src/<module>`. Declare dependencies in `package.xml`, and `rosdep` installs them during the image build. Only add manual installs to the `dependencies` stage of `docker/<module>.Dockerfile` when rosdep can't handle them.

### Adding a module

Copy an existing module end to end:
- `docker/<module>.Dockerfile`
- the `_source`, `_deps`, and `_bringup` services in `modules/docker-compose.yaml`
- matching entries in `docker-compose.{dep,dev,test,watcloud}.yaml`
- `<MODULE>_IMAGE` in `watod_scripts/watod-setup-env.sh`
- `get_all_modules()` in `watod`
- the module lists in `.github/templates/`

## Repository layout

```text
.github/          CI: build + unit test per changed module, pre-commit, base images
docker/           Module Dockerfiles, shared template.Dockerfile, base image injectors, container config
modules/          docker compose files (base, dep, dev, test, watcloud overlay)
src/              ROS 2 packages grouped by module
watod             docker compose wrapper CLI
watod_scripts/    watod helpers and the log viewer
```
