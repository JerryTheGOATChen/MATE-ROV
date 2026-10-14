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
