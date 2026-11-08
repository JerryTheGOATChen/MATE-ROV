#!/usr/bin/env bash
# Shared Docker configuration for Gazebo on macOS, native Linux, and Windows with WSL2.
# Copy this file to watod-config.local.sh; keep personal overrides in that ignored file.

# The Gazebo launch already includes the action stack, so no hardware-facing modules are needed.
export ACTIVE_MODULES="action"
export ACTION_COMMAND="ros2 launch rov_gazebo_sim sim.launch.yaml gui:=false"

# Keep simulator containers separate from an older/default watod stack on the same Docker daemon.
# This matters on Docker Desktop for Windows, where Windows and WSL2 share the daemon. Developers
# can still override this name in their ignored watod-config.local.sh.
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-materov_gazebo_${USER:-user}}"

# Avoid the UID-derived ports used by a normal watod stack. Change this in the local config if
# another application already owns the 11000-11009 range.
export BASE_PORT="${BASE_PORT:-11000}"

# Software rendering avoids requiring a host GPU inside Docker and works across all three hosts.
export LIBGL_ALWAYS_SOFTWARE="1"

# Give the ROS services one shared network namespace. This is required by Docker Desktop and also
# works on native Linux, giving every supported host the same container topology.
export WATCLOUD_MODE="true"

# The project images are currently built for amd64. Apple Silicon and ARM Linux run them through
# Docker's emulation; Intel/AMD Linux and WSL2 use this platform natively.
export DOCKER_DEFAULT_PLATFORM="linux/amd64"

# Claude Code is unrelated to running the simulator and makes the image larger.
export CLAUDE_CODE="false"
