ARG BASE_IMAGE=ghcr.io/jerrythegoatchen/mate-rov/base:jazzy-ubuntu24.04
# Base images are built by this repo's "Build Monorepo Base Images" workflow
# (.github/workflows/build_base_images.yml), configured in .github/include/base_image_config.json.

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/infrastructure infrastructure
COPY src/wato_test wato_test

# Copy in every msgs package so Foxglove and ros2 bag can decode them
COPY src/interfacing/pid_msgs pid_msgs

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# Install module-specific dependencies (non-rosdep)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*
