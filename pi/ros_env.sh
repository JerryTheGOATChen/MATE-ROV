#!/usr/bin/env bash
set -euo pipefail
pi_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-shell}" in
  install)
    sudo docker build -t rov-ros:jazzy "$pi_dir"
    ;;
  build)
    sudo docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -v "$pi_dir:/workspace" rov-ros:jazzy \
      bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --base-paths rov_sensors rov_sonar rov_cameras rov_bringup'
    ;;
  shell)
    sudo docker run --rm -it --network host --ipc host \
      --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -v "$pi_dir:/workspace" rov-ros:jazzy \
      bash -c 'source /opt/ros/jazzy/setup.bash; if [ -f install/setup.bash ]; then source install/setup.bash; fi; exec bash --norc -i'
    ;;
  *) echo "Usage: $0 {install|build|shell}" >&2; exit 2 ;;
esac
