# Raspberry Pi 5 setup

Start with Raspberry Pi OS Lite (64-bit), flashed using Raspberry Pi Imager.
Configure a username, network access and SSH in Imager, then boot the Pi and log in.
This setup runs the dummy publishers using ROS 2 Jazzy in Docker.

## Install dependencies

```bash
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y git docker.io
sudo systemctl enable --now docker
```

Reboot with `sudo reboot`, then log back in. The helper script uses `sudo`
for Docker commands and will prompt for your password when needed.

## Build and run

Clone this repository onto the Pi, then enter its `pi` directory:

```bash
cd ~/MATE-ROV/pi
./ros_env.sh install
./ros_env.sh build
./ros_env.sh shell
ros2 launch rov_bringup all_hardware.launch.py
```

Adjust the `cd` path if you cloned elsewhere. The install command downloads
ROS 2 Jazzy and its build dependencies; the build command compiles all four
packages. The shell command opens a terminal with ROS and the workspace loaded.

For future runs, only the last two commands are needed. Stop the publishers
with Ctrl-C and leave the container shell with `exit`.

## Shared publishing frequency

The shared file is `rov_bringup/config/publish_rates.yaml`:

```yaml
/**:
  ros__parameters:
    publish_frequency_hz: 1.0
```

Change `1.0` to `10.0` to publish every dummy topic at 10 Hz on the next launch.
The supplied build uses `--symlink-install`, so config edits are available on
the next launch without rebuilding.

Append a node-specific entry after the wildcard to override just that node:

```yaml
/bno055_node:
  ros__parameters:
    publish_frequency_hz: 20.0
```

The wildcard remains the default for other nodes. Use floating-point YAML values
(e.g. `20.0`). Non-finite or non-positive double values trigger a warning and
fall back to 1.0 Hz; the exposed parameter also becomes 1.0. Parameters are read
at startup: restart the launch after changing rates.

Select a different file with:

```bash
ros2 launch rov_bringup all_hardware.launch.py \
  params_file:=/absolute/path/to/custom_rates.yaml
```

In Docker, the file must be accessible inside the container: files under `pi`
are mounted at `/workspace`. Individual package launches also accept
`params_file`, which must be supplied when launching them directly:

```bash
ros2 launch rov_sensors sensors.launch.py \
  params_file:=/workspace/rov_bringup/config/publish_rates.yaml
```

## Verify configured and measured frequencies

Keep the launch running. Open another terminal on the Pi and run:

```bash
cd ~/MATE-ROV/pi
./ros_env.sh shell
```

Inspect each node’s configured rate:

```bash
ros2 param get /bno055_node publish_frequency_hz
ros2 param get /bar30_node publish_frequency_hz
ros2 param get /leak_sensor_node publish_frequency_hz
ros2 param get /ping2_node publish_frequency_hz
ros2 param get /ping360_node publish_frequency_hz
ros2 param get /front_camera_node publish_frequency_hz
ros2 param get /down_camera_node publish_frequency_hz
```

Measure actual topic frequency with `ros2 topic hz`. Run each command for several
seconds and stop it with Ctrl-C. The reported average should approximately match
the corresponding parameter; scheduling and subscriber load can affect it.

```bash
ros2 topic hz /sensors/imu/data
ros2 topic hz /sensors/pressure
ros2 topic hz /sensors/leak
ros2 topic hz /sonar/ping2/range
ros2 topic hz /sonar/ping360/scan
ros2 topic hz /cameras/front/image_raw
ros2 topic hz /cameras/down/image_raw
```

Setup follows the [ROS 2 Raspberry Pi guidance](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Installation/Installing-on-Raspberry-Pi.rst): 64-bit Raspberry Pi OS with ROS in Docker.
