# altimeter_driver

Driver for the Blue Robotics Ping2 altimeter. `ping2_node` is a lifecycle node that publishes the distance from the transducer to the floor, using the [bluerobotics-ping](https://pypi.org/project/bluerobotics-ping/) library.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `altitude` | `sensor_msgs/Range` | Distance along the beam to the floor in metres |

A missing or low-confidence reading publishes nothing, so a stale topic means the altitude is unknown.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyUSB0` | Serial device the Ping2 is on |
| `baudrate` | int | `115200` | Serial baud rate |
| `rate_hz` | double | `10.0` | Ping and publish rate |
| `speed_of_sound` | double | `1500.0` | Speed of sound in the water in m/s |
| `min_confidence` | int | `80` | Readings the device rates below this percentage are dropped |
| `field_of_view` | double | `0.436` | Beam width in radians (the Ping2's 25 degrees) |
| `frame_id` | string | `ping2_link` | Frame of the transducer |
