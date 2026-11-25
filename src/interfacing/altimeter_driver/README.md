# altimeter_driver

Driver for the Blue Robotics Ping2 altimeter. `ping2_node` is a C++ lifecycle node (also a component, `altimeter_driver::Ping2Node`) that publishes the distance from the transducer to the floor. It speaks the [Ping protocol](https://docs.bluerobotics.com/ping-protocol/) over the serial port itself (`ping2.hpp`), implementing only the messages it uses, so it has no dependencies beyond ROS.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `altitude` | `sensor_msgs/Range` | Distance along the beam to the floor in metres |

A missing or low-confidence reading publishes nothing, so a stale topic means the altitude is unknown.

Configuring the node writes `speed_of_sound` and a ping interval of `1000 / rate_hz` ms to the device, and fails unless the device reports both back.

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
