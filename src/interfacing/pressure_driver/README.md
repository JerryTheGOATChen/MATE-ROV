# pressure_driver

Driver for the Blue Robotics Bar30 pressure sensor (MS5837-30BA on I2C address `0x76`). `bar30_node` is a C++ lifecycle node (also a component, `pressure_driver::Bar30Node`) that reads the pressure over Linux I2C and publishes the ROV's depth.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `depth` | `nav_msgs/Odometry` | `pose.pose.position.z` is the negative depth in metres. Only z and its variance are filled in. |

A failed read publishes nothing, so a stale topic means the sensor is not answering.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_bus` | int | `1` | I2C bus number the sensor is wired to (`/dev/i2c-<n>`) |
| `rate_hz` | double | `10.0` | Publish rate. One reading takes about 40 ms, so the limit is about 20 Hz |
| `fluid_density` | double | `997.0` | Water density in kg/m^3. Use `1029.0` for sea water |
| `surface_pressure_pa` | double | `101325.0` | Air pressure at the surface in Pa. Depth is measured from this pressure |
| `depth_variance` | double | `0.0004` | Variance of the published z in m^2 |
| `frame_id` | string | `odom` | Frame the z position is expressed in |
| `child_frame_id` | string | `base_link` | The ROV's body frame |

## Credits

`ms5837.cpp` follows the MS5837-30BA datasheet, as do Blue Robotics' MIT-licensed [MS5837 libraries](https://github.com/bluerobotics/ms5837-python).
