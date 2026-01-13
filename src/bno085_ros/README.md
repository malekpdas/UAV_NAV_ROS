# BNO085 ROS2 Driver

ROS2 driver for the Hillcrest BNO085 9-axis IMU with quaternion output from AR/VR Stabilized Rotation Vector.

## Features

- **Quaternion orientation** from AR/VR Stabilized Rotation Vector (report 0x28)
- **Calibrated sensors**: accelerometer, gyroscope, magnetometer
- **Dynamic Calibration Data (DCD)** save to flash
- Works without RST pin via extended soft reset timing

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Orientation, angular velocity, linear acceleration |
| `/imu/mag` | `sensor_msgs/MagneticField` | Calibrated magnetic field |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bus` | 1 | I2C bus number |
| `i2c_addr` | 0x4A | I2C address (0x4A or 0x4B) |
| `rate_hz` | 100.0 | Publishing rate (Hz) |
| `frame_id` | "imu_link" | TF frame ID |

## Usage

```bash
# Launch with default config
ros2 launch bno085_ros bno085.launch.py

# Or run node directly
ros2 run bno085_ros bno085_node
```

## Services

```bash
# Save calibration to flash
ros2 service call /bno085_node/save_calibration std_srvs/srv/Trigger

# Enable/disable dynamic calibration
ros2 service call /bno085_node/enable_calibration std_srvs/srv/SetBool "{data: true}"
```

## Calibration

The BNO085 calibrates automatically in the background. Monitor status via:

```bash
ros2 topic echo /imu/calibration_status
```

**Status values:** 0=Unreliable, 1=Low, 2=Medium, 3=High (fully calibrated). It is normal to start at 0.

### Calibration Procedure
To reach High Accuracy (3), perform the following motions:

1. **Gyroscope (Stationary)**:
   - Place sensor on a stable surface.
   - Leave completely motionless for 2-3 seconds.

2. **Accelerometer (Gravity)**:
   - Place sensor in 6 stationary positions for ~2s each:
     - Z-axis up (flat on table)
     - Z-axis down (upside down)
     - X-axis up/down
     - Y-axis up/down

3. **Magnetometer (North)**:
   - Rotate sensor in a large "Figure-8" pattern in the air.
   - Rotate around all 3 axes.

**Note:** Once accuracy reaches 2 or 3, run:
```bash
ros2 service call /bno085_node/save_calibration std_srvs/srv/Trigger
```

## Wiring

| BNO085 Pin | Connection |
|------------|------------|
| VIN/3V3 | 3.3V |
| GND | Ground |
| SDA | I2C SDA |
| SCL | I2C SCL |
| RST | (Optional) GPIO for hardware reset |
| INT | (Optional) Interrupt |

## License

MIT
