# ROS 2 UAV State Estimation Stack

**I2C ONLY • PX4 EKF2 • WIND ESTIMATION • FLIGHT-CONTROL GRADE**

## Overview
This project implements a complete state estimation stack for a UAV using:
- **Bosch BMX160 IMU** (I2C)
- **u-blox ZOE-M8Q GPS** (I2C/DDC)
- **PX4-faithful EKF2** (Python)

## Packages
1. `imu_bmx160_driver`: Pure I2C driver for BMX160. Publishes `sensor_msgs/Imu`.
2. `gps_zoe_m8q_driver`: UBX/I2C driver for ZOE-M8Q. Publishes `sensor_msgs/NavSatFix` and Velocity.
3. `ekf2_estimator`: Core EKF logic with wind estimation and fault detection.

## Usage
### 1. Build
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch
```bash
ros2 launch uav_state_estimation.launch.py
```

## Parameters
See `config/params.yaml` in each package for:
- I2C Bus/Address
- Update Rates
- Sensor Noise Characteristics
