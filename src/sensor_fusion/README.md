# sensor_fusion

This package provides state estimation for the UAV using a Linear Kalman Filter (EKF) and AHRS fusion.

## Nodes

### [sensor_fusion](descriptor/sensor_fusion.yaml)

Fuses IMU, Magnetometer, and GPS data to estimate the UAV's attitude, velocity, and position in the NED frame.

## Architecture

- **AHRS**: Uses `imufusion` to estimate orientation from gyro, accel, and mag.
- **EKF**: A Linear Kalman Filter for position and velocity estimation, with acceleration bias compensation.

## Configuration

- `config/sensor_fusion_config.yaml`: Filter gains, process noise, and topic names.

## Launch

- `launch/sensor_fusion.launch.py`: Launches the fusion node with parameters.
- `launch/sensor_fusion.launch.yaml`: Launches the fusion node with parameters.
