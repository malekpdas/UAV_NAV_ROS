# ekf_estimator

This package provides state estimation for the UAV using a Linear Kalman Filter (EKF) and AHRS fusion.

## Nodes

### [ekf_node](Descriptor/ekf_node.yaml)

Fuses IMU, Magnetometer, and GPS data to estimate the UAV's attitude, velocity, and position in the NED frame.

## Architecture

- **AHRS**: Uses `imufusion` to estimate orientation from gyro, accel, and mag.
- **EKF**: A Linear Kalman Filter for position and velocity estimation, with acceleration bias compensation.

## Configuration

- `config/ekf_params.yaml`: Filter gains, process noise, and topic names.

## Launch

- `launch/ekf_estimator.launch.py`: Launches the fusion node with parameters.
