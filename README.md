# UAV_NAV_ROS

This project provides a ROS2-based navigation stack for a UAV, including sensor drivers, an EKF estimator, and control nodes.

## Project Structure

- `src/sensors`: Drivers for BMX160 IMU, BNO085 IMU, ZOE-M8Q GPS, and Lidar Lite v3HP.
- `src/ekf_estimator`: AHRS and EKF-based state estimator (Linear Kalman Filter).
- `src/rc_control_ros`: Node for reading RC receiver signals via GPIO.
- `src/servo_control_ros`: Node for controlling servos and ESCs via GPIO/PWM.
- `src/calibration`: Tools for magnetometer calibration.
- `src/extra_nodes`: Logging and visualization utilities.
- `src/launch`: Global launch files (if any).

## Packages

- [sensors](src/sensors/README.md)
- [ekf_estimator](src/ekf_estimator/README.md)
- [rc_control_ros](src/rc_control_ros/README.md)
- [servo_control_ros](src/servo_control_ros/README.md)
- [calibration](src/calibration/README.md)
- [extra_nodes](src/extra_nodes/README.md)

## Getting Started

### Prerequisites

- ROS2 (Humble or later recommended)
- `lgpio` library (for RPi GPIO access)
- `imufusion` (for AHRS)
- `scipy`, `numpy`, `matplotlib`

### Building

```bash
colcon build --symlink-install
source install/setup.bash
```

### Running

Each package has its own launch files and nodes. See the package-specific READMEs for more details.
