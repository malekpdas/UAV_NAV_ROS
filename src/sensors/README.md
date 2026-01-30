# sensors

This package contains drivers for various UAV sensors.

## Nodes

### IMU Drivers
- [imu_bmx160](Descriptor/imu_bmx160.yaml): Bosch BMX160 9-axis IMU driver.
- [imu_bno085](Descriptor/imu_bno085.yaml): Hillcrest BNO085 9-axis IMU driver.

### Positioning
- [gps_zoe_m8q](Descriptor/gps_zoe_m8q.yaml): U-blox ZOE-M8Q GPS driver using UBX protocol.

### Altimeter
- [lidar_lite_v3hp](Descriptor/lidar_lite_v3hp.yaml): Garmin Lidar Lite v3HP driver for range measurements.

## Configuration

Standard configuration files are located in the `config/` directory.

## Launch

Various launch files are provided in the `launch/` directory for individual or combined sensor startup.
