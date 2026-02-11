# Parameter Reference

Complete tuning guide for all 9 nodes in the UAV Navigation system.

## Sensor Fusion Node (`sensor_fusion`)

Main EKF state estimator combining IMU, magnetometer, and GPS.

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `frames.map_frame` | string | "map" | N/A | Fixed coordinate frame (global reference) |
| `frames.odom_frame` | string | "odom" | N/A | Moving reference frame (odometry estimates) |
| `frames.base_link_frame` | string | "base_link" | N/A | Vehicle body frame |
| `sensor_topics.imu_topic` | string | "imu_bno085/data" | topic | IMU data source |
| `sensor_topics.mag_topic` | string | "imu_bno085/mag" | topic | Magnetometer data source |
| `sensor_topics.gps_pos_topic` | string | "gps_zoe_m8q/fix" | topic | GPS position source |
| `sensor_topics.gps_vel_topic` | string | "gps_zoe_m8q/vel" | topic | GPS velocity source |
| `ahrs.offset_samples` | int | 100 | samples | Num samples for initial bias estimation (50-500) |
| `ahrs.gain` | float | 0.05 | 0.001-0.1 | Complementary filter gain (lower = smoother, slower response) |
| `ahrs.gyro_range` | float | 500.0 | dps | Gyroscope measurement range (degrees/sec) |
| `ahrs.accel_rejection` | float | 10.0 | g | Ignore accel readings > this value (shock detection) |
| `ahrs.mag_rejection` | float | 10.0 | µT | Ignore mag readings > this value (interference detection) |
| `kalman.initial_pos_uncertainty` | float | 25.0 | m² | Initial covariance for position (higher = less trust sensors) |
| `kalman.initial_vel_uncertainty` | float | 1.0 | m²/s² | Initial covariance for velocity |
| `kalman.initial_bias_uncertainty` | float | 0.01 | unitless | Initial covariance for gyro bias |
| `kalman.process_noise.pos` | float | 0.1 | N/A | Process noise in position (models world uncertainty) |
| `kalman.process_noise.vel` | float | 0.01 | N/A | Process noise in velocity |
| `kalman.process_noise.bias` | float | 0.001 | N/A | Process noise in gyro bias |
| `kalman.measurement_noise.pos` | float | 25.0 | N/A | GPS position measurement noise (matches `pos_std²`) |
| `kalman.measurement_noise.vel` | float | 0.25 | N/A | GPS velocity measurement noise |
| `earth.gravity` | float | 9.8066 | m/s² | Gravity constant (varies by latitude) |
| `earth.radius` | float | 6378137.0 | m | Earth radius (for lat/lon to meters) |
| `earth.ref_pos[0]` | float | 34.8761905 | degrees | Reference latitude (where UAV operates) |
| `earth.ref_pos[1]` | float | 136.9617842 | degrees | Reference longitude |
| `earth.ref_pos[2]` | float | 19.63 | m | Reference altitude above sea level |

### Tuning Guidance

**Drifty odometry?**
- Decrease `initial_pos_uncertainty` (trust sensors more)
- OR increase `process_noise.pos` (expect more world noise)
- OR tighten GPS variance (reduce `pos_std` in GPS config)

**Jerky/noisy odometry?**
- Increase `initial_pos_uncertainty` (trust sensors less)
- OR decrease `process_noise.pos` (expect less world noise)

**Accel spikes throwing off attitude?**
- Increase `ahrs.accel_rejection` (5.0 → 15.0 typical)

**Magnetometer interference/wrong compass?**
- Increase `ahrs.mag_rejection`
- OR verify `mag_decl` matches location
- OR check soft-iron calibration (`mag_transform`)

---

## IMU BNO085 Node (`imu_bno085`)

9-DOF Industrial Grade IMU with integrated sensor fusion processor.

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `rate_hz` | int | 100 | Hz | Publishing frequency (50-250 typical) |
| `frame_id` | string | "imu_link" | N/A | ROS2 frame name for this IMU |
| `sensor_calibration.bias_removal` | bool | true | N/A | Remove gyro bias before publishing |
| `sensor_calibration.accel_bias[0,1,2]` | float | -1.145, -0.014, 0.054 | m/s² | X/Y/Z accel offset (factory calibrated) |
| `sensor_calibration.gyro_bias[0,1,2]` | float | -0.0016, 0.00089, 0.00043 | rad/s | X/Y/Z gyro offset (factory calibrated) |
| `sensor_calibration.mag_bias[0,1,2]` | float | 0.123, -0.543, 0.099 | µT | X/Y/Z mag offset (from soft-iron cal) |
| `sensor_calibration.mag_transform[3x3]` | matrix | see below | N/A | Soft-iron correction matrix (calibration) |
| `sensor_variance.accel[0,1,2]` | float | 0.01 | (m/s²)² | Accel measurement noise covariance |
| `sensor_variance.gyro[0,1,2]` | float | 0.001 | (rad/s)² | Gyro measurement noise covariance |
| `sensor_variance.mag[0,1,2]` | float | 0.001 | µT² | Mag measurement noise covariance |
| `transformation.imu_rotation[3x3]` | matrix | Identity | N/A | Mounting orientation rotation matrix |
| `transformation.mag_decl` | float | -8.03 | degrees | Magnetic declination at UAV location |

### Default Soft-Iron Correction Matrix

```
[1.004,  0.012, -0.005]
[0.008,  1.003,  0.011]
[-0.003, 0.009,  0.999]
```

### Magnetic Declination by Region

| Region | Range | Example |
|--------|-------|---------|
| Japan (Project default) | -7° to -9° | -8.03° |
| USA (East) | -5° to -8° | -6.5° |
| USA (Central) | -8° to -12° | -10° |
| USA (West) | -8° to -16° | -12° |
| Europe | -2° to 5° | 0° |
| Australia | 8° to 12° | 10° |

**Find yours:** https://www.magnetic-declination.com/

### Mounting Orientation (imu_rotation)

If IMU is mounted upside-down, tilted, or rotated:

**Identity (no rotation):**
```
[1, 0, 0]
[0, 1, 0]
[0, 0, 1]
```

**Upside-down (180° roll):**
```
[1,  0,  0]
[0, -1,  0]
[0,  0, -1]
```

**90° yaw rotation (CCW from above):**
```
[0, -1, 0]
[1,  0, 0]
[0,  0, 1]
```

---

## GPS ZOE M8Q Node (`gps_zoe_m8q`)

Ublox multi-constellation GNSS receiver with RTK capability.

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `interface_type` | string | "uart" | uart / i2c | Communication protocol |
| `rate_hz` | int | 10 | Hz | Position solution rate (1-10 typical) |
| `frame_id` | string | "gps_link" | N/A | ROS2 frame name for GPS antenna |
| `sensor_variance.pos_std[0,1,2]` | float | [4.0, 4.0, 10.0] | m | Position std dev in East, North, Up |
| `sensor_variance.vel_std` | float | 0.5 | m/s | Velocity std dev (all axes) |

### Position Variance Guidance

| Environment | pos_std | vel_std | Notes |
|-------------|---------|---------|-------|
| Clear sky, good signal | [2.0, 2.0, 5.0] | 0.2 | Open field, minimal multipath |
| Urban canyon, trees | [6.0, 6.0, 12.0] | 0.8 | More reflections, bouncing signals |
| Indoors/poor signal | [10.0, 10.0, 20.0] | 1.5 | If it locks at all (rare indoors) |
| RTK fixed solution | [0.1, 0.1, 0.2] | 0.1 | With RTK base station active |

**Tuning Tip:** Match `pos_std` to your environment's actual GPS accuracy. Use field testing to validate.

---

## RC Control Node (`rc_control`)

GPIO-based RC receiver interface (typically Raspberry Pi).

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `gpiochip` | int | 4 | N/A | GPIO chip number (4 for Pi, varies by platform) |
| `publish_rate` | int | 50 | Hz | How often to publish RC channel values |
| `failsafe_timeout` | float | 1.0 | sec | If no signal > this, trigger failsafe |
| `gpio_pins` | int[] | [20,25,16,12,24,21] | GPIO # | Pins for RC channels 1-6 (BCM numbering) |
| `failsafe_values` | int[] | [1500,1500,1000,1500,1500,1000] | µs | PWM values to use if signal lost |

### GPIO Pin Mapping (BCM numbers for Raspberry Pi)

| Pin | Function | Typical Use |
|-----|----------|-------------|
| 20 | RC Channel 1 | Throttle |
| 25 | RC Channel 2 | Aileron (Roll) |
| 16 | RC Channel 3 | Elevator (Pitch) |
| 12 | RC Channel 4 | Rudder (Yaw) |
| 24 | RC Channel 5 | Mode Switch |
| 21 | RC Channel 6 | Auxiliary |

### Typical PWM Values

| Channel | Min (µs) | Mid (µs) | Max (µs) | Function |
|---------|----------|----------|----------|----------|
| 1 | 1000 | 1500 | 2000 | Throttle |
| 2 | 1000 | 1500 | 2000 | Aileron |
| 3 | 1000 | 1500 | 2000 | Elevator |
| 4 | 1000 | 1500 | 2000 | Rudder |
| 5 | 1000 | 1500 | 2000 | Mode |

### Failsafe Strategy

Default failsafe cuts throttle (1000 µs = minimum) ensuring immediate motor stoppage if RC link lost.

```json
{
  "failsafe_timeout": 1.0,
  "failsafe_values": [1500, 1500, 1000, 1500, 1500, 1000]
}
```

---

## Servo Control Node (`servo_control`)

PWM output for servos, ESC, and control surfaces.

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `gpiochip` | int | 4 | N/A | GPIO chip number |
| `pwm_freq` | int | 50 | Hz | PWM frequency for servos/ESC (50 Hz standard) |
| **GPIO Pins:** | | | | |
| `l_aileron` | int | 6 | GPIO # | Left aileron servo |
| `elevator` | int | 19 | GPIO # | Elevator servo |
| `esc_rotor` | int | 5 | GPIO # | ESC/motor control |
| `rudder` | int | 13 | GPIO # | Rudder servo |
| `r_aileron` | int | 26 | GPIO # | Right aileron servo |
| **RC Channel Mapping:** | | | | |
| `rc_ch_map.l_aileron` | int | 0 | channel # | Map RC ch to left aileron |
| `rc_ch_map.elevator` | int | 1 | channel # | Map RC ch to elevator |
| `rc_ch_map.rotor` | int | 2 | channel # | Map RC ch to rotor (throttle) |
| `rc_ch_map.rudder` | int | 3 | channel # | Map RC ch to rudder |
| `rc_ch_map.r_aileron` | int | 4 | channel # | Map RC ch to right aileron |
| **Pulse Limits:** | | | | |
| `safety.arming_duration` | float | 1.0 | sec | How long hold arming switch |
| `safety.esc_min_pulse` | float | 1000.0 | µs | ESC minimum PWM (motor off) |
| `safety.esc_max_pulse` | float | 2000.0 | µs | ESC maximum PWM (full throttle) |
| `safety.servo_min_pulse` | float | 1000.0 | µs | Servo min PWM (full neg deflection) |
| `safety.servo_max_pulse` | float | 2000.0 | µs | Servo max PWM (full pos deflection) |

### Typical RC-to-Servo Mapping

```
RC Channel 1 (1000-2000 µs) → Left Aileron (-45° to +45°)
RC Channel 2 (1000-2000 µs) → Elevator (-45° to +45°)
RC Channel 3 (1000-2000 µs) → Rotor/ESC (0-100% throttle)
RC Channel 4 (1000-2000 µs) → Rudder (-45° to +45°)
RC Channel 5 (1000-2000 µs) → Right Aileron (-45° to +45°)
```

---

## IMU BMX160 Node (`imu_bmx160`) - Alternative Sensor

Same parameter structure as BNO085, with different calibration values:

| Parameter | Default (BMX160) | Units |
|-----------|------------------|-------|
| `rate_hz` | 100 | Hz |
| `sensor_calibration.accel_bias` | [0, 0, 0] | m/s² |
| `sensor_calibration.gyro_bias` | [0, 0, 0] | rad/s |
| `sensor_calibration.mag_bias` | [0, 0, 0] | µT |
| `transformation.mag_decl` | -8.03 | degrees |

---

## Lidar Lite v3HP Node (`lidar_lite_v3hp`)

Optical range sensor for altitude/obstacle detection.

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `frequency` | int | 100 | Hz | Measurement rate |
| `frame_id` | string | "lidar_link" | N/A | ROS2 frame |
| `sensor_config.min_range` | float | 0.05 | m | Minimum measurable distance |
| `sensor_config.max_range` | float | 40.0 | m | Maximum measurable distance |
| `sensor_config.preset` | string | "balanced" | balanced/long_range/high_accuracy/high_speed | Operating mode |
| `sensor_config.sig_count_val` | int | -1 | -1/auto or 2-200 | Signal samples (-1 = use preset) |
| `sensor_config.acq_config_reg` | int | -1 | register value | Acquisition config (-1 = use preset) |
| `sensor_config.threshold_bypass` | int | -1 | -1/0/1 | Threshold bypass mode (-1 = use preset) |
| `sensor_config.ref_count_val` | int | -1 | register value | Reference count (-1 = use preset) |

### Preset Modes

| Mode | Use Case | Range | Accuracy |
|------|----------|-------|----------|
| balanced | Default, all-purpose | 0-40m | ±6cm |
| long_range | Measure tall buildings | 0-60m | ±10cm |
| high_accuracy | Precise altitude | 0-20m | ±3cm |
| high_speed | Fast updates | 0-40m | ±6cm |

---

## Calibration Node (`mag_calibration`)

Magnetometer soft-iron calibration utility.

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `max_samples` | int | 50000 | samples | Max data points to collect |
| `min_samples` | int | 400 | samples | Min before trying calibration |
| `angular_bins` | int | 6 | bins | Spatial resolution (6 = 60° sectors) |
| `gap_threshold_percent` | float | 1.0 | % | Max gap in heading coverage |
| `plot_every_n` | int | 50 | samples | Update visualization every N points |

**Usage:** Run this node separately during ground calibration (3D rotation test with UAV on level ground). Generates corrected `mag_transform` matrix.

---

## Plot Orientation Node (`plot_orientation`)

Real-time odometry visualization (optional, not required for flight).

| Parameter | Type | Default | Units |
|-----------|------|---------|-------|
| `odom_topic` | string | "/odom" | topic |
| `update_period` | float | 0.03 | sec |
| `box_dimensions.length` | float | 4.0 | m |
| `box_dimensions.width` | float | 2.0 | m |
| `box_dimensions.height` | float | 0.8 | m |
| `colors.face_colors` | string[] | ['#444444', '#00FF00', ...] | hex |
| `colors.text_color` | string | '#00FF00' | hex |
| `colors.edge_color` | string | 'white' | color |

---

**Next:** [Integration Examples](integration-examples.md) or [Appendix](appendix.md)