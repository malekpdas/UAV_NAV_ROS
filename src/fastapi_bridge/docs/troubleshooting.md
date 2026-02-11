# Troubleshooting & Common Errors

Solutions for common issues when working with the FastAPI Bridge API.

## HTTP 500 - "Node executable not found"

### Symptoms

- `/launch/start` returns 500 with error message about missing executable

### Causes

- ROS2 package not installed in workspace
- Executable not compiled (requires `colcon build`)
- Wrong executable name in request

### Solution

1. Verify package is in `src/` directory:
   ```bash
   ls src/sensors/
   ls src/sensor_fusion/
   ```
2. Rebuild workspace:
   ```bash
   colcon build --symlink-install
   ```
3. Verify executable name matches package (get from `/nodes/get_all_nodes`)
4. Check package is discoverable:
   ```bash
   ros2 pkg list | grep sensors
   ```

---

## HTTP 500 - "Config file malformed"

### Symptoms

- `/launch/start` fails or node crashes immediately after starting
- Error mentions YAML syntax

### Causes

- Config file has YAML syntax error (bad indentation, missing quotes)
- Custom config created via `/config/create` has invalid structure
- Parameter type mismatch (e.g., string provided where float expected)

### Solution

1. Validate YAML syntax:
   ```bash
   python -m yaml /path/to/config.yaml
   ```
   Should parse without error.

2. Check custom config at `<launch_dir>/config/<node_id>.yaml`

3. Compare structure to default config:
   ```bash
   cat src/sensors/config/imu_bno085.yaml
   ```

4. Delete custom config and restart to use defaults:
   ```bash
   rm ~/.ros/launch/config/<node_id>.yaml
   ```

---

## HTTP 404 - "No launch process running"

### Symptoms

- `/launch/stop` returns 404 when you haven't started a system yet

### Causes

- Never called `/launch/start`
- Already called `/launch/stop` once (can't stop twice)
- System crashed and process died naturally

### Solution

- Only call `/launch/stop` after successful `/launch/start` response
- Check status via `/logs/stream` before stopping
- If process died, just restart via `/launch/start` again

---

## HTTP 500 - "Permission denied" (writing config/launch)

### Symptoms

- `/config/create` or `/launch/create` return 500 with "Permission denied"
- Usually happens on Linux systems with limited user permissions

### Causes

- `launch_dir` directory not writable by ROS2 user
- Filesystem mounted as read-only
- Previous file created by different user with restricted permissions

### Solution

```bash
# Ensure directory is writable
mkdir -p ~/.ros/launch/config
chmod 755 ~/.ros/launch
chmod 755 ~/.ros/launch/config

# Or specify a user-writable location in api_bridge.yaml
# launch_dir: "/home/uav-user/ros2_ws/.launch"
```

---

## System Starts But Nodes Crash Immediately

### Symptoms

- `/launch/start` returns 200 OK with PID
- `/logs/stream` shows node starting but then crashing within 1-2 seconds
- Common for sensor/RC control nodes

### Causes

- Hardware not connected (GPIO pins, UART not available)
- Permission denied accessing device files (e.g., `/dev/ttyUSB0`)
- Calibration parameters out of range

### Solution - Hardware Check

```bash
# Check GPIO availability (for RC/servo control)
ls /dev/gpiochip*

# Check UART devices (GPS, magnetometer)
ls /dev/ttyUSB* /dev/ttyACM*

# Check permissions (user must have access)
groups $USER | grep dialout
# Should include dialout group
```

---

## GPS Never Acquires Lock

### Symptoms

- Sensor Fusion complains about missing GPS fixes
- Logs show "GPS acquiring lock... 0/24 satellites" forever

### Causes

- GPS module not powered
- Poor antenna placement (indoors, surrounded by metal)
- Cold start (first satellites take 30-60 seconds)
- Bad UART connection

### Solution

1. Check GPS is powered:
   - Verify LED is blinking on GPS module
   - Check power supply voltage (usually 5V)

2. Verify antenna placement:
   - Move antenna outdoors with clear sky view
   - Away from metal structures, walls, trees
   - Keep antenna level (not tilted)

3. Cold start troubleshooting:
   - First power-up takes 30-60 seconds to acquire
   - Move to open field and wait 2-3 minutes
   - Allow GPS to sit powered for 5 minutes before flight

4. Test UART connection:
   ```bash
   cat /dev/ttyUSB0 | head -5
   ```
   Should see NMEA sentences (e.g., `$GPRMC,...`)

---

## Sensor Fusion Publishes Bad Odometry

### Symptoms

- Fusion node starts but odometry estimates are noisy/drifting
- Position error accumulates over time
- Heading (yaw) estimates are wrong

### Root Cause Analysis

**If drifting fast:**
- IMU calibration parameters incorrect
- GPS variance parameters too tight
- Kalman filter measuring noise too low

**If noisy but not drifting:**
- GPS variance set too tight
- IMU bias not removed
- Accelerometer multipath corrections needed

**If compass heading wrong:**
- Magnetic declination incorrect for location
- Soft-iron correction matrix not calibrated
- Magnetometer near metallic objects

### Solution

1. **Verify IMU calibration:**
   - Read current config via `/config/read` with `default: true`
   - Check `mag_decl` matches your location
     - UAV project uses -8.03° for Japan
     - USA: -8° to -12°
     - Europe: -2° to 5°
     - [Find yours](https://www.magnetic-declination.com/)
   - Check `mag_transform` soft-iron matrix is calibrated

2. **Relax GPS variance** if outdoors with multipath:
   ```json
   {
     "id": "zoe_m8q_node",
     "config": {
       "sensor_variance": {
         "pos_std": [6.0, 6.0, 15.0],
         "vel_std": [1.0]
       }
     }
   }
   ```

3. **Adjust Kalman tuning** if drifting persistently:
   ```json
   {
     "id": "fusion_node",
     "config": {
       "kalman": {
         "initial_pos_uncertainty": 50.0,
         "process_noise": {
           "pos": 0.2
         }
       }
     }
   }
   ```

4. **Increase sensor rejection** if accel/mag spikes:
   ```json
   {
     "id": "fusion_node",
     "config": {
       "ahrs": {
         "accel_rejection": 15.0,
         "mag_rejection": 15.0
       }
     }
   }
   ```

---

## High CPU Usage During Flight

### Symptoms

- System runs fine on bench but overheats on aircraft
- Sensor fusion node stops publishing
- RC control becomes unresponsive

### Causes

- IMU/GPS sampling rates too high for processor
- Fusion algorithm parameters too aggressive
- Rosbag2 recording too many topics

### Solution

1. **Reduce sampling rates** (if not critical for control):
   ```json
   {
     "id": "bno085_node",
     "config": {
       "rate_hz": 50
     }
   }
   ```

2. **Record fewer topics** during flight:
   - Skip raw sensor data, keep only odometry
   - Use `/launch/start_recording` with minimal list

3. **Check CPU temperature:**
   ```bash
   watch -n 1 vcgencmd measure_temp  # RPi
   ```

---

## Network Connection Drops During Flight

### Symptoms

- `/logs/stream` disconnects or stalls
- Cannot receive real-time telemetry
- Wi-Fi/Ethernet drops

### Causes

- Wi-Fi signal loss at altitude
- Network interference near motors/ESC
- Aircraft too far from base station

### Solution

1. **Use longer Ethernet cable** if possible
2. **Shield Wi-Fi antenna** from motor EMI:
   - Wrap antenna in aluminum foil away from prop
   - Keep antenna elevated/clear
3. **Reduce flight range** or improve Wi-Fi coverage
4. **Accept that logs won't stream** at altitude:
   - Record all data locally (rosbag2)
   - Download and analyze after landing

---

## Quick Diagnosis Checklist

Before reporting a problem, verify:

- [ ] `/nodes/get_all_nodes` returns all 9 nodes
- [ ] `/config/read` for each node works (default or custom)
- [ ] `/launch/create` with sensor nodes only completes
- [ ] `/launch/start` returns PID without errors
- [ ] `/logs/stream` shows sensor initialization
- [ ] All 3 sensors show "initialized" in logs
- [ ] GPS shows "acquiring lock" or "acquired lock"
- [ ] Fusion shows "publishing /fusion/odom"
- [ ] No "permission denied" or "not found" errors
- [ ] `/launch/stop` cleanly terminates system

If any step fails, that's your problem area.

---

**Next:** [Parameters Reference](parameters-reference.md) or [Integration Guide](integration-examples.md)