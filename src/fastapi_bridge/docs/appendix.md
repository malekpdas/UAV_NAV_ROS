# Appendix & Resources

Useful ROS2 commands, API response reference, and deployment checklist.

## ROS2 Debugging Commands

Helpful commands for troubleshooting and monitoring the system.

### Node Management

```bash
# List all active nodes
ros2 node list

# Show detailed node information
ros2 node info /bno085_node

# Check if specific node is running
ros2 node list | grep fusion
```

### Topic Inspection

```bash
# List all topics and their message types
ros2 topic list -t

# Echo a specific topic (watch live data)
ros2 topic echo /imu_bno085/data

# Show topic bandwidth usage
ros2 topic bw /fusion/odom

# Measure topic frequency
ros2 topic hz /imu_bno085/data
```

### Recording & Playback

```bash
# Record topics to rosbag2
ros2 bag record -o my_flight /fusion/odom /imu_bno085/data

# View bag information
ros2 bag info my_flight

# Playback recorded data
ros2 bag play my_flight

# Convert rosbag1 to rosbag2 (if needed)
rosbag2_converter your.bag
```

### Configuration & Launching

```bash
# Check generated launch file
cat ~/.ros/launch/launch.yaml

# View current logs
tail -f ~/.ros/log/latest_launch.log

# Inspect default config files
cat src/sensors/config/imu_bno085.yaml
cat src/sensor_fusion/config/sensor_fusion.yaml

# Rebuild workspace after code/config changes
colcon build --symlink-install

# Source setup in new terminal
source install/setup.bash
```

### System Information

```bash
# List all ROS2 packages
ros2 pkg list

# Show package location
ros2 pkg prefix sensors

# Verify executable exists
ros2 pkg executables sensors

# Check ROS2 environment
echo $ROS_DOMAIN_ID
echo $ROS_DISTRO

# Current ROS2 version
ros2 --version
```

### Hardware Diagnostics

```bash
# Check GPIO availability (RPi)
ls /dev/gpiochip*

# Check UART devices
ls /dev/ttyUSB* /dev/ttyACM*

# Check user permissions
groups $USER

# Monitor CPU/memory during flight
watch -n 1 'top -b -n 1 | head -15'

# RPi temperature monitoring
watch -n 1 vcgencmd measure_temp
```

---

## API Response Status Reference

Complete matrix of all endpoints, status codes, and example responses.

| Endpoint | Method | Status | Response | Notes |
|----------|--------|--------|----------|-------|
| `/nodes/get_all_nodes` | GET | 200 | `[{...node...}, ...]` | Array of 9 nodes |
| `/nodes/get_all_nodes` | GET | 500 | `{"detail": "..."}` | Rare; descriptor files missing |
| `/config/read` | POST | 200 | `{"config": {...}, "default": bool}` | Config + source indicator |
| `/config/read` | POST | 400 | `{"detail": "Missing field: id"}` | Malformed request |
| `/config/read` | POST | 500 | `{"detail": "File not readable"}` | Permission/missing file|
| `/config/create` | POST | 200 | `{"created": "/path/..."}` | Config saved successfully |
| `/config/create` | POST | 400 | `{"detail": "Invalid JSON"}` | Malformed config object |
| `/config/create` | POST | 500 | `{"detail": "Permission denied"}` | Directory not writable |
| `/launch/create` | POST | 200 | `{"created": "/path/..."}` | Launch file generated |
| `/launch/create` | POST | 400 | `{"detail": "Invalid node names"}` | Unknown pkg/exec |
| `/launch/create` | POST | 500 | `{"detail": "Write failed"}` | Directory not writable |
| `/launch/start` | POST | 200 | `{"msg": "OK", "pid": 2847}` | System started, save PID |
| `/launch/start` | POST | 500 | `{"detail": "Node crashed: ..."}` | Check logs for reason |
| `/launch/stop` | POST | 200 | `{"msg": "OK"}` | System stopped cleanly |
| `/launch/stop` | POST | 404 | `{"detail": "No launch process..."}` | Never started or already stopped |
| `/launch/stop` | POST | 500 | `{"detail": "Kill failed"}` | Rare; zombie process |
| `/launch/start_recording` | POST | 200 | `{"msg": "OK", "bag_path": "..."}` | Recording started |
| `/launch/start_recording` | POST | 400 | `{"detail": "Topic not found"}` | Topic doesn't exist |
| `/launch/start_recording` | POST | 404 | `{"detail": "No launch process"}` | System not running |
| `/logs/stream` | GET | 200 | `text/event-stream` | Log lines via SSE |
| `/logs/stream` | GET | 404 | `{"detail": "Log file missing"}` | No launch has run yet |

---

## Deployment Checklist

10-point verification list before deploying to production.

### Pre-Deployment

- [ ] **Verify all 9 nodes discoverable**
  ```bash
  curl http://localhost:8000/nodes/get_all_nodes | jq 'length'  # Should be 9
  ```

- [ ] **Test reading default configs** (at least 3 node types)
  ```bash
  # IMU
  curl -X POST http://localhost:8000/config/read \
    -H "Content-Type: application/json" \
    -d '{"id":"bno085_node","exec":"imu_bno085","pkg":"sensors","default":true}'
  
  # Fusion
  curl -X POST http://localhost:8000/config/read \
    -H "Content-Type: application/json" \
    -d '{"id":"fusion_node","exec":"sensor_fusion","pkg":"sensor_fusion","default":true}'
  
  # Servo
  curl -X POST http://localhost:8000/config/read \
    -H "Content-Type: application/json" \
    -d '{"id":"servo_control","exec":"servo_control","pkg":"servo_control","default":true}'
  ```

- [ ] **Test creating custom config**
  ```bash
  curl -X POST http://localhost:8000/config/create \
    -H "Content-Type: application/json" \
    -d '{"id":"test_node","config":{"test_param":123}}'
  
  ls ~/.ros/launch/config/test_node.yaml  # Should exist
  ```

- [ ] **Create launch file with all 6 nodes**
  ```bash
  # Test launch creation
  # Verify file at ~/.ros/launch/launch.yaml
  ```

- [ ] **Start system & verify sequential startup**
  ```bash
  # Should show sensors first → fusion with 2s delay → control
  # Check logs for "SYSTEM READY FOR FLIGHT"
  ```

### Hardware Testing

- [ ] **Test recording** with actual data
  - Verify rosbag2 file created
  - Check file size reasonable (~60 KB/min)
  - Play back with `ros2 bag play`

- [ ] **Test error handling**
  - Wrong node name → 400/500
  - Missing config → 404 or fallback to default
  - No launch running on `/launch/stop` → 404

- [ ] **Test frontend SSE connection**
  - `/logs/stream` connects successfully
  - Receives all startup logs
  - Handles disconnect/reconnect gracefully

- [ ] **Test on actual hardware** (RPi with GPIO/UART)
  - RC control GPIO pins accessible
  - GPS/IMU UART devices found
  - All sensors initialize properly
  - No "permission denied" errors

### Load Testing

- [ ] **Multiple `/config/read` calls** don't cause deadlock
- [ ] **Recording 4+ topics** simultaneously works
- [ ] **Long-running system** (30+ min) stable
- [ ] **Rapid start/stop cycles** don't crash API

---

## Performance Benchmarks

Expected performance characteristics for reference.

### API Response Times

| Endpoint | Typical Time | Max Time |
|----------|--------------|----------|
| `/nodes/get_all_nodes` | 50-100 ms | 500 ms |
| `/config/read` | 10-50 ms | 200 ms |
| `/config/create` | 20-100 ms | 500 ms |
| `/launch/create` | 50-200 ms | 1000 ms |
| `/launch/start` | 100-500 ms | 5000 ms |
| `/launch/stop` | 100-1000 ms | 10000 ms |
| `/logs/stream` established | <100 ms | 500 ms |

### Rosbag2 Recording Size

| Topics | Duration | Size | Rate |
|--------|----------|------|------|
| IMU only (100 Hz) | 1 min | 3 MB | 50 KB/s |
| GPS only (10 Hz) | 1 min | 0.3 MB | 5 KB/s |
| Odometry only (10 Hz) | 1 min | 0.5 MB | 8 KB/s |
| All sensors + odom | 1 min | 4 MB | 63 KB/s |
| 1-hour flight (all) | 60 min | ~240 MB | 4 MB/s |

### CPU/Memory Usage

| Component | CPU | RAM |
|-----------|-----|-----|
| FastAPI Bridge (idle) | <1% | 50 MB |
| Sensor Fusion node | 5-15% | 100 MB |
| All 6 nodes running | 30-40% | 300-400 MB |
| Recording (4 topics) | +5-10% | +50 MB |
| ROS middleware (colcon) | Varies | 100-200 MB |

**Note:** Measured on Raspberry Pi 4 (4GB). Adjust expectations for different hardware.

---

## Environment Setup Script

Bash script to automate setup on new machine.

```bash
#!/bin/bash

# Setup FastAPI Bridge environment

set -e

# 1. Install Python dependencies
pip install fastapi uvicorn pyyaml

# 2. Create required directories
mkdir -p ~/.ros/launch/config
mkdir -p ~/.ros/records
mkdir -p ~/.ros/log

# 3. Set permissions
chmod 755 ~/.ros/launch
chmod 755 ~/.ros/launch/config
chmod 755 ~/.ros/records
chmod 755 ~/.ros/log

# 4. (Optional) Install ROS2 tools
# sudo apt install ros-humble-rosbag2  # Ubuntu
# brew install ros-rosbag2              # macOS

# 5. Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# 6. Source setup
source install/setup.bash

# 7. Test basic connectivity
echo "Testing API..."
curl -s http://localhost:8000/nodes/get_all_nodes | jq 'length'

echo "Setup complete!"
```

---

## Troubleshooting Decision Tree

```
Is API reachable? (Can you curl it?)
├─ NO → Check if fastapi_bridge node is running
│   └─ ros2 node list | grep bridge
│
├─ YES → Can you discover nodes?
│   ├─ NO → Check descriptor files exist
│   │   └─ ls src/*/descriptor/*.yaml
│   │
│   └─ YES → Can you read configs?
│       ├─ NO → Check file permissions
│       │   └─ chmod 755 ~/.ros/launch/config
│       │
│       └─ YES → Can you create launch?
│           ├─ NO → Check launch_dir is writable
│           │   └─ ls -l ~/.ros/launch
│           │
│           └─ YES → Can you start system?
│               ├─ NO → Check node executables installed
│               │   └─ ros2 pkg list | grep sensors
│               │
│               └─ YES → Check logs for errors
│                   └─ curl http://localhost:8000/logs/stream
```

---

## Common Configuration Values

Quick reference for typical parameter values by environment.

### GPS Configuration

**Suburban/Open Area:**
```json
{"sensor_variance.pos_std": [4.0, 4.0, 10.0]}
```

**Urban/Trees:**
```json
{"sensor_variance.pos_std": [6.0, 6.0, 12.0]}
```

**RTK Mode:**
```json
{"sensor_variance.pos_std": [0.1, 0.1, 0.2]}
```

### IMU Sampling Rates

**Battery-conscious (slow flight):**
```json
{"rate_hz": 50}
```

**Balanced (default):**
```json
{"rate_hz": 100}
```

**Aggressive control:**
```json
{"rate_hz": 200}
```

### Sensor Fusion Tuning

**Trust GPS more:**
```json
{"kalman.initial_pos_uncertainty": 10.0}
```

**Trust IMU more:**
```json
{"kalman.initial_pos_uncertainty": 50.0}
```

**Smoother response:**
```json
{"ahrs.gain": 0.02}
```

**Faster response:**
```json
{"ahrs.gain": 0.1}
```

---

## API Documentation Links & References

- [JSON Schema Validator](https://www.jsonschema.org/)
- [ROS2 Official Documentation](https://docs.ros.org/)
- [Rosbag2 User Guide](https://github.com/ros2/rosbag2)
- [Server-Sent Events (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events)
- [RESTful API Design Best Practices](https://restfulapi.net/)

---

**Documentation Complete!**

For questions or issues, refer back to:
1. [Troubleshooting Guide](troubleshooting.md)
2. [Parameter Reference](parameters-reference.md)
3. [Integration Examples](integration-examples.md)