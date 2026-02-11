# FastAPI Bridge API Documentation

**Comprehensive REST API for ROS2 UAV Navigation System Management**

This documentation provides complete details for integrating with the FastAPI Bridge—a REST API that manages the complete lifecycle of ROS2 nodes in the UAV Navigation system.

## Quick Navigation

### 📚 Getting Started
- [Overview & Setup](docs/getting-started.md) - Base URL, configuration, connection examples
- [Real-World Workflows](docs/workflows.md) - Pre-flight verification, full flight launch, recalibration

### 🔌 API Endpoints
- [Nodes Discovery](docs/endpoints/nodes.md) - Get available nodes and their capabilities
- [Configuration Management](docs/endpoints/config.md) - Read/create node parameters
- [System Control](docs/endpoints/launch.md) - Launch, stop, and record ROS2 systems
- [Logging & Monitoring](docs/endpoints/logs.md) - Real-time log streaming

### 🛠️ Reference & Examples
- [Parameter Reference](docs/parameters-reference.md) - Complete tuning guide for all 9 nodes
- [Integration Guide](docs/integration-examples.md) - JavaScript/React client code samples
- [Troubleshooting](docs/troubleshooting.md) - Common errors and solutions

### 📋 Appendices
- [Appendix & Checklists](docs/appendix.md) - Useful ROS2 commands, response reference, deployment checklist

---

## What Can You Do With This API?

- **Discover** all available ROS2 nodes and their capabilities (sensors, fusion, control)
- **Configure** node parameters (sensor rates, calibration, thresholds) before launch
- **Generate** custom launch configurations for different flight modes
- **Launch** multi-node systems with proper initialization sequencing and timing
- **Monitor** real-time ROS2 logs and system diagnostics
- **Record** flight data (sensor readings, odometry) to ROS bags for post-flight analysis
- **Stop** systems gracefully with cleanup

---

## API at a Glance

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/nodes/get_all_nodes` | GET | Discover all 9 available nodes |
| `/config/read` | POST | Read node configuration (default or custom) |
| `/config/create` | POST | Create/update node configuration |
| `/launch/create` | POST | Generate launch.yaml from node list |
| `/launch/start` | POST | Start ROS2 system |
| `/launch/stop` | POST | Stop ROS2 system |
| `/launch/start_recording` | POST | Record ROS2 topics to bag file |
| `/logs/stream` | GET | Stream system logs in real-time |

---

## Where to Start?

**New to the FastAPI Bridge?**
→ Start with [Getting Started](docs/getting-started.md)

**Need real examples?**
→ Read [Real-World Workflows](docs/workflows.md)

**Integrating with your frontend?**
→ Check [Endpoints Documentation](docs/endpoints/) and [Integration Guide](docs/integration-examples.md)

**Stuck on an error?**
→ Visit [Troubleshooting](docs/troubleshooting.md)

**Want to tune system parameters?**
→ See [Parameter Reference](docs/parameters-reference.md)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│         FastAPI Bridge (REST API Server)                    │
│              listening on http://0.0.0.0:8000               │
└─────────────────────────────────────────────────────────────┘
                            ↓
            Frontend (Web/Mobile Application)
                 Makes HTTP requests to API
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 Node Manager                          │
├─────────────────────────────────────────────────────────────┤
│  Sensors        Fusion          Control                      │
│  ├─ IMU         ├─ EKF Filter   ├─ RC Receiver              │
│  ├─ GPS         └─ Odometry     └─ Servo Control            │
│  └─ Lidar                                                    │
└─────────────────────────────────────────────────────────────┘
```

---

**Documentation Version:** 2.0 (Modular)  
**Last Updated:** February 11, 2026  
**API Version:** 1.0  


```yaml
api_host: "0.0.0.0"           # Listen on all interfaces (0.0.0.0) or specific IP
api_port: 8000                # API server port
ws_root: "/path/to/workspace" # ROS2 workspace root (usually auto-detected)
log_file_path: "~/.ros/log"   # Where ROS2 launch logs are written
launch_dir: "~/.ros/launch"   # Where generated launch.yaml is saved
rec_dir: "~/.ros/records"     # Where rosbag2 recordings are saved
```

To change these, edit the config file and restart the `fastapi_bridge` node.

### Quick Connection Examples

**Python (requests):**
```python
import requests

BASE_URL = "http://localhost:8000"

# Discover nodes
response = requests.get(f"{BASE_URL}/nodes/get_all_nodes")
nodes = response.json()
print(f"Found {len(nodes)} nodes")
```

**JavaScript (Fetch API):**
```javascript
const BASE_URL = "http://localhost:8000";

async function getAvailableNodes() {
  const response = await fetch(`${BASE_URL}/nodes/get_all_nodes`);
  const nodes = await response.json();
  console.log(`Found ${nodes.length} nodes`);
  return nodes;
}
```

**cURL:**
```bash
curl http://localhost:8000/nodes/get_all_nodes | jq '.'
```

### Error Response Format

All errors follow this format:

```json
{
  "detail": "Descriptive error message explaining what went wrong",
  "status": 400,
  "error_code": "INVALID_CONFIG"
}
```

Common HTTP status codes:
- `200` - Success
- `400` - Bad request (malformed JSON, missing required fields)
- `404` - Resource not found (no launch process running, config file missing)
- `500` - Server error (filesystem permission denied, ROS2 command failed)

## Real-World Workflows

Before diving into endpoint details, here's how these APIs work together in practice:

### Workflow 1: Pre-Flight Sensor Verification

Your frontend needs to verify sensors are configurable before takeoff. Steps:

1. **Discover all nodes** → `GET /nodes/get_all_nodes`
2. **Read default sensor configs** → `POST /config/read` with `default: true` for sensors
3. **Display configs to geek-out technician** → Show calibration values, sampling rates
4. **Modify if necessary** → `POST /config/create` with updated params
5. **Build launch config** → `POST /launch/create` with sensor nodes only
6. **Start verification** → `POST /launch/start`
7. **Monitor output** → `GET /logs/stream` in separate WebSocket/SSE connection
8. **Stop & next step** → `POST /launch/stop`

### Workflow 2: Full Flight Launch

1. Get all nodes → Read configs for all nodes (sensors, fusion, control) → Create custom configs as needed
2. Build full launch file with all 6 nodes (3 sensors + fusion + RC + servo)
3. Start system → Logs show sequential startup (sensors first, fusion waits 2s)
4. Start recording → `POST /launch/start_recording` with odom + sensor topics
5. Return to user → "System ready for flight"
6. After landing → Stop recording, download bags, stop system

### Workflow 3: Emergency Recalibration Mid-Session

1. Land UAV → `POST /launch/stop` (kills all nodes)
2. Read magnetometer config → `POST /config/read` for `mag_calibration`
3. Modify calibration → `POST /config/create` with new soft-iron correction matrix
4. Restart → `POST /launch/create` with updated nodes → `POST /launch/start`
5. Resume flights with new calibration

---

## API Endpoints

### Nodes - Node Discovery & Metadata

Discover all available ROS2 nodes and their capabilities.

#### `GET /nodes/get_all_nodes`

**Use Case:** Frontend UI needs to populate a node selection dropdown, show available sensors, and get parameter file paths for each node.

Returns a list of node descriptions found in the workspace. Each description includes package details, node executable information, topics, services, actions, and the absolute path to the default configuration file.

**Response:**
- `200 OK`: JSON array containing node descriptor objects.

**Real Example Response (5 of 9 available nodes):**
```json
[
  {
    "pkg_name": "sensors",
    "executable_name": "imu_bno085",
    "node_default_id": "bno085_node",
    "default_config": "/home/uav-user/ros2_ws/src/sensors/config/imu_bno085.yaml",
    "sub_topics": [],
    "pub_topics": [
      {
        "name": "imu_bno085/data",
        "type": "sensor_msgs/msg/Imu"
      },
      {
        "name": "imu_bno085/mag",
        "type": "sensor_msgs/msg/MagneticField"
      }
    ],
    "services": [],
    "actions": []
  },
  {
    "pkg_name": "sensors",
    "executable_name": "gps_zoe_m8q",
    "node_default_id": "zoe_m8q_node",
    "default_config": "/home/uav-user/ros2_ws/src/sensors/config/gps_zoe_m8q.yaml",
    "sub_topics": [],
    "pub_topics": [
      {
        "name": "gps_zoe_m8q/fix",
        "type": "sensor_msgs/msg/NavSatFix"
      },
      {
        "name": "gps_zoe_m8q/vel",
        "type": "geometry_msgs/msg/TwistWithCovarianceStamped"
      }
    ],
    "services": [],
    "actions": []
  },
  {
    "pkg_name": "sensor_fusion",
    "executable_name": "sensor_fusion",
    "node_default_id": "fusion_node",
    "default_config": "/home/uav-user/ros2_ws/src/sensor_fusion/config/sensor_fusion.yaml",
    "sub_topics": [
      {
        "name": "imu_bno085/data",
        "type": "sensor_msgs/msg/Imu"
      },
      {
        "name": "imu_bno085/mag",
        "type": "sensor_msgs/msg/MagneticField"
      },
      {
        "name": "gps_zoe_m8q/fix",
        "type": "sensor_msgs/msg/NavSatFix"
      },
      {
        "name": "gps_zoe_m8q/vel",
        "type": "geometry_msgs/msg/TwistWithCovarianceStamped"
      }
    ],
    "pub_topics": [
      {
        "name": "fusion/odom",
        "type": "nav_msgs/msg/Odometry"
      },
      {
        "name": "fusion/linear_acceleration",
        "type": "sensor_msgs/msg/Imu"
      }
    ],
    "services": [],
    "actions": []
  },
  {
    "pkg_name": "rc_control",
    "executable_name": "rc_control",
    "node_default_id": "rc_control",
    "default_config": "/home/uav-user/ros2_ws/src/rc_control/config/rc_control.yaml",
    "sub_topics": [],
    "pub_topics": [
      {
        "name": "rc/channels",
        "type": "std_msgs/msg/Int32MultiArray"
      },
      {
        "name": "rc/mode",
        "type": "std_msgs/msg/String"
      }
    ],
    "services": [],
    "actions": []
  },
  {
    "pkg_name": "servo_control",
    "executable_name": "servo_control",
    "node_default_id": "servo_control",
    "default_config": "/home/uav-user/ros2_ws/src/servo_control/config/servo_control.yaml",
    "sub_topics": [
      {
        "name": "rc/channels",
        "type": "std_msgs/msg/Int32MultiArray"
      },
      {
        "name": "rc/mode",
        "type": "std_msgs/msg/String"
      }
    ],
    "pub_topics": [],
    "services": [],
    "actions": []
  }
]
```

**Field Descriptions:**

| Field | Meaning |
|-------|---------|
| `pkg_name` | ROS2 package name (e.g., "sensors", "sensor_fusion") |
| `executable_name` | Executable name within the package (e.g., "imu_bno085") |
| `node_default_id` | Default node instance name; used in config/launch APIs as the `id` parameter |
| `default_config` | Absolute path to default parameter file; read via `POST /config/read` with `default: true` |
| `sub_topics` | Topics this node **subscribes to** (ingested data); show dependencies in UI |
| `pub_topics` | Topics this node **publishes** (output data); available for recording |
| `services` | ROS2 services exposed (currently empty in this project) |
| `actions` | ROS2 actions exposed (currently empty in this project) |

**Frontend Integration Tip:**
```javascript
// After fetching nodes, extract metadata for dropdowns
const nodes = await getAvailableNodes();
const sensorNodes = nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length === 0);
const fusionNodes = nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length > 0);
const controlNodes = nodes.filter(n => n.sub_topics.length > 0 && n.pub_topics.length === 0);

// Display dependencies
const fusionDeps = fusionNodes[0].sub_topics.map(t => t.name);
// Show: "Sensor Fusion requires: imu_bno085/data, imu_bno085/mag, gps_zoe_m8q/fix, gps_zoe_m8q/vel"
```


### Config - Parameter Management

Manage ROS2 node configuration files (YAML parameters).

#### `POST /config/read`

Reads a configuration file for a specific node. It prioritizes a custom config in the launch directory; if not found (or if `default` is true), it falls back to the package's default config.

**Use Case 1 - View Factory Settings:** Technician logs in before flight and wants to see what the factory calibration is for the BNO085 IMU sensor.

**Use Case 2 - Check for Custom Config:** Flight controller wants to load the previous flight's configuration to verify settings haven't changed.

**Request Body:** `ConfigReadBody`
```json
{
  "id": "bno085_node",
  "exec": "imu_bno085",
  "pkg": "sensors",
  "default": true
}
```

**Real Response - BNO085 IMU (factory default config):**
```json
{
  "config": {
    "rate_hz": 100,
    "frame_id": "imu_link",
    "sensor_calibration": {
      "bias_removal": true,
      "accel_bias": [-1.14582811, -0.01432422, 0.05357129],
      "gyro_bias": [-0.00156234, 0.00089123, 0.00043211],
      "mag_bias": [0.12345, -0.54321, 0.09876],
      "mag_transform": [
        [1.004, 0.012, -0.005],
        [0.008, 1.003, 0.011],
        [-0.003, 0.009, 0.999]
      ]
    },
    "sensor_variance": {
      "accel": [0.01, 0.01, 0.01],
      "gyro": [0.001, 0.001, 0.001],
      "mag": [0.001, 0.001, 0.001]
    },
    "transformation": {
      "imu_rotation": [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
      ],
      "mag_decl": -8.03
    }
  },
  "msg": "OK",
  "default": true
}
```

**Real Response - GPS ZOE M8Q (factory config):**
```json
{
  "config": {
    "interface_type": "uart",
    "rate_hz": 10,
    "frame_id": "gps_link",
    "sensor_variance": {
      "pos_std": [4.0, 4.0, 10.0],
      "vel_std": [0.5]
    }
  },
  "msg": "OK",
  "default": true
}
```

**Real Response - Sensor Fusion (factory config):**
```json
{
  "config": {
    "frames": {
      "map_frame": "map",
      "odom_frame": "odom",
      "base_link_frame": "base_link"
    },
    "sensor_topics": {
      "imu_topic": "imu_bno085/data",
      "mag_topic": "imu_bno085/mag",
      "gps_pos_topic": "gps_zoe_m8q/fix",
      "gps_vel_topic": "gps_zoe_m8q/vel"
    },
    "ahrs": {
      "offset_samples": 100,
      "gain": 0.05,
      "gyro_range": 500.0,
      "accel_rejection": 10.0,
      "mag_rejection": 10.0
    },
    "kalman": {
      "initial_pos_uncertainty": 25.0,
      "initial_vel_uncertainty": 1.0,
      "initial_bias_uncertainty": 0.01,
      "process_noise": {
        "pos": 0.1,
        "vel": 0.01,
        "bias": 0.001
      },
      "measurement_noise": {
        "pos": 25.0,
        "vel": 0.25
      }
    },
    "earth": {
      "gravity": 9.8066,
      "radius": 6378137.0,
      "ref_pos": [34.8761905, 136.9617842, 19.63]
    }
  },
  "msg": "OK",
  "default": true
}
```

**Response Fields:**
- `200 OK`: Returns config object with metadata
  - `config`: Actual parameters (structure depends on node type)
  - `msg`: "OK" on success
  - `default`: Boolean indicating if this is the factory default (true) or custom config (false)
- `500 Internal Server Error`: If config file unreadable (permissions, missing file)

**Important:** If `default: false` but no custom config exists, API automatically returns default config and sets `"default": true` in response.

---

#### `POST /config/create`

Creates or overwrites a configuration file for a specific node in the launch configuration directory. This custom config will be used for the next system launch instead of the default.

**Use Case:** Technician landed after 20-minute flight and noticed drift. Wants to increase IMU sampling rate from 100 Hz to 200 Hz and increase GPS variance estimates for noisier readings.

**Request Body:** `ConfigFileBody`
```json
{
  "id": "bno085_node",
  "config": {
    "rate_hz": 200,
    "frame_id": "imu_link",
    "sensor_calibration": {
      "bias_removal": true,
      "accel_bias": [-1.14582811, -0.01432422, 0.05357129],
      "gyro_bias": [-0.00156234, 0.00089123, 0.00043211],
      "mag_bias": [0.12345, -0.54321, 0.09876],
      "mag_transform": [
        [1.004, 0.012, -0.005],
        [0.008, 1.003, 0.011],
        [-0.003, 0.009, 0.999]
      ]
    },
    "sensor_variance": {
      "accel": [0.01, 0.01, 0.01],
      "gyro": [0.001, 0.001, 0.001],
      "mag": [0.001, 0.001, 0.001]
    },
    "transformation": {
      "imu_rotation": [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
      ],
      "mag_decl": -8.03
    }
  }
}
```

**Real Example - Modifying GPS Config for Noisy Conditions:**
```json
{
  "id": "zoe_m8q_node",
  "config": {
    "interface_type": "uart",
    "rate_hz": 10,
    "frame_id": "gps_link",
    "sensor_variance": {
      "pos_std": [6.0, 6.0, 15.0],
      "vel_std": [1.0]
    }
  }
}
```

**Response:**
- `200 OK`:
  ```json
  {
    "created": "/home/uav-user/.ros/launch/config/bno085_node.yaml",
    "msg": "OK"
  }
  ```
- `500 Internal Server Error`: If `launch_dir/config/` not writable (filesystem permissions issue)

**Important Notes:**
1. Custom config is stored at `<launch_dir>/config/<node_id>.yaml`
2. Custom config only takes effect on **next launch** (not retroactive to running nodes)
3. Invalid parameters are silently ignored by ROS2; Sensor Fusion params like `initial_pos_uncertainty` must match the expected type (float)
4. To reset to factory defaults, delete the custom config file at `<launch_dir>/config/<node_id>.yaml` or call `/config/read` with `default: true`

---

### Launch - System Control & Recording

Control ROS2 system launches and record flight data.

#### `POST /launch/create`

Generates a `launch.yaml` file that defines which nodes to start and in what order. This file is then used by `/launch/start` to begin the system.

**Use Case 1 - Minimal Launch (Debug Mode):** Frontend offers a "Debug Sensors Only" mode that starts just the IMU and GPS without the full fusion stack, so technician can verify raw sensor readings.

**Use Case 2 - Full Flight Stack:** Complete system with all 6 nodes (3 sensors + fusion + RC + servo control) for autonomous flight.

**Request Body:** `LaunchFileBody` - Array of node definitions:
```json
{
  "launch": [
    {
      "pkg": "sensors",
      "exec": "imu_bno085",
      "id": "bno085_node"
    },
    {
      "pkg": "sensors",
      "exec": "gps_zoe_m8q",
      "id": "zoe_m8q_node"
    },
    {
      "pkg": "sensor_fusion",
      "exec": "sensor_fusion",
      "id": "fusion_node"
    },
    {
      "pkg": "rc_control",
      "exec": "rc_control",
      "id": "rc_control"
    },
    {
      "pkg": "servo_control",
      "exec": "servo_control",
      "id": "servo_control"
    }
  ]
}
```

**Response:**
- `200 OK`:
  ```json
  {
    "created": "/home/uav-user/.ros/launch/launch.yaml",
    "msg": "OK"
  }
  ```
- `500 Internal Server Error`: If `launch_dir` not writable or invalid node names provided

**Generated File Structure:** The API creates a ROS2 launch file that:
1. Starts sensors immediately (IMU & GPS)
2. Starts RC control node immediately
3. Starts sensor fusion with 2.0s delay (waits for sensors to initialize)
4. Starts servo control with 1.0s delay (waits for RC control to initialize)

**Important:**
- Config files must exist before launching (created via `/config/create` or use defaults)
- Node execution order follows dependencies: sensor publishers must start before subscribers (fusion)
- File location: `<launch_dir>/launch.yaml` (configured in `api_bridge.yaml`)

---

#### `POST /launch/start`

Starts the ROS2 system by executing the `launch.yaml` file created via `/launch/create`. All configured nodes are started and their output is logged.

**Prerequisite:** Must call `/launch/create` first to generate the launch file.

**Request Body:** Empty POST (no body required)
```json
{}
```

**Real Example Response - Full Flight Stack:**
```json
{
  "msg": "OK",
  "pid": 2847
}
```

**Response:**
- `200 OK`: System started successfully
  - `msg`: "OK"
  - `pid`: Process ID of the ROS2 launch manager; save this for `/launch/stop`
- `500 Internal Server Error`: Launch failed (see error message for reason)
  - "Node executable not found" → executable installed incorrectly
  - "Config file malformed" → YAML syntax error in config
  - "Parameter validation failed" → Invalid parameter values

**Sequential Node Startup (Full Flight Stack):**
```
[00:00] Starting imu_bno085 → Waits for sensor to initialize (~200ms)
[00:00] Starting gps_zoe_m8q → Waits for GPS lock (~5-30s depending on location)
[00:00] Starting rc_control → Reads RC receiver GPIO (instant)
[00:02] Starting sensor_fusion → Begins reading IMU/GPS topics (2s delay allows sensors ready)
[00:03] Starting servo_control → Ready to receive RC/mode updates (1s after RC)
```

**Logs:** All outputs captured in `<log_file_path>/latest_launch.log`; stream via `GET /logs/stream`

---

#### `POST /launch/stop`

Stops the ROS2 system and all running nodes. Must have successfully called `/launch/start` first.

**Prerequisite:** Valid process ID from `/launch/start` response.

**Request Body:** Empty POST
```json
{}
```

**Response:**
- `200 OK`: `{"msg": "OK"}` - System stopped and cleaned up
- `404 Not Found`: No launch process running (either never started or already stopped)
  ```json
  {
    "detail": "No launch process is currently running",
    "status": 404
  }
  ```
- `500 Internal Server Error`: Failed to terminate process (rare; usually filesystem permissions)

**Behavior:**
1. Sends SIGTERM to all ROS2 nodes
2. Waits 5 seconds for graceful shutdown
3. Sends SIGKILL if nodes don't terminate
4. Stops any active recording (launched via `/launch/start_recording`)
5. Closes all topic subscriptions
6. Cleans up ROS2 daemon processes

---

#### `POST /launch/start_recording`

Records ROS2 messages from specified topics to a rosbag2 file. Useful for post-flight analysis: examine sensor readings, debug estimation errors, retrain state estimator. Must be called **after** `/launch/start` (nodes must be running).

**Use Case:** After UAV lands, frontend asks technician which data to save. Technician selects "Save odometry + IMU for post-flight analysis" which records fusion/odom and raw imu_bno085/data.

**Request Body:** `RecordingBody`
```json
{
  "topics": [
    "fusion/odom",
    "imu_bno085/data",
    "imu_bno085/mag",
    "gps_zoe_m8q/fix"
  ]
}
```

**Response:**
- `200 OK`:
  ```json
  {
    "msg": "OK",
    "bag_path": "/home/uav-user/.ros/records/uav_nav_2026-02-11_14-32-05",
    "topics": 4,
    "recording": true
  }
  ```
- `404 Not Found`: No launch process running (start the system first)
- `400 Bad Request`: Topic does not exist or not being published

**Recording Details:**
- **Location:** Stored in `<rec_dir>` (configured in `api_bridge.yaml`, default: `~/.ros/records/`)
- **Naming:** Automatic timestamp: `uav_nav_YYYY-MM-DD_HH-MM-SS`
- **File Format:** ROS2 rosbag2 (SQLite-based, can inspect with `ros2 bag info`, `ros2 bag play`)
- **Size Estimate:** 
  - IMU at 100 Hz: ~50 KB/min
  - GPS at 10 Hz: ~5 KB/min
  - Odometry at 10 Hz: ~8 KB/min
  - **Total:** ~63 KB/min for sensor data, ~2-4 MB for 1-hour flight

**Stop Recording:** Call `/launch/stop` (automatically stops active recording)

---

### Logs - Real-Time System Monitoring

#### `GET /logs/stream`

Streams ROS2 launch logs in real-time. Useful for technician dashboard: display system status, sensor initialization, node startup order, any warnings/errors.

**Use Case:** Frontend opens a "System Monitor" view while system is running. User sees live log updates (100-500ms latency) as nodes start and report their status.

**Response:**
- `200 OK`: Server-Sent Events (SSE) stream of text lines
- Headers: `Content-Type: text/event-stream`, `Cache-Control: no-cache`

**Real Example - Full Flight Stack Logs:**
```
[0.000] ROS2 Launch Manager starting...
[0.123] Loading launch file: /home/uav-user/.ros/launch/launch.yaml
[0.456] Starting node: bno085_node (sensor: imu_bno085)
[0.478] BNO085 IMU initializing... port /dev/ttyUSB0
[0.612] BNO085 IMU initialized, publishing to /imu_bno085/data @ 100 Hz
[0.520] Starting node: zoe_m8q_node (sensor: gps_zoe_m8q)
[0.589] GPS receiver powering on... acquiring satellites
[0.890] Starting node: rc_control
[0.901] RC control reading GPIO pins [20, 25, 16, 12, 24, 21]
[0.925] RC channels available: 6 (throttle, aileron, elevator, etc.)
[1.234] GPS acquiring lock... 3/24 satellites
[2.000] TIMER: Starting delayed node fusion_node (sensor_fusion)
[2.056] Sensor Fusion initializing Kalman filter...
[2.123] Sensor Fusion subscribed to: /imu_bno085/data, /imu_bno085/mag, /gps_zoe_m8q/fix, /gps_zoe_m8q/vel
[2.456] Sensor Fusion publishing: /fusion/odom @ 10 Hz
[3.000] TIMER: Starting delayed node servo_control (1s after rc_control)
[3.087] Servo Control initializing PWM outputs on GPIO [6, 19, 5, 13, 26]
[3.234] Servo Control armed and ready
[4.567] GPS acquired lock! 12/24 satellites, HDOP: 1.8m
[5.000] **SYSTEM READY FOR FLIGHT**
```

**How to Consume (JavaScript EventSource):**
```javascript
const eventSource = new EventSource("http://localhost:8000/logs/stream");

eventSource.onmessage = (event) => {
  const logLine = event.data;
  console.log(logLine);
  // Append to UI log display
  logContainer.innerHTML += logLine + "<br>";
};

eventSource.onerror = (event) => {
  console.error("Log stream error:", event);
  eventSource.close();
};
```

**Python Client:**
```python
import requests

response = requests.get("http://localhost:8000/logs/stream", stream=True)
for line in response.iter_lines():
    if line:
        print(line.decode('utf-8'))
```

**Response Codes:**
- `200 OK`: Stream active; keep connection open to receive updates
- `404 Not Found`: Log file doesn't exist yet (no launch has been run)

**Notes:**
- Stream terminates when `/launch/stop` is called
- Typical latency: 100-500ms from log write to client display
- Typical latency: 100-500ms from log write to client display
- Long-lived connection; frontend should handle disconnects and reconnect automatically

---

## Troubleshooting & Common Errors

### `HTTP 500 - "Node executable not found"`

**Symptoms:** 
- `/launch/start` returns 500 with error message about missing executable

**Causes:**
- ROS2 package not installed in workspace
- Executable not compiled (requires `colcon build`)
- Wrong executable name in request

**Solution:**
1. Verify package is in `src/` directory: `ls src/sensors/`, `ls src/sensor_fusion/`, etc.
2. Rebuild workspace: `colcon build --symlink-install`
3. Verify executable name matches package (get from `/nodes/get_all_nodes`)
4. Check: `ros2 pkg list | grep sensors` (should see package)

---

### `HTTP 500 - "Config file malformed"`

**Symptoms:**
- `/launch/start` fails or node crashes immediately after starting
- Error mentions YAML syntax

**Causes:**
- Config file has YAML syntax error (bad indentation, missing quotes)
- Custom config created via `/config/create` has invalid structure
- Parameter type mismatch (e.g., string provided where float expected)

**Solution:**
1. Validate YAML: `python -m yaml /path/to/config.yaml` (should parse without error)
2. Check custom config at `<launch_dir>/config/<node_id>.yaml` 
3. Compare structure to default config: `cat src/sensors/config/imu_bno085.yaml`
4. Delete custom config and restart to use defaults: `rm ~/.ros/launch/config/<node_id>.yaml`

---

### `HTTP 404 - "No launch process running"` (from `/launch/stop`)

**Symptoms:**
- `/launch/stop` returns 404 when you haven't started a system yet

**Causes:**
- Never called `/launch/start`
- Already called `/launch/stop` once (can't stop twice)
- System crashed and process died naturally

**Solution:**
- Only call `/launch/stop` after successful `/launch/start` response
- Check status via `/logs/stream` before stopping
- If process died, just restart via `/launch/start` again

---

### `HTTP 500 - "Permission denied"` (writing config/launch)

**Symptoms:**
- `/config/create` or `/launch/create` return 500 with "Permission denied"
- Usually happens on Linux systems with limited user permissions

**Causes:**
- `launch_dir` directory not writable by ROS2 user
- Filesystem mounted as read-only
- Previous file created by different user with restricted permissions

**Solution:**
```bash
# Ensure directory is writable
mkdir -p ~/.ros/launch/config
chmod 755 ~/.ros/launch
chmod 755 ~/.ros/launch/config

# Or specify a user-writable location in api_bridge.yaml
# launch_dir: "/home/uav-user/ros2_ws/.launch"
```

---

### System Starts But Nodes Crash Immediately

**Symptoms:**
- `/launch/start` returns 200 OK with PID
- `/logs/stream` shows node starting but then crashing within 1-2 seconds
- Common for sensor/RC control nodes

**Causes:**
- Hardware not connected (GPIO pins, UART not available)
- Permission denied accessing device files (e.g., `/dev/ttyUSB0`)
- Calibration parameters out of range

**Solution - Hardware Check:**
```bash
# Check GPIO availability (for RC/servo control)
ls /dev/gpiochip* 

# Check UART devices (GPS, magnetometer)
ls /dev/ttyUSB* /dev/ttyACM*

# Check permissions (user must have access)
groups $USER | grep dialout  # Should include dialout group
```

---

### GPS Never Acquires Lock (`gps_zoe_m8q` not publishing)

**Symptoms:**
- Sensor Fusion complains about missing GPS fixes
- Logs show "GPS acquiring lock... 0/24 satellites" forever

**Causes:**
- GPS module not powered
- Poor antenna placement (indoors, surrounded by metal)
- Cold start (first satellites take 30-60 seconds)
- Bad UART connection

**Solution:**
1. Check GPS is powered: verify LED is blinking
2. Move antenna outdoors with clear sky view
3. If never acquired ever, move to open field and wait 2-3 minutes
4. Test UART connection: `cat /dev/ttyUSB0` (should see NMEA messages if GPS configured)

---

### Sensor Fusion Publishes Bad Odometry

**Symptoms:**
- Fusion node starts but odometry estimates are noisy/drifting
- Position error accumulates over time

**Causes:**
- IMU calibration parameters incorrect (especially magnetometer)
- GPS variance parameters too tight (overweighting noisy GPS)
- Kalman filter tuning needs adjustment

**Solution:**
1. **Verify IMU calibration:** Read current config via `/config/read` with `default: true`
   - Check `mag_decl` matches your location (UAV is at -8.03° in Japan; yours different?)
   - Check `mag_transform` (soft-iron correction matrix) has been calibrated
2. **Relax GPS variance:** Increase `sensor_variance.pos_std` if outdoors with poor signal
   ```json
   "sensor_variance": {
     "pos_std": [6.0, 6.0, 15.0],
     "vel_std": [1.0]
   }
   ```
3. **Increase Kalman gains:** Increase `initial_pos_uncertainty` if trusting sensors less
   ```json
   "kalman": {
     "initial_pos_uncertainty": 50.0
   }
   ```

---

## Appendix A: Complete Parameter Reference

### Sensor Fusion Node (`sensor_fusion`)

| Parameter | Type | Default | Units/Range | Description |
|-----------|------|---------|-------------|-------------|
| `frames.map_frame` | string | "map" | N/A | Fixed coordinate frame (global reference) |
| `frames.odom_frame` | string | "odom" | N/A | Moving reference frame (odometry estimates) |
| `frames.base_link_frame` | string | "base_link" | N/A | Vehicle body frame |
| `sensor_topics.imu_topic` | string | "imu_bno085/data" | topic name | IMU data source |
| `sensor_topics.mag_topic` | string | "imu_bno085/mag" | topic name | Magnetometer data source |
| `sensor_topics.gps_pos_topic` | string | "gps_zoe_m8q/fix" | topic name | GPS position source |
| `sensor_topics.gps_vel_topic` | string | "gps_zoe_m8q/vel" | topic name | GPS velocity source |
| `ahrs.offset_samples` | int | 100 | 50-500 | Num samples for initial bias estimation |
| `ahrs.gain` | float | 0.05 | 0.001-0.1 | Complementary filter gain (lower = smoother, slower) |
| `ahrs.gyro_range` | float | 500.0 | dps | Gyroscope measurement range (degrees/sec) |
| `ahrs.accel_rejection` | float | 10.0 | g | Ignore accel readings > this value (shock detection) |
| `ahrs.mag_rejection` | float | 10.0 | µT | Ignore mag readings > this value (interference detection) |
| `kalman.initial_pos_uncertainty` | float | 25.0 | m² | Initial covariance for position (higher = less trust) |
| `kalman.initial_vel_uncertainty` | float | 1.0 | m²/s² | Initial covariance for velocity |
| `kalman.initial_bias_uncertainty` | float | 0.01 | unitless | Initial covariance for gyro bias |
| `kalman.process_noise.pos` | float | 0.1 | N/A | How much process noise in position (models world uncertainty) |
| `kalman.process_noise.vel` | float | 0.01 | N/A | Process noise in velocity |
| `kalman.process_noise.bias` | float | 0.001 | N/A | Process noise in gyro bias |
| `kalman.measurement_noise.pos` | float | 25.0 | N/A | GPS position measurement noise (matches `sensor_variance.pos_std²`) |
| `kalman.measurement_noise.vel` | float | 0.25 | N/A | GPS velocity measurement noise |
| `earth.gravity` | float | 9.8066 | m/s² | Standard gravity constant (varies by latitude) |
| `earth.radius` | float | 6378137.0 | m | Earth radius (for lat/lon to meters conversion) |
| `earth.ref_pos[0]` | float | 34.8761905 | degrees | Reference latitude (where UAV operates) |
| `earth.ref_pos[1]` | float | 136.9617842 | degrees | Reference longitude |
| `earth.ref_pos[2]` | float | 19.63 | m | Reference altitude above sea level |

**Tuning Guidance:**
- **Drifty odometry?** Decrease `initial_pos_uncertainty` (trust sensors more) or increase `process_noise.pos`
- **Jerky/noisy odometry?** Increase `initial_pos_uncertainty` (trust sensors less)
- **Accel spikes throwing off attitude?** Increase `ahrs.accel_rejection`
- **Magnetometer interference?** Increase `ahrs.mag_rejection`

---

### IMU BNO085 Node (`imu_bno085`)

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `rate_hz` | int | 100 | Hz | Publishing frequency (50-250 typical) |
| `frame_id` | string | "imu_link" | N/A | ROS2 frame name for this IMU |
| `sensor_calibration.bias_removal` | bool | true | N/A | Remove gyro bias before publishing |
| `sensor_calibration.accel_bias[0,1,2]` | float | see note | m/s² | X/Y/Z accelerometer offset (factory calibrated) |
| `sensor_calibration.gyro_bias[0,1,2]` | float | see note | rad/s | X/Y/Z gyroscope offset (factory calibrated) |
| `sensor_calibration.mag_bias[0,1,2]` | float | see note | µT | X/Y/Z magnetometer offset (from soft-iron cal) |
| `sensor_calibration.mag_transform[3x3]` | float matrix | see note | N/A | Soft-iron correction matrix (calibration) |
| `sensor_variance.accel[0,1,2]` | float | 0.01 | (m/s²)² | Accel measurement noise covariance |
| `sensor_variance.gyro[0,1,2]` | float | 0.001 | (rad/s)² | Gyro measurement noise covariance |
| `sensor_variance.mag[0,1,2]` | float | 0.001 | µT² | Mag measurement noise covariance |
| `transformation.imu_rotation[3x3]` | float matrix | Identity | N/A | Rotation matrix for mounting orientation (e.g., if IMU upside-down) |
| `transformation.mag_decl` | float | -8.03 | degrees | Magnetic declination at UAV location (affects compass heading) |

**Note on Calibration Values:**
The default calibration values for BNO085 are factory-calibrated and should not be modified unless you:
1. Move to a different location (change `mag_decl`) 
2. Re-run magnetometer calibration (via `calibration` package)
3. Remount the IMU at a different orientation (modify `imu_rotation`)

**Magnetic Declination Reference:**
- Most of USA: -8° to -12°
- Europe: -2° to 5°
- Japan: -7° to -9° (Japan specific: -8.03°)
- Find yours: https://www.magnetic-declination.com/

---

### GPS ZOE M8Q Node (`gps_zoe_m8q`)

| Parameter | Type | Default | Units/Values | Description |
|-----------|------|---------|---------|-------------|
| `interface_type` | string | "uart" | "uart", "i2c" | Communication protocol |
| `rate_hz` | int | 10 | Hz | Position solution rate (1-10 typical) |
| `frame_id` | string | "gps_link" | N/A | ROS2 frame name for GPS antenna |
| `sensor_variance.pos_std[0,1,2]` | float | [4.0, 4.0, 10.0] | m | Position std dev in East, North, Up |
| `sensor_variance.vel_std` | float | 0.5 | m/s | Velocity std dev (all axes) |

**Position Variance Guidance:**
- **Clear sky, good signal:** [2.0, 2.0, 5.0] m (1-2 satellites per axis)
- **Urban canyon, trees:** [6.0, 6.0, 12.0] m (more multipath)
- **Indoors/poor signal:** [10.0, 10.0, 20.0] m (if it locks at all)

---

### RC Control Node (`rc_control`)

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `gpiochip` | int | 4 | N/A | GPIO chip number (usually 4 for RPi) |
| `publish_rate` | int | 50 | Hz | How often to publish RC channel values |
| `failsafe_timeout` | float | 1.0 | sec | If no signal > this, trigger failsafe |
| `gpio_pins` | int[] | [20,25,16,12,24,21] | GPIO # | Pins for RC channels 1-6 (BCM numbering) |
| `failsafe_values` | int[] | [1500,1500,1000,1500,1500,1000] | µs | PWM values to use if signal lost |

**GPIO Pin Mapping (BCM numbers for Raspberry Pi):**
- Pin 20: RC Channel 1 (Throttle)
- Pin 25: RC Channel 2 (Aileron) 
- Pin 16: RC Channel 3 (Elevator)
- Pin 12: RC Channel 4 (Rudder)
- Pin 24: RC Channel 5 (Mode switch)
- Pin 21: RC Channel 6 (Aux)

**Failsafe Strategy:**
Default failsafe cuts throttle (1000 µs = minimum) ensuring immediate motor stoppage if RC link lost.

---

### Servo Control Node (`servo_control`)

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `gpiochip` | int | 4 | N/A | GPIO chip number |
| `pwm_freq` | int | 50 | Hz | PWM frequency for servos/ESC (50 standard) |
| `l_aileron` | int | 6 | GPIO # | GPIO pin for left aileron servo |
| `elevator` | int | 19 | GPIO # | GPIO pin for elevator servo |
| `esc_rotor` | int | 5 | GPIO # | GPIO pin for ESC/motor control |
| `rudder` | int | 13 | GPIO # | GPIO pin for rudder servo |
| `r_aileron` | int | 26 | GPIO # | GPIO pin for right aileron servo |
| `rc_ch_map.l_aileron` | int | 0 | channel # | Map RC channel to left aileron |
| `rc_ch_map.elevator` | int | 1 | channel # | Map RC channel to elevator |
| `rc_ch_map.rotor` | int | 2 | channel # | Map RC channel to rotor (throttle) |
| `rc_ch_map.rudder` | int | 3 | channel # | Map RC channel to rudder |
| `rc_ch_map.r_aileron` | int | 4 | channel # | Map RC channel to right aileron |
| `safety.arming_duration` | float | 1.0 | sec | How long to hold arming switch before motors active |
| `safety.esc_min_pulse` | float | 1000.0 | µs | ESC minimum PWM (motor off) |
| `safety.esc_max_pulse` | float | 2000.0 | µs | ESC maximum PWM (full throttle) |
| `safety.servo_min_pulse` | float | 1000.0 | µs | Servo minimum PWM (full negative deflection) |
| `safety.servo_max_pulse` | float | 2000.0 | µs | Servo maximum PWM (full positive deflection) |

**Typical RC-to-Servo Mapping:**
```
RC Channel 1 (1000-2000 µs) → Left Aileron (-45° to +45°)
RC Channel 2 (1000-2000 µs) → Elevator (-45° to +45°)
RC Channel 3 (1000-2000 µs) → Rotor/ESC (0-100% throttle)
RC Channel 4 (1000-2000 µs) → Rudder (-45° to +45°)
RC Channel 5 (1000-2000 µs) → Right Aileron (-45° to +45°)
```

---

## Appendix B: Sample Frontend Integration Code

### Complete Flight Control Workflow (JavaScript/React)

```javascript
const API_BASE = "http://localhost:8000";

// Step 1: Discover available nodes
async function discoverNodes() {
  const res = await fetch(`${API_BASE}/nodes/get_all_nodes`);
  const nodes = await res.json();
  console.log("Found nodes:", nodes.map(n => `${n.executable_name} (${n.pkg_name})`));
  return nodes;
}

// Step 2: Read sensor configs
async function readSensorConfig(nodeId, exec, pkg) {
  const res = await fetch(`${API_BASE}/config/read`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ id: nodeId, exec, pkg, default: true })
  });
  return res.json();
}

// Step 3: Modify config if needed
async function updateSensorConfig(nodeId, updatedConfig) {
  const res = await fetch(`${API_BASE}/config/create`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ id: nodeId, config: updatedConfig })
  });
  return res.json();
}

// Step 4: Create launch file
async function createLaunchFile(nodesList) {
  const launchConfig = {
    launch: nodesList.map(n => ({
      pkg: n.pkg_name,
      exec: n.executable_name,
      id: n.node_default_id
    }))
  };
  
  const res = await fetch(`${API_BASE}/launch/create`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(launchConfig)
  });
  return res.json();
}

// Step 5: Start the system
async function startSystem() {
  const res = await fetch(`${API_BASE}/launch/start`, { method: "POST" });
  const { pid, msg } = await res.json();
  console.log(`System started with PID ${pid}`);
  return pid;
}

// Step 6: Stream logs to UI
function streamLogs(logContainer) {
  const eventSource = new EventSource(`${API_BASE}/logs/stream`);
  
  eventSource.onmessage = (event) => {
    const line = event.data;
    logContainer.innerHTML += `${line}<br>`;
    logContainer.scrollTop = logContainer.scrollHeight; // Auto-scroll
  };
  
  eventSource.onerror = () => {
    console.error("Log stream error");
    eventSource.close();
  };
  
  return eventSource;
}

// Step 7: Start recording
async function startRecording(topics) {
  const res = await fetch(`${API_BASE}/launch/start_recording`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ topics })
  });
  const { bag_path } = await res.json();
  console.log(`Recording to: ${bag_path}`);
  return bag_path;
}

// Step 8: Stop the system
async function stopSystem() {
  const res = await fetch(`${API_BASE}/launch/stop`, { method: "POST" });
  console.log("System stopped");
  return res.json();
}

// Complete flight sequence
async function runFlight() {
  console.log("=== Pre-Flight Sequence ===");
  const nodes = await discoverNodes();
  
  // Get all nodes for full system
  const sensorNodes = nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length === 0);
  const fusionNode = nodes.find(n => n.executable_name === "sensor_fusion");
  const controlNodes = nodes.filter(n => n.sub_topics.length > 0);
  
  const systemNodes = [...sensorNodes, fusionNode, ...controlNodes];
  
  // Verify configs
  for (const node of systemNodes) {
    const cfg = await readSensorConfig(node.node_default_id, node.executable_name, node.pkg_name);
    console.log(`${node.executable_name} config loaded:`, cfg.default ? "factory" : "custom");
  }
  
  // Create launch file
  await createLaunchFile(systemNodes);
  
  console.log("=== Starting Flight ===");
  const pid = await startSystem();
  
  // Monitor logs
  const logDiv = document.getElementById("system-logs");
  streamLogs(logDiv);
  
  // Wait for system to fully initialize (sensors, fusion ready)
  await new Promise(resolve => setTimeout(resolve, 5000));
  
  // Start recording
  const recordedTopics = [
    "fusion/odom",
    "imu_bno085/data",
    "gps_zoe_m8q/fix",
    "rc/channels"
  ];
  await startRecording(recordedTopics);
  
  console.log("=== READY FOR FLIGHT ===");
  
  // Flight duration...
  // After landing:
  await stopSystem();
  console.log("Flight complete, data recorded");
}
```

---

## Appendix C: Useful ROS2 Commands for Debugging

```bash
# List all active nodes
ros2 node list

# Show node details
ros2 node info /bno085_node

# List all topics and their types
ros2 topic list -t

# Echo a specific topic (debug sensor output)
ros2 topic echo /imu_bno085/data

# Record topics manually (alternative to API)
ros2 bag record -o my_flight /fusion/odom /imu_bno085/data

# Playback recorded data
ros2 bag play my_flight

# Check launch file
cat ~/.ros/launch/launch.yaml

# View logs
tail -f ~/.ros/log/latest_launch.log

# Inspect default config
cat src/sensors/config/imu_bno085.yaml

# Rebuild workspace after config changes
colcon build --symlink-install
```

---

## Appendix D: API Response Status Reference

| Endpoint | Status | Meaning | Common Response |
|----------|--------|---------|-----------------|
| GET /nodes/get_all_nodes | 200 | Success | JSON array of 9 nodes |
| POST /config/read | 200 | Config loaded | `{"config": {...}, "default": bool}` |
| POST /config/read | 500 | Read failed | `{"detail": "File not readable"}` |
| POST /config/create | 200 | Config written | `{"created": "/path/..."}` |
| POST /config/create | 500 | Write failed | `{"detail": "Permission denied"}` |
| POST /launch/create | 200 | Launch file generated | `{"created": "/path/..."}` |
| POST /launch/create | 500 | Generation failed | `{"detail": "Invalid node names"}` |
| POST /launch/start | 200 | System started | `{"msg": "OK", "pid": 2847}` |
| POST /launch/start | 500 | Launch failed | `{"detail": "Node crashed: ..."}` |
| POST /launch/stop | 200 | System stopped | `{"msg": "OK"}` |
| POST /launch/stop | 404 | No process running | `{"detail": "No launch process..."}` |
| POST /launch/start_recording | 200 | Recording started | `{"bag_path": "/path/..."}` |
| POST /launch/start_recording | 404 | No launch running | `{"detail": "No launch process..."}` |
| GET /logs/stream | 200 | Stream open | SSE text stream |
| GET /logs/stream | 404 | No log file | `"Log file does not exist"` |

---

## Final Checklist: Before Deployment

- [ ] Verify all 9 nodes appear in `/nodes/get_all_nodes` response
- [ ] Test reading default config for at least 3 node types (sensor, fusion, control)
- [ ] Test creating custom config, verify file appears in `<launch_dir>/config/`
- [ ] Create launch file with all 6 nodes; verify file at `<launch_dir>/launch.yaml`
- [ ] Start system; verify `/logs/stream` shows sequential startup with correct delays
- [ ] Stop system; verify all processes terminated cleanly
- [ ] Test recording; verify rosbag2 file created with expected contents
- [ ] Test error cases: wrong node name, missing config, no launch process
- [ ] Verify frontend can handle SSE stream disconnect/reconnect
- [ ] Test on actual hardware (RPi with GPIO, UART devices) before production

---

**Documentation Version:** 1.0  
**Last Updated:** February 11, 2026  
**API Version:** 1.0  
**Target Users:** Frontend developers, Flight operators, System integrators
