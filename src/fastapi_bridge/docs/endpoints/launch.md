# Launch - System Control & Recording

Control ROS2 system launches and record flight data.

## POST /launch/create

Generates a `launch.yaml` file that defines which nodes to start and in what order. This file is then used by `/launch/start` to begin the system.

### Use Cases

**Use Case 1 - Minimal Launch (Debug Mode):** Frontend offers a "Debug Sensors Only" mode that starts just the IMU and GPS without the full fusion stack, so technician can verify raw sensor readings.

**Use Case 2 - Full Flight Stack:** Complete system with all 6 nodes (3 sensors + fusion + RC + servo control) for autonomous flight.

### Request Body

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

| Field | Type | Description |
|-------|------|-------------|
| `launch` | array | List of node definitions to start |
| `pkg` | string | ROS2 package name (from `/nodes/get_all_nodes`) |
| `exec` | string | Executable name within package |
| `id` | string | Node instance ID (label for config/logging) |

### Response

**200 OK**:
```json
{
  "created": "/home/uav-user/.ros/launch/launch.yaml",
  "msg": "OK"
}
```

**500 Internal Server Error**: If `launch_dir` not writable or invalid node names provided

### Generated File Behavior

The API creates a ROS2 launch file that:
1. Starts sensors immediately (IMU & GPS)
2. Starts RC control node immediately
3. Starts sensor fusion with **2.0s delay** (waits for sensors to initialize)
4. Starts servo control with **1.0s delay** (waits for RC control to initialize)

### Important Notes

- **Config files must exist** before launching (created via `/config/create` or use defaults)
- **Node execution order** follows dependencies: sensor publishers must start before subscribers (fusion)
- **File location:** `<launch_dir>/launch.yaml` (configured in `api_bridge.yaml`)
- **Overwrites previous:** Each call to `/launch/create` replaces the previous `launch.yaml`

---

## POST /launch/start

Starts the ROS2 system by executing the `launch.yaml` file created via `/launch/create`. All configured nodes are started and their output is logged.

**Prerequisite:** Must call `/launch/create` first to generate the launch file.

### Request Body

Empty POST (no body required)
```json
{}
```

### Real Example Response - Full Flight Stack

```json
{
  "msg": "OK",
  "pid": 2847
}
```

| Field | Type | Description |
|-------|------|-------------|
| `msg` | string | "OK" on success |
| `pid` | integer | Process ID of ROS2 launch manager (save for `/launch/stop`) |

### Response Status Codes

**200 OK**: System started successfully
- Store the `pid` value for use in `/launch/stop`

**500 Internal Server Error**: Launch failed
- "Node executable not found" → executable not installed/compiled
- "Config file malformed" → YAML syntax error in config
- "Parameter validation failed" → Invalid parameter values in config

### Sequential Node Startup (Full Flight Stack)

```
[00:00] Starting imu_bno085 → Waits for sensor to initialize (~200ms)
[00:00] Starting gps_zoe_m8q → Waits for GPS lock (~5-30s depending on location)
[00:00] Starting rc_control → Reads RC receiver GPIO (instant)
[00:02] Starting sensor_fusion → Begins reading IMU/GPS topics (2s delay allows sensors ready)
[00:03] Starting servo_control → Ready to receive RC/mode updates (1s after rc_control)
[00:05+] SYSTEM READY FOR FLIGHT → Ready for autonomous control
```

### Logs

All outputs captured in `<log_file_path>/latest_launch.log`; stream via `GET /logs/stream`

### Error Handling

If a node crashes immediately after being started, the entire system may still be running. Check `/logs/stream` to see which node failed.

---

## POST /launch/stop

Stops the ROS2 system and all running nodes. Must have successfully called `/launch/start` first.

**Prerequisite:** Valid process ID from `/launch/start` response.

### Request Body

Empty POST
```json
{}
```

### Response

**200 OK**: 
```json
{
  "msg": "OK"
}
```

**404 Not Found**: No launch process running (either never started or already stopped)
```json
{
  "detail": "No launch process is currently running",
  "status": 404
}
```

**500 Internal Server Error**: Failed to terminate process (rare; usually filesystem permissions)

### Cleanup Behavior

When `/launch/stop` is called:
1. Sends SIGTERM to all ROS2 nodes (wait for graceful shutdown)
2. Waits 5 seconds for nodes to terminate
3. Sends SIGKILL if nodes don't terminate (forceful kill)
4. **Stops any active recording** (launched via `/launch/start_recording`)
5. Closes all topic subscriptions
6. Cleans up ROS2 daemon processes

---

## POST /launch/start_recording

Records ROS2 messages from specified topics to a rosbag2 file. Useful for post-flight analysis: examine sensor readings, debug estimation errors, retrain state estimator. **Must be called after `/launch/start`** (nodes must be running).

### Use Case

After UAV lands, frontend asks technician which data to save. Technician selects "Save odometry + IMU for post-flight analysis" which records fusion/odom and raw imu_bno085/data.

### Request Body

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

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `topics` | string[] | Yes | Array of topic names to record |

### Response

**200 OK**:
```json
{
  "msg": "OK",
  "bag_path": "/home/uav-user/.ros/records/uav_nav_2026-02-11_14-32-05",
  "topics": 4,
  "recording": true
}
```

| Field | Type | Description |
|-------|------|-------------|
| `msg` | string | "OK" on success |
| `bag_path` | string | Path to rosbag2 directory (can transfer/analyze later) |
| `topics` | integer | Number of topics being recorded |
| `recording` | boolean | Always `true` if 200 OK |

**404 Not Found**: No launch process running (start the system first)

**400 Bad Request**: Topic does not exist or not being published

### Recording Details

- **Location:** Stored in `<rec_dir>` (configured in `api_bridge.yaml`, default: `~/.ros/records/`)
- **Naming:** Automatic timestamp: `uav_nav_YYYY-MM-DD_HH-MM-SS`
- **File Format:** ROS2 rosbag2 (SQLite-based; inspect with `ros2 bag info`, `ros2 bag play`)

### Size Estimates

| Topic | Frequency | Size |
|-------|-----------|------|
| IMU data | 100 Hz | ~50 KB/min |
| GPS fix | 10 Hz | ~5 KB/min |
| Odometry | 10 Hz | ~8 KB/min |
| **Total** | mixed | **~63 KB/min** |

Example: 1-hour flight = 3.8 MB (plus ~50% for metadata/indexes)

### Stop Recording

Call `/launch/stop` to automatically stop active recording

---

## Workflow Example: Full Flight Lifecycle

```javascript
const API_BASE = "http://localhost:8000";

// 1. Create launch configuration with all nodes
const launchResp = await fetch(`${API_BASE}/launch/create`, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    launch: [
      { pkg: "sensors", exec: "imu_bno085", id: "bno085_node" },
      { pkg: "sensors", exec: "gps_zoe_m8q", id: "zoe_m8q_node" },
      { pkg: "sensor_fusion", exec: "sensor_fusion", id: "fusion_node" },
      { pkg: "rc_control", exec: "rc_control", id: "rc_control" },
      { pkg: "servo_control", exec: "servo_control", id: "servo_control" }
    ]
  })
});
const { created } = await launchResp.json();
console.log("Launch file created:", created);

// 2. Start system
const startResp = await fetch(`${API_BASE}/launch/start`, { method: "POST" });
const { pid, msg } = await startResp.json();
console.log("System started with PID:", pid);

// 3. Wait for initialization
await new Promise(resolve => setTimeout(resolve, 5000));

// 4. Start recording specific topics
const recordResp = await fetch(`${API_BASE}/launch/start_recording`, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    topics: ["fusion/odom", "imu_bno085/data", "gps_zoe_m8q/fix", "rc/channels"]
  })
});
const { bag_path } = await recordResp.json();
console.log("Recording flight data to:", bag_path);

// 5. [Flight happens here - keep connection alive]

// 6. Stop everything
const stopResp = await fetch(`${API_BASE}/launch/stop`, { method: "POST" });
console.log("System stopped, recording saved");
```

---

**Next:** [Logging](logs.md)