# Config - Parameter Management

Manage ROS2 node configuration files (YAML parameters).

## POST /config/read

Reads a configuration file for a specific node. It prioritizes a custom config in the launch directory; if not found (or if `default` is true), it falls back to the package's default config.

### Use Cases

**Use Case 1 - View Factory Settings:** Technician logs in before flight and wants to see what the factory calibration is for the BNO085 IMU sensor.

**Use Case 2 - Check for Custom Config:** Flight controller wants to load the previous flight's configuration to verify settings haven't changed.

### Request Body

```json
{
  "id": "bno085_node",
  "exec": "imu_bno085",
  "pkg": "sensors",
  "default": true
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | Yes | Node instance ID (get from `/nodes/get_all_nodes`) |
| `exec` | string | Yes | Executable name (e.g., "imu_bno085") |
| `pkg` | string | Yes | Package name (e.g., "sensors") |
| `default` | boolean | Yes | `true` = force factory default, `false` = use custom if exists |

### Real Response - BNO085 IMU (factory default config)

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

### Real Response - GPS ZOE M8Q (factory config)

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

### Real Response - Sensor Fusion (factory config)

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

### Response Fields

**200 OK**: Returns config object with metadata
- `config`: Actual parameters (structure depends on node type)
- `msg`: "OK" on success
- `default`: Boolean indicating if this is the factory default (true) or custom config (false)

**500 Internal Server Error**: If config file unreadable (permissions, missing file)

### Important Notes

- If `default: false` but no custom config exists, API automatically returns default config and sets `"default": true` in response
- This endpoint is **read-only**; use `/config/create` to modify

---

## POST /config/create

Creates or overwrites a configuration file for a specific node in the launch configuration directory. This custom config will be used for the next system launch instead of the default.

### Use Case

Technician landed after 20-minute flight and noticed drift. Wants to increase IMU sampling rate from 100 Hz to 200 Hz and increase GPS variance estimates for noisier readings.

### Request Body

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

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | Yes | Node instance ID |
| `config` | object | Yes | Parameter object (structure matches output from `/config/read`) |

### Real Example - Modifying GPS Config for Noisy Conditions

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

### Response

**200 OK**:
```json
{
  "created": "/home/uav-user/.ros/launch/config/bno085_node.yaml",
  "msg": "OK"
}
```

**500 Internal Server Error**: If `launch_dir/config/` not writable (filesystem permissions issue)

### Important Notes

1. **Storage Location:** Custom config is stored at `<launch_dir>/config/<node_id>.yaml`
2. **When Applied:** Custom config only takes effect on **next launch** (not retroactive to running nodes)
3. **Invalid Parameters:** Silently ignored by ROS2; ensure all parameters match expected types
4. **Reset to Factory:** To reset to factory defaults:
   - Delete the custom config file at `<launch_dir>/config/<node_id>.yaml`, OR
   - Call `/config/read` with `default: true`, OR
   - Use the API to explicitly set all values back to factory defaults

---

## Workflow Example: Update & Verify Config

```javascript
// 1. Read current config
const response = await fetch(`${API_BASE}/config/read`, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    id: "bno085_node",
    exec: "imu_bno085",
    pkg: "sensors",
    default: false  // Get custom config if exists, else factory
  })
});
const { config, default: isFactory } = await response.json();

// 2. Show current values to user
console.log("Current rate:", config.rate_hz);
console.log("Using factory defaults:", isFactory);

// 3. Modify and save
const newConfig = {
  ...config,
  rate_hz: 200  // Increase sampling rate
};

const createResponse = await fetch(`${API_BASE}/config/create`, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    id: "bno085_node",
    config: newConfig
  })
});
const { created } = await createResponse.json();
console.log("Config saved to:", created);
```

---

**Next:** [System Control](launch.md)