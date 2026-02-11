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
    "rate_hz": {
      "value": 100,
      "description": "Sensor update rate in Hz",
      "dtype": "int"
    },
    "frame_id": {
      "value": "imu_link",
      "description": "ROS frame ID for the IMU",
      "dtype": "str"
    },
    "sensor_calibration": {
      "bias_removal": {
        "value": false,
        "description": "Whether to perform bias removal on startup",
        "dtype": "bool"
      },
      "accel_bias": {
        "value": [-1.14582811, -0.01432422, 0.05357129],
        "description": "Accelerometer bias offsets",
        "dtype": "float"
      },
      "gyro_bias": {
        "value": [8.42370863e-05, 6.66060682e-05, -9.59910983e-05],
        "description": "Gyroscope bias offsets",
        "dtype": "float"
      },
      "mag_bias": {
        "value": [4.82532218e-08, -9.12769731e-08, -6.33085141e-08],
        "description": "Magnetometer bias offsets",
        "dtype": "float"
      },
      "mag_transform": {
        "value": [2.31239933e-02, -1.33933075e-04, -1.48083236e-05, -1.33933075e-04, 2.34637194e-02, 1.98589239e-05, -1.48083236e-05, 1.98589239e-05, 2.31128531e-02],
        "description": "Magnetometer soft-iron transformation matrix",
        "dtype": "float"
      }
    },
    "sensor_variance": {
      "accel": {
        "value": [0.01, 0.01, 0.01],
        "description": "Accelerometer variance (x, y, z)",
        "dtype": "float"
      },
      "gyro": {
        "value": [0.001, 0.001, 0.001],
        "description": "Gyroscope variance (x, y, z)",
        "dtype": "float"
      },
      "mag": {
        "value": [0.001, 0.001, 0.001],
        "description": "Magnetometer variance (x, y, z)",
        "dtype": "float"
      }
    },
    "transformation": {
      "imu_rotation": {
        "value": [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
        "description": "Mounting rotation matrix from IMU to body frame",
        "dtype": "float"
      },
      "mag_decl": {
        "value": -8.03,
        "description": "Magnetic declination in degrees",
        "dtype": "float"
      }
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
    "interface_type": {
      "value": "uart",
      "description": "Communication interface (uart or spi)",
      "dtype": "str"
    },
    "rate_hz": {
      "value": 10,
      "description": "Update rate in Hz",
      "dtype": "int"
    },
    "frame_id": {
      "value": "gps_link",
      "description": "ROS frame ID for GPS",
      "dtype": "str"
    },
    "sensor_variance": {
      "pos_std": {
        "value": [4.0, 4.0, 10.0],
        "description": "Position std dev (x, y, z) in meters",
        "dtype": "float"
      },
      "vel_std": {
        "value": [0.5],
        "description": "Velocity std dev in m/s",
        "dtype": "float"
      }
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
      "map_frame": {
        "value": "map",
        "description": "Global map frame name",
        "dtype": "str"
      },
      "odom_frame": {
        "value": "odom",
        "description": "Odometry frame name",
        "dtype": "str"
      },
      "base_link_frame": {
        "value": "base_link",
        "description": "Robot base link frame",
        "dtype": "str"
      }
    },
    "kalman": {
      "initial_pos_uncertainty": {
        "value": 25.0,
        "description": "Initial position uncertainty in meters",
        "dtype": "float"
      },
      "initial_vel_uncertainty": {
        "value": 1.0,
        "description": "Initial velocity uncertainty in m/s",
        "dtype": "float"
      },
      "process_noise": {
        "pos": {
          "value": 0.1,
          "description": "Process noise for position",
          "dtype": "float"
        },
        "vel": {
          "value": 0.01,
          "description": "Process noise for velocity",
          "dtype": "float"
        }
      },
      "measurement_noise": {
        "pos": {
          "value": 25.0,
          "description": "Measurement noise for position",
          "dtype": "float"
        },
        "vel": {
          "value": 0.25,
          "description": "Measurement noise for velocity",
          "dtype": "float"
        }
      }
    },
    "earth": {
      "ref_pos": {
        "value": [34.8761905, 136.9617842, 19.63],
        "description": "Reference position [lat, lon, alt]",
        "dtype": "float"
      }
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
    "rate_hz": {
      "value": 200,
      "description": "Sensor update rate in Hz",
      "dtype": "int"
    },
    "frame_id": {
      "value": "imu_link",
      "description": "ROS frame ID for the IMU",
      "dtype": "str"
    },
    "sensor_calibration": {
      "bias_removal": {
        "value": true,
        "description": "Whether to perform bias removal on startup",
        "dtype": "bool"
      },
      "accel_bias": {
        "value": [-1.14582811, -0.01432422, 0.05357129],
        "description": "Accelerometer bias offsets",
        "dtype": "float"
      },
      "mag_transform": {
        "value": [2.31239933e-02, -1.33933075e-04, -1.48083236e-05, -1.33933075e-04, 2.34637194e-02, 1.98589239e-05, -1.48083236e-05, 1.98589239e-05, 2.31128531e-02],
        "description": "Magnetometer soft-iron transformation matrix",
        "dtype": "float"
      }
    },
    "sensor_variance": {
      "accel": {
        "value": [0.01, 0.01, 0.01],
        "description": "Accelerometer variance (x, y, z)",
        "dtype": "float"
      },
      "gyro": {
        "value": [0.001, 0.001, 0.001],
        "description": "Gyroscope variance (x, y, z)",
        "dtype": "float"
      }
    },
    "transformation": {
      "imu_rotation": {
        "value": [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
        "description": "Mounting rotation matrix from IMU to body frame",
        "dtype": "float"
      },
      "mag_decl": {
        "value": -8.03,
        "description": "Magnetic declination in degrees",
        "dtype": "float"
      }
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
    "interface_type": {
      "value": "uart",
      "description": "Communication interface (uart or spi)",
      "dtype": "str"
    },
    "rate_hz": {
      "value": 10,
      "description": "Update rate in Hz",
      "dtype": "int"
    },
    "frame_id": {
      "value": "gps_link",
      "description": "ROS frame ID for GPS",
      "dtype": "str"
    },
    "sensor_variance": {
      "pos_std": {
        "value": [6.0, 6.0, 15.0],
        "description": "Position std dev (x, y, z) in meters - increased for noisy urban conditions",
        "dtype": "float"
      },
      "vel_std": {
        "value": [1.0],
        "description": "Velocity std dev in m/s - increased tolerance",
        "dtype": "float"
      }
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