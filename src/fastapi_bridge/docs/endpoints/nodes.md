# Nodes - Discovery & Metadata

Discover all available ROS2 nodes and their capabilities.

## GET /nodes/get_all_nodes

**Use Case:** Frontend UI needs to populate a node selection dropdown, show available sensors, and get parameter file paths for each node.

Returns a list of node descriptions found in the workspace. Each description includes package details, node executable information, topics, services, actions, and the absolute path to the default configuration file.

### Response

**200 OK**: JSON array containing node descriptor objects.

### Real Example Response (5 of 9 available nodes)

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

## Field Descriptions

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

## Frontend Integration Example

```javascript
// After fetching nodes, extract metadata for dropdowns
const nodes = await getAvailableNodes();
const sensorNodes = nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length === 0);
const fusionNodes = nodes.filter(n => n.pub_topics.length > 0 && n.sub_topics.length > 0);
const controlNodes = nodes.filter(n => n.sub_topics.length > 0 && n.pub_topics.length === 0);

// Display dependencies
const fusionDeps = fusionNodes[0].sub_topics.map(t => t.name);
console.log("Sensor Fusion requires:", fusionDeps);
// Output: ["imu_bno085/data", "imu_bno085/mag", "gps_zoe_m8q/fix", "gps_zoe_m8q/vel"]
```

## All 9 Available Nodes

The complete list of nodes discoverable via this endpoint:

1. **sensors.imu_bno085** - 9-DOF IMU with magnetometer (100 Hz)
2. **sensors.imu_bmx160** - Alternative 9-DOF IMU (100 Hz)
3. **sensors.gps_zoe_m8q** - Ublox GPS/GNSS receiver (10 Hz)
4. **sensors.lidar_lite_v3hp** - Lidar distance sensor (100 Hz)
5. **sensor_fusion.sensor_fusion** - EKF state estimator (10 Hz)
6. **rc_control.rc_control** - RC receiver interface (50 Hz)
7. **servo_control.servo_control** - Servo/ESC PWM outputs (50 Hz+)
8. **calibration.mag_calibration** - Magnetometer calibration tool
9. **extra_nodes.plot_orientation** - Odometry visualization

---

**Next:** [Configuration Management](config.md)