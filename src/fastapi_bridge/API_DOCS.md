# FastAPI Bridge API Documentation

This document provides detailed information about the API endpoints exposed by the `fastapi_bridge` ROS2 package.

## Base URL

By default, the API is served at: `http://0.0.0.0:8000`

The host and port can be configured via ROS2 parameters `api_host` and `api_port`.

## Endpoints

### Nodes

Manage and retrieve information about ROS2 nodes.

#### `GET /nodes/get_all_nodes`

Returns a list of node descriptions found in the workspace. Each description includes package details, node executable information, topics, services, actions, and the absolute path to the default configuration file.

**Response:**
- `200 OK`: JSON array containing node descriptor objects.

**Example Response:**
```json
[
  {
    "pkg_name": "math_operators",
    "executable_name": "add_operator",
    "node_default_id": "add_operator",
    "default_config": "[ROS2-WS]/src/package_name/config/node_name.yaml",
    "sub_topics": [
      {
        "name": "number1",
        "type": "std_msgs/msg/Int32"
      }
    ],
    "pub_topics": [
      {
        "name": "result",
        "type": "std_msgs/msg/Float64"
      }
    ],
    "services": [],
    "actions": []
  }
]
```

---

### Logs

Access ROS2 launch logs.

#### `GET /logs/stream`

Streams the content of the latest ROS2 launch log file in real-time. This endpoint uses Server-Sent Events (like behavior) to stream text data.

**Response:**
- `200 OK`: Stream of text/plain data.
- Headers: `Cache-Control: no-cache`

**Notes:**
- If the log file does not exist, it returns a message indicating so.
- Changes to the log file are streamed as they happen (tail logic).

---

### Launch

Control ROS2 launch files.

#### `POST /launch/start`

Starts the ROS2 system using the generated `launch.yaml` file.

**Response:**
- `200 OK`: 
  ```json
  {
    "msg": "OK", 
    "pid": 12345
  }
  ```
- `500 Internal Server Error`: If launch fails.

#### `POST /launch/stop`

Stops the currently running ROS2 system initiated by `/launch/start`.

**Response:**
- `200 OK`: `{"msg": "OK"}`
- `404 Not Found`: If no launch process is currently running.
- `500 Internal Server Error`: If stopping the process fails.

#### `POST /launch/create`

Creates a new `launch.yaml` file based on the provided configuration.

**Request Body:** `LaunchFileBody`
```json
{
  "launch": [
    {
      "pkg": "package_name",
      "exec": "executable_name",
      "id": "node_name"
    }
  ]
}
```

**Response:**
- `200 OK`:
  ```json
  {
    "created": "/path/to/launch.yaml",
    "msg": "OK"
  }
  ```
- `500 Internal Server Error`: If file creation fails.

---

### Config

Manage ROS2 node configuration files (YAML parameters).

#### `POST /config/read`

Reads a configuration file for a specific node. It prioritizes a custom config in the launch directory; if not found (or if `default` is true), it falls back to the package's default config.

**Request Body:** `ConfigReadBody`
```json
{
  "id": "node_name",
  "exec": "executable_name",
  "pkg": "package_name",
  "default": false
}
```

**Response:**
- `200 OK`:
  ```json
  {
    "config": { ... },
    "msg": "OK",
    "default": true/false
  }
  ```
- `500 Internal Server Error`: If reading fails.

#### `POST /config/create`

Creates or overwrites a configuration file for a specific node in the launch configuration directory.

**Request Body:** `ConfigFileBody`
```json
{
  "id": "node_name",
  "config": {
    "param1": "value1",
    "param2": 123
  }
}
```

**Response:**
- `200 OK`:
  ```json
  {
    "created": "/path/to/config/node_name.yaml",
    "msg": "OK"
  }
  ```
- `500 Internal Server Error`: If file creation fails.
