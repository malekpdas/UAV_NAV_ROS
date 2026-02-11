# Getting Started with FastAPI Bridge

Welcome! This guide covers basic setup and connection to the FastAPI Bridge API.

## Base URL & Configuration

### Default Endpoint

```
http://0.0.0.0:8000
```

Access the API from any machine on your network by replacing `0.0.0.0` with the actual IP address of the ROS2 system.

### Configuration Parameters

The API behavior is configured via `src/fastapi_bridge/config/api_bridge.yaml`:

```yaml
api_host: "0.0.0.0"           # Listen on all interfaces (0.0.0.0) or specific IP
api_port: 8000                # API server port
ws_root: "/path/to/workspace" # ROS2 workspace root (usually auto-detected)
log_file_path: "~/.ros/log"   # Where ROS2 launch logs are written
launch_dir: "~/.ros/launch"   # Where generated launch.yaml is saved
rec_dir: "~/.ros/records"     # Where rosbag2 recordings are saved
```

To change these, edit the config file and restart the `fastapi_bridge` node.

## Quick Connection Examples

### Python (requests)

```python
import requests

BASE_URL = "http://localhost:8000"

# Discover nodes
response = requests.get(f"{BASE_URL}/nodes/get_all_nodes")
nodes = response.json()
print(f"Found {len(nodes)} nodes")
```

### JavaScript (Fetch API)

```javascript
const BASE_URL = "http://localhost:8000";

async function getAvailableNodes() {
  const response = await fetch(`${BASE_URL}/nodes/get_all_nodes`);
  const nodes = await response.json();
  console.log(`Found ${nodes.length} nodes`);
  return nodes;
}
```

### cURL

```bash
curl http://localhost:8000/nodes/get_all_nodes | jq '.'
```

## Error Response Format

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

## Next Steps

- [Explore Real-World Workflows](workflows.md)
- [Read Endpoint Documentation](endpoints/)
- [Integrate with Your Frontend](integration-examples.md)
