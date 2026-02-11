# Logs - Real-Time System Monitoring

Stream ROS2 launch logs in real-time for technician dashboards and debugging.

## GET /logs/stream

Streams ROS2 launch logs in real-time. Useful for technician dashboard: display system status, sensor initialization, node startup order, any warnings/errors.

### Use Case

Frontend opens a "System Monitor" view while system is running. User sees live log updates (100-500ms latency) as nodes start and report their status.

### Response

**200 OK**: Server-Sent Events (SSE) stream of text lines
- **Content-Type:** `text/event-stream`
- **Cache-Control:** `no-cache`

**404 Not Found**: Log file doesn't exist yet (no launch has been run)

### Real Example - Full Flight Stack Logs

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

## How to Consume

### JavaScript (EventSource)

```javascript
const eventSource = new EventSource("http://localhost:8000/logs/stream");

eventSource.onmessage = (event) => {
  const logLine = event.data;
  console.log(logLine);
  
  // Append to UI log display
  const logContainer = document.getElementById("system-logs");
  logContainer.innerHTML += logLine + "<br>";
  
  // Auto-scroll to bottom
  logContainer.scrollTop = logContainer.scrollHeight;
};

eventSource.onerror = (event) => {
  console.error("Log stream error:", event);
  eventSource.close();
  
  // Optionally reconnect every 5 seconds
  setTimeout(() => {
    location.reload();  // or re-establish connection
  }, 5000);
};
```

### Python Client

```python
import requests

response = requests.get("http://localhost:8000/logs/stream", stream=True)
for line in response.iter_lines():
    if line:
        decoded_line = line.decode('utf-8')
        print(decoded_line)
```

### React Component Example

```jsx
import { useEffect, useRef } from 'react';

export function SystemLogs() {
  const logsRef = useRef(null);
  
  useEffect(() => {
    const eventSource = new EventSource("http://localhost:8000/logs/stream");
    
    eventSource.onmessage = (event) => {
      if (logsRef.current) {
        const newLog = document.createElement('div');
        newLog.textContent = event.data;
        logsRef.current.appendChild(newLog);
        logsRef.current.scrollTop = logsRef.current.scrollHeight;
      }
    };
    
    return () => eventSource.close();
  }, []);
  
  return (
    <div 
      ref={logsRef} 
      style={{ 
        height: '400px', 
        overflowY: 'auto', 
        fontFamily: 'monospace',
        backgroundColor: '#1e1e1e',
        color: '#00ff00',
        padding: '10px'
      }}
    />
  );
}
```

## Common Log Patterns

### Successful System Startup

Look for these indicators of successful startup:

```
✅ "BNO085 IMU initialized, publishing to /imu_bno085/data"
✅ "GPS acquired lock! 12/24 satellites"
✅ "Sensor Fusion publishing: /fusion/odom"
✅ "Servo Control armed and ready"
✅ "SYSTEM READY FOR FLIGHT"
```

### GPS Acquisition Timeline

```
[T+0s] GPS receiver powering on... acquiring satellites
[T+1s] GPS acquiring lock... 3/24 satellites
[T+2s] GPS acquiring lock... 6/24 satellites
[T+5s] GPS acquired lock! 12/24 satellites, HDOP: 1.8m
```

**Note:** GPS may take 30-60 seconds on first startup (cold start). Subsequent power-ups are faster if warm.

### Error Conditions

```
❌ "permission denied /dev/ttyUSB0" → User doesn't have UART access
❌ "Port /dev/ttyUSB0 not found" → GPS hardware not connected
❌ "GPIO chip 4 not found" → RC control hardware issue
❌ "Sensor fusion failed to initialize" → Config parameters invalid
```

## Monitoring Tips

### Real-Time Status Dashboard

Create a frontend dashboard that:
1. Displays system logs in scrollable terminal widget
2. Shows node status indicators (initialized, running, crashed)
3. Highlights errors in red, warnings in yellow
4. Shows GPS satellite count and signal strength
5. Displays which nodes are publishing successfully

### Log-Based Triggers

Use log keywords to trigger frontend actions:

| Log Contains | Meaning | Frontend Action |
|--------------|---------|-----------------|
| "READY FOR FLIGHT" | All systems initialized | Enable "Takeoff" button |
| "permission denied" | Hardware access issue | Show error modal |
| "GPS acquiring lock" | Waiting for GPS | Show GPS status indicator |
| "GPS acquired lock" | GPS ready | Mark GPS as ready |
| "Sensor Fusion publishing" | EKF initialized | Enable autonomy mode |

### Sampling Rates

Verify expected sampling rates in logs:

| Node | Expected Rate | Log Pattern |
|------|---------------|-------------|
| IMU | 100 Hz | "publishing to /imu_bno085/data @ 100 Hz" |
| GPS | 10 Hz | "publishing to /gps_zoe_m8q/fix @ 10 Hz" |
| Fusion | 10 Hz | "publishing /fusion/odom @ 10 Hz" |
| RC | 50 Hz | "RC control publishing @ 50 Hz" |

## Stream Connection Lifecycle

### Normal Operation
```
1. Frontend opens SSE connection
2. System starts via /launch/start
3. Logs stream in real-time (100-500ms latency)
4. User monitors logs during flight ops
5. System stops via /launch/stop
6. Stream terminates gracefully
7. Frontend closes connection
```

### Handling Disconnects

The SSE connection is long-lived and may disconnect due to:
- Network interruption
- Server restart
- Client timeout after 30+ minutes inactivity

**Recommended frontend practices:**
- Auto-reconnect if connection drops
- Store last N lines of logs in memory
- Show visual indicator when stream is active/inactive
- Display "Last log: X seconds ago" if stream stales

## Response Codes

| Code | Meaning | Action |
|------|---------|--------|
| 200 | Stream active | Keep connection open |
| 404 | Log file missing | No launch has run yet |

## Notes

- **Stream terminates** when `/launch/stop` is called
- **Typical latency:** 100-500ms from log write to client display
- **Long-lived connection:** Frontend should handle disconnects and reconnect automatically
- **No message history:** Connecting after logs have been written won't show past logs (use rosbag2 for post-flight playback)
- **Log file location:** Configured via `log_file_path` in `api_bridge.yaml` (default: `~/.ros/log/latest_launch.log`)

---

**Next:** [Troubleshooting](../troubleshooting.md) or [Parameters Reference](../parameters-reference.md)