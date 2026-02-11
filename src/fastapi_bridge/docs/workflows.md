# Real-World Workflows

Before diving into endpoint details, here's how the FastAPI Bridge APIs work together in practice.

## Workflow 1: Pre-Flight Sensor Verification

Your frontend needs to verify sensors are configurable before takeoff.

**Steps:**

1. **Discover all nodes** → `GET /nodes/get_all_nodes`
2. **Read default sensor configs** → `POST /config/read` with `default: true` for each sensor
3. **Display to technician** → Show calibration values, sampling rates, variance estimates
4. **Modify if necessary** → `POST /config/create` with updated parameters
5. **Build sensor launch** → `POST /launch/create` with sensor nodes
6. **Start verification** → `POST /launch/start`
7. **Stream logs** → `GET /logs/stream` in separate connection
8. **Verify sensors** → Check logs for "initialized" messages and sensor data publishing
9. **Stop & proceed** → `POST /launch/stop`

**Expected Duration:** 5-10 minutes

**Typical Log Output:**
```
[0.478] BNO085 IMU initialized, publishing to /imu_bno085/data @ 100 Hz
[0.612] GPS receiver powered on... acquiring satellites
[4.567] GPS acquired lock! 12/24 satellites, HDOP: 1.8m
```

---

## Workflow 2: Full Flight Launch

Complete system launch with all 6 nodes before autonomous flight.

**Steps:**

1. **Get all nodes** → `GET /nodes/get_all_nodes`
2. **Read all configs** → `POST /config/read` for sensors, fusion, RC, servo
3. **Review calibrations** → Verify IMU calibration and GPS variance match environment
4. **Create custom configs** → `POST /config/create` if modifications needed
5. **Build full launch** → `POST /launch/create` with all 6 nodes
6. **Start system** → `POST /launch/start`
7. **Monitor startup** → `GET /logs/stream` until "SYSTEM READY FOR FLIGHT"
8. **Start recording** → `POST /launch/start_recording` with sensor + odometry topics
9. **Return to user** → Display "Ready for takeoff" message
10. **After landing** → `POST /launch/stop` (automatically stops recording)

**Node Startup Order & Timing:**
```
[00:00] IMU sensor starts
[00:00] GPS sensor starts
[00:00] RC control starts
[00:02] Sensor Fusion starts (2s delay for sensors to initialize)
[00:03] Servo Control starts (1s delay for RC to initialize)
[00:05] System ready for autonomous flight
```

**Expected Duration:** 10-15 minutes pre-flight check + flight duration + 5 min post-flight

---

## Workflow 3: Emergency Recalibration Mid-Session

Magnetometer calibration error detected during flight or before next flight.

**Steps:**

1. **Land UAV** → Physically bring UAV down
2. **Stop system** → `POST /launch/stop` (kills all nodes immediately)
3. **Read mag config** → `POST /config/read` for `imu_bno085` node
4. **Analyze values** → Check `mag_transform` matrix (soft-iron correction)
5. **Create new calibration** → `POST /config/create` with updated `mag_transform` and `mag_decl`
   - Run calibration utility separately if full recalibration needed
6. **Restart system** → `POST /launch/create` with updated nodes → `POST /launch/start`
7. **Verify fix** → Stream logs to confirm "Mag calibration applied"
8. **Resume flights** → System now uses new calibration

**Calibration Hints:**
- Magnetic declination changes by location (UAV project uses -8.03° for Japan)
- Soft-iron correction is a 3x3 matrix learned during ground calibration spin test
- If magnitude drifts, variance may need adjustment instead (see Troubleshooting)

---

## Workflow 4: Debug Sensor Noise Issue

Fusion odometry producing noisy/drifting estimates midway through flights.

**Steps:**

1. **Capture data** → `POST /launch/start_recording` with fusion/odom + all sensor topics
2. **Fly brief test** → 5-10 minute flight to gather data
3. **Stop & analyze** → `POST /launch/stop`
4. **Download bag** → Transfer rosbag2 from `<rec_dir>` to development machine
5. **Inspect topics** → `ros2 bag info my_flight` + `ros2 topic echo` for patterns
6. **Identify root cause:**
   - Noisy IMU? Increase `ahrs.accel_rejection` or check for mounting vibration
   - GPS drift? Increase `sensor_variance.pos_std` if outdoors with multipath
   - Slow response? Decrease `initial_pos_uncertainty` to trust sensors more
7. **Update config** → `POST /config/create` with tuning changes
8. **Test fix** → Repeat flight with new config and new recording
9. **Compare bags** → Use rosbag2 analysis tools to validate improvement

**Quick Tuning Reference:**
```json
{
  "ahrs": {
    "accel_rejection": 15.0,  "mag_rejection": 15.0
  },
  "kalman": {
    "initial_pos_uncertainty": 50.0
  },
  "sensor_topics": {
    "gps_pos_topic": "gps_zoe_m8q/fix"
  }
}
```

---

## Workflow 5: Multi-Flight Session with Configuration Tracking

Run multiple flights with different configurations and compare results.

**Steps:**

1. **Flight 1 - Baseline**
   - Start system with default configs → `POST /launch/start`
   - Record flight 1 → `POST /launch/start_recording` to `session_2026_02_11_baseline.bag`
   - Stop → `POST /launch/stop`

2. **Modify Configuration**
   - `POST /config/read` to get current IMU settings
   - `POST /config/create` to modify `rate_hz` from 100 to 200
   - `POST /config/create` to modify GPS variance
   
3. **Flight 2 - Modified Settings**
   - `POST /launch/create` with same 6 nodes
   - `POST /launch/start` (picks up new config via `/launch/create`)
   - Record flight 2 → `POST /launch/start_recording` to `session_2026_02_11_modified.bag`
   - Stop → `POST /launch/stop`

4. **Analysis**
   - Compare rosbags: `ros2 bag play session_2026_02_11_baseline.bag` vs `modified.bag`
   - Measure drift, noise, initialization time for each config
   - Validate that modified config improves desired metric

---

## Workflow 6: System Diagnostics & Status Check

Daily system health check before operational flights.

**Steps:**

1. **Minimal startup** → `POST /launch/create` with only sensor nodes
2. **Monitor logs** → `GET /logs/stream` for 30 seconds
3. **Verify output:**
   - All 3 sensors initialize without errors?
   - Sampling rates match config (100 Hz IMU, 10 Hz GPS)?
   - No "permission denied" or "device not found" errors?
4. **Stop minimal test** → `POST /launch/stop`
5. **Check log file** → No warnings or critical errors?
6. **Proceed to full system** if all checks pass

**Expected Clean Logs:**
```
[0.612] BNO085 IMU initialized, publishing to /imu_bno085/data @ 100 Hz
[0.589] GPS receiver powering on... acquiring satellites
[0.901] RC control reading GPIO pins [20, 25, 16, 12, 24, 21]
```

**Warning Signs:**
- "permission denied /dev/ttyUSB0" → USB/UART not accessible
- "Port /dev/ttyUSB0 not found" → GPS not connected
- "GPIO chip 4 not found" → RC control hardware issue

---

## Decision Tree: Which Workflow?

```
Are you launching for the first time?
  ├─ YES → Use Workflow 1 (Pre-Flight Verification)
  └─ NO → Continue...

Do you have new hardware or recalibrated sensors?
  ├─ YES → Use Workflow 1
  └─ NO → Continue...

Are you doing autonomous flight?
  ├─ YES → Use Workflow 2 (Full Flight Launch)
  └─ NO → Use Workflow 6 (Diagnostics)

Did you detect a problem during last flight?
  ├─ Noisy odometry? → Use Workflow 4 (Debug Noise)
  ├─ Bad compass heading? → Use Workflow 3 (Recalibration)
  └─ Other? → Use Workflow 6 then investigate
```

---

## Tips for Frontend Implementation

1. **Pre-flight checklist UI:**
   - List all 9 nodes with status (discoverable/missing)
   - Show config file paths for user verification
   - Allow one-click config reset to factory defaults

2. **System startup progress:**
   - Show phase-based progress (sensors → fusion → control)
   - Stream `/logs/stream` to terminal widget in real-time
   - Alert when "SYSTEM READY" found in logs

3. **Recording management:**
   - Show list of available topics from `/nodes/get_all_nodes`
   - Let user select topics before launching
   - Display recording status (duration, file size estimate)
   - Provide download link when stopped

4. **Error recovery:**
   - If launch fails, show last few lines of log stream
   - Provide "Diagnose" button that runs Workflow 6
   - Allow user to roll-back config via `/config/read` with `default: true`

---

**Next:** Review [API Endpoints](endpoints/) for detailed request/response examples, or see [Integration Guide](integration-examples.md) for code samples.
