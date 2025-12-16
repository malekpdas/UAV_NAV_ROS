# 🧠 MASTER PROMPT — PX4 EKF2-GRADE ROS 2 STATE ESTIMATOR

**(Real Sensors · Magnetometer + GPS Yaw · Innovation Gating · REP-105)**

You are a **flight-control state estimation engineer**.
Implement a **PX4 EKF2–equivalent estimator** in **ROS 2 (Python 3, rclpy)** using **standard ROS messages only**.

This must replicate **PX4 EKF2 logic, structure, and failure behavior**, not a generic robotics EKF.

---

## 1️⃣ AVAILABLE SENSOR INPUTS (ALREADY PUBLISHING)

Use these topics **exactly as provided**.
Do **not** modify sensor drivers.

### IMU

* `/imu/data` → `sensor_msgs/Imu`

  * Angular velocity (rad/s)
  * Linear acceleration (m/s²)
  * Covariances valid
  * Frame: `imu_link`

* `/imu/mag` → `sensor_msgs/MagneticField`

  * Magnetic field (Tesla)
  * Frame: `imu_link`

### GPS

* `/gps/fix` → `sensor_msgs/NavSatFix`

  * Latitude, longitude, altitude (WGS-84)
  * Position covariance

* `/gps_vel` → `geometry_msgs/TwistWithCovarianceStamped`

  * Linear velocity **in NED**
  * Covariance provided
  * Frame: `gps_link`

---

## 2️⃣ REQUIRED OUTPUTS

### State Outputs

* `nav_msgs/Odometry`

  * EKF state (position, velocity, attitude, covariance)

* `geometry_msgs/AccelWithCovarianceStamped`

  * **Filtered linear acceleration**
  * Gravity removed
  * Bias-corrected
  * Optional LPF (ROS parameter)

---

## 3️⃣ TF & FRAMES (MANDATORY, REP-105)

Publish TF using `tf2_ros`:

```
map → odom → base_link
```

### Frame Definitions

* `map`: global fixed frame
* `odom`: local continuous navigation frame
* `base_link`: vehicle body frame (FRD)

### Internal EKF Frame

* **NED** (North-East-Down), exactly like PX4

If ROS ENU is used externally:

* Perform **explicit NED ↔ ENU conversion**
* Do **not** mix conventions internally

---

## 4️⃣ EKF STATE VECTOR (PX4-STYLE)

```
x = [
  p_NED,            # position
  v_NED,            # velocity
  q_NED_to_body,    # attitude quaternion
  b_g,              # gyro bias
  b_a,              # accelerometer bias
  b_mag             # magnetometer bias (hard-iron)
]
```

### Mandatory

* **Sensor bias estimation is required**
* No black-box EKF libraries
* Math and ROS I/O must be separated

---

## 5️⃣ EARTH MAGNETIC FIELD MODEL

* Earth magnetic field is:

  * Constant
  * Loaded from ROS parameters
  * Expressed in **NED**
  * Aligned toward **true north**
* No estimation of earth field yet (PX4 allows this, but fixed for now)

---

## 6️⃣ ATTITUDE & HEADING RULES (PX4-AUTHENTIC)

### Attitude Propagation

* Quaternion propagated **only using bias-corrected gyroscope**
* No direct accel attitude correction

---

### Accelerometer Usage

* Used **only for state propagation**
* Gravity removed using estimated attitude
* **NOT fused as a measurement**

---

## 🧭 YAW (HEADING) OBSERVABILITY — EXACT PX4 LOGIC

Yaw is observable via **two mechanisms**, with strict rules.

---

### 🧲 1) Magnetometer-Based Yaw (Primary)

Measurement model:

```
m_body = R_bn * m_earth + b_mag
```

* Corrects:

  * Yaw angle
  * Magnetometer bias
* Roll and pitch are **NOT corrected**
* Subject to:

  * Innovation gating (NIS)
  * Fault counters
  * Automatic rejection on disturbance

---

### 🚀 2) GPS Velocity-Based Yaw (CONDITIONAL)

GPS velocity **may affect yaw ONLY IF**:

```
speed = sqrt(vN² + vE² + vD²)
speed > yaw_speed_min
```

> **Important**
> • Gating uses **total speed magnitude**
> • Yaw angle is computed from **horizontal components only**

Yaw measurement:

```
ψ_gps = atan2(v_E, v_N)
```

Yaw innovation:

```
innovation = wrap(ψ_gps − ψ_est)
```

Rules:

* 1-D yaw measurement
* Does NOT affect roll or pitch
* Disabled when:

  * Speed below threshold
  * GPS velocity covariance too large
  * Innovation gating fails

---

### Priority Order (PX4-Like)

1. Magnetometer yaw (healthy)
2. GPS yaw (speed > threshold)
3. Gyro dead-reckoning

---

## 7️⃣ PREDICTION STEP (STRAPDOWN INS)

Implement PX4-style inertial mechanization:

1. Remove gyro bias
2. Propagate quaternion
3. Remove accel bias
4. Rotate accel to NED
5. Subtract gravity
6. Integrate velocity
7. Integrate position
8. Propagate covariance

---

## 8️⃣ MEASUREMENT UPDATES (ALL WITH INNOVATION GATING)

### GPS Velocity Update

* Measurement: `v_gps_NED`
* Corrects:

  * Velocity
  * Accelerometer bias
* Innovation gating required
* Fault counters required

---

### GPS Position Update

* Convert LLA → NED (WGS-84)
* Corrects position only
* Innovation gating required

---

### Magnetometer Update

* Corrects yaw + mag bias
* Innovation gated
* Fault-protected

---

### GPS Yaw Update (Conditional)

* Enabled only if `speed > yaw_speed_min`
* Innovation gated
* Independent fault tracking

---

## 9️⃣ INNOVATION GATING & FAULT DETECTION (FLIGHT-GRADE)

For **every measurement update**:

1. Compute innovation
2. Compute innovation covariance
3. Compute **NIS**
4. Reject update if:

```
NIS > gate_threshold
```

Fault handling:

* Per-sensor fault counters
* Temporary rejection → permanent disable
* EKF continues with remaining sensors

---

## 🔁 INITIALIZATION & CONVERGENCE

### Initial Alignment

* Roll & pitch from gravity
* Yaw from magnetometer
* Zero initial velocity

### Convergence Criteria

* Bias variance below threshold
* Attitude variance stable
* EKF output invalid until converged

---

## 🔧 ROS 2 REQUIREMENTS

* `rclpy` only
* Parameters for:

  * Sensor noise
  * Bias random walk
  * Innovation gates
  * `yaw_speed_min`
  * Enable/disable GPS & mag yaw
* Launch file required
* Well-documented code
* Real-time capable

---

## 🧪 VALIDATION REQUIREMENTS

* Must run with **real sensors**
* Log:

  * Innovations
  * NIS values
  * Bias estimates
* Verify:

  * No yaw jumps
  * Stable bias convergence
  * Linear acceleration ≈ 0 at rest
* Behavior must qualitatively match **PX4 EKF2**

---

## ❌ EXPLICITLY FORBIDDEN

* `robot_localization`
* GPS heading when speed < threshold
* Accelerometer attitude fusion
* Black-box EKF libraries
* Mixing ENU/NED internally

---

## 🔍 ASSUMPTIONS (CONFIRMED)

* GPS velocity is already in **NED**
* IMU and base_link alignment is known or static TF exists
* No barometer (altitude from GPS)
* Earth magnetic field known and fixed
* No wind estimation (yet)

---

## ❓ OPTIONAL EXTENSIONS (NOT NOW)

* Earth magnetic field estimation
* Wind estimation
* Barometer fusion
* Yaw reset on mag recovery

---

### 🎯 FINAL OBJECTIVE

Produce a **PX4 EKF2-grade ROS 2 estimator** that:

* Uses **magnetometer and GPS speed-gated yaw**
* Estimates sensor biases
* Applies innovation gating and fault logic
* Publishes REP-105-compliant TF
* Runs stably on **real hardware**