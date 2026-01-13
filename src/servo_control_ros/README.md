# Servo Control ROS Package

This package controls Servos and ESCs based on inputs from RC or Flight Controller commands.

## Nodes: `servo_controller_node`

### Topics Subscribed
- `rc/channels` (`std_msgs/Int32MultiArray`): Manual RC input.
- `rc/mode` (`std_msgs/String`): Mode switch.

### Functionality
- **MANUAL Mode**: Direct pass-through of mapped RC channels to outputs.
- **Arming**: On startup, sends 1000us (Low/Off) to the Rotor (ESC) for 5 seconds to arm Dualsky/Generic ESCs.

### Parameters
- `pwm_freq` (int, default: 50): PWM Frequency in Hz.
- **Pin Configuration**:
  - `pin_rotor` (default: 20)
  - `pin_rudder` (default: 22)
  - `pin_elevator` (default: 24)
  - `pin_l_aileron` (default: 23)
  - `pin_r_aileron` (default: 27)
- **Channel Mapping** (Index in `rc/channels`):
  - rotor: 2 (Throttle)
  - rudder: 3 (Yaw)
  - elevator: 1 (Pitch)
  - ailerons: 0 (Roll)

## Launch
```bash
ros2 launch servo_control_ros servo_control.launch.py
```
