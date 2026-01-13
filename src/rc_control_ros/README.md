# RC Control ROS Package

This package reads RC receiver signals via GPIO and publishes them to ROS2 topics.

## Nodes: `rc_reader_node`

### Topics Published
- `rc/channels` (`std_msgs/Int32MultiArray`): [Ch1, Ch2, Ch3, Ch4, Ch5]
- `rc/mode` (`std_msgs/String`): "MANUAL" or "AUTO" based on Channel 6.

### Parameters
- `gpiochip` (int, default: 4): The GPIO chip number.
- `publish_rate` (float, default: 50.0):Hz.
- `gpio_pins` (int[], default: `[26, 19, 13, 6, 5, 12]`): 
  - Pins for Channels 1-6 respectively.
  - Channel 6 is used for Mode Switching.

## Launch
```bash
ros2 launch rc_control_ros rc_control.launch.py
```
