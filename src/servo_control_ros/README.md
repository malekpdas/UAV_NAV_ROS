# servo_control_ros

This package controls servos and ESCs via PWM signals using the `lgpio` library.

## Nodes

### [servo_controller_node](descriptor/servo_controller_node.yaml)

Subscribes to RC channels and flight mode, then maps them to GPIO pins for PWM control.

## Configuration

- `config/servo_params.yaml`: Pin mapping, PWM frequency, and safety limits.

## Launch

- `launch/servo_control.launch.py`: Launches the servo controller node.
