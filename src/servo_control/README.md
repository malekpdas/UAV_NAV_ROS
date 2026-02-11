# servo_control

This package controls servos and ESCs via PWM signals using the `lgpio` library.

## Nodes

### [servo_controller](descriptor/servo_controller.yaml)

Subscribes to RC channels and flight mode, then maps them to GPIO pins for PWM control.

## Configuration

- `config/servo_control.yaml`: Pin mapping, PWM frequency, and safety limits.

## Launch

- `launch/servo_control.launch.yaml`: Launches the servo controller node.
