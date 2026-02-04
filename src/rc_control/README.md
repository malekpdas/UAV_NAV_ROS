# rc_control

This package reads RC receiver signals via GPIO interrupts and publishes them as ROS2 topics.

## Nodes

### [rc_control](descriptor/rc_control.yaml)

Reads 6 channels (Pulse Width) from an RC receiver and publishes to `rc/channels` and `rc/mode`.

## Configuration

- `config/rc_control_config.yaml`: GPIO pin definitions and publishing rate.

## Launch

- `launch/rc_control.launch.py`: Launches the RC reader node.
