# rc_control_ros

This package reads RC receiver signals via GPIO interrupts and publishes them as ROS2 topics.

## Nodes

### [rc_reader_node](descriptor/rc_reader_node.yaml)

Reads 6 channels (Pulse Width) from an RC receiver and publishes to `rc/channels` and `rc/mode`.

## Configuration

- `config/rc_params.yaml`: GPIO pin definitions and publishing rate.

## Launch

- `launch/rc_control.launch.py`: Launches the RC reader node.
