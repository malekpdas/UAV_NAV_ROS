# Custom ROS 2 Message Setup Guide

## Overview
This guide demonstrates how to create and use a custom ROS 2 message (`TelemetryArray`) in ROS 2 Jazzy.

---

## 1. Message Definition

### File: `src/custom_interfaces/msg/TelemetryArray.msg`

```
std_msgs/Header header
int32[] data
string metadata
```

**Field Descriptions:**
- `header` (std_msgs/Header): Timestamp and frame ID for the message
- `data` (int32[]): Dynamic array of integer values (sensor readings)
- `metadata` (string): Optional comment/label (e.g., sensor type, batch info)

---

## 2. Package Configuration - `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_interfaces</name>
  <version>0.0.1</version>
  <description>Custom message definitions for UAV navigation</description>
  <maintainer email="uav@example.com">UAV Developer</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Message generation dependencies -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- Standard message dependencies -->
  <build_depend>std_msgs</build_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
</package>
```

**Key Dependencies:**
- `rosidl_default_generators`: Generates C++ and Python interfaces from .msg files
- `std_msgs`: Required for using stdHeader in your custom message

---

## 3. Build Configuration - `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(custom_interfaces)

# Find build dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define custom message files
set(msg_files
  "msg/RcChannels.msg"
  "msg/FlightMode.msg"
)

# Generate message code
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# Export dependencies for downstream packages
ament_export_dependencies(
  std_msgs
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_files()
endif()

ament_package()
```

**Key Points:**
- `rosidl_generate_interfaces()` generates Python and C++ code from all .msg files
- `DEPENDENCIES std_msgs` tells the generator which message types are used
- Currently configured to build three messages: RcChannels, and FlightMode

---

## 4. Build Instructions

### Build from workspace root:

```bash
# Navigate to workspace
cd C:\Users\PDAS-L0001\Desktop\Projects\UAV_NAV_ROS

# Build only the custom_interfaces package
colcon build --packages-select custom_interfaces

# Or build the example package too
colcon build --packages-select custom_interfaces telemetry_example

# Or build all packages
colcon build
```

### After building, source the setup:

```bash
# Windows PowerShell
.\install\setup.ps1

# For each terminal session, run this to enable the new messages
```

### Verify the message was created:

```bash
ros2 interface show custom_interfaces/msg/TelemetryArray
```

Expected output:
```
std_msgs/Header header
    uint32 seq
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
int32[] data
string metadata
```

---

## 5. Python Publisher Node

### File: `src/telemetry_example/telemetry_example/telemetry_publisher.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import TelemetryArray
from std_msgs.msg import Header
import time


class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher')
        
        # Create publisher for TelemetryArray messages
        self.publisher = self.create_publisher(
            TelemetryArray,
            'telemetry/data',
            10  # QoS depth
        )
        
        # Create a timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)
        
        self.counter = 0
        self.get_logger().info('TelemetryArray Publisher started')

    def publish_telemetry(self):
        """Publish a TelemetryArray message."""
        
        # Create message with header
        msg = TelemetryArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        
        # Populate data array (simulated sensor values)
        msg.data = [
            100 + self.counter,
            200 + self.counter,
            300 + self.counter,
            400 + self.counter,
            500 + self.counter,
        ]
        
        # Add metadata
        msg.metadata = f'Sensor batch #{self.counter}, timestamp: {time.time():.2f}'
        
        # Publish the message
        self.publisher.publish(msg)
        
        self.get_logger().debug(
            f'Published: data={msg.data}, metadata="{msg.metadata}"'
        )
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = TelemetryPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 6. Python Subscriber Node

### File: `src/telemetry_example/telemetry_example/telemetry_subscriber.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import TelemetryArray


class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        
        # Create subscription for TelemetryArray messages
        self.subscription = self.create_subscription(
            TelemetryArray,
            'telemetry/data',
            self.telemetry_callback,
            10  # QoS depth
        )
        
        self.get_logger().info('TelemetryArray Subscriber started')

    def telemetry_callback(self, msg: TelemetryArray):
        """Callback function when a TelemetryArray message is received."""
        
        # Extract header information
        frame_id = msg.header.frame_id
        timestamp = msg.header.stamp
        
        # Extract data array
        data_values = msg.data
        
        # Extract metadata
        metadata = msg.metadata
        
        # Log the received message
        self.get_logger().info(
            f'\n--- TelemetryArray Message ---\n'
            f'Frame ID: {frame_id}\n'
            f'Timestamp: {timestamp.sec}.{timestamp.nanosec}\n'
            f'Data: {data_values}\n'
            f'Metadata: {metadata}\n'
        )
        
        # Calculate statistics
        if data_values:
            avg = sum(data_values) / len(data_values)
            max_val = max(data_values)
            min_val = min(data_values)
            
            self.get_logger().debug(
                f'Stats - Avg: {avg:.2f}, Max: {max_val}, Min: {min_val}'
            )


def main(args=None):
    rclpy.init(args=args)
    subscriber = TelemetrySubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 7. Runtime Usage

### Terminal 1 - Start Publisher:
```bash
# After building and sourcing setup
ros2 run telemetry_example telemetry_publisher
```

Output:
```
[INFO] [telemetry_publisher]: TelemetryArray Publisher started
[DEBUG] [telemetry_publisher]: Published: data=[100, 200, 300, 400, 500], metadata="Sensor batch #0, timestamp: 1234567890.12"
[DEBUG] [telemetry_publisher]: Published: data=[101, 201, 301, 401, 501], metadata="Sensor batch #1, timestamp: 1234567890.22"
...
```

### Terminal 2 - Start Subscriber:
```bash
ros2 run telemetry_example telemetry_subscriber
```

Output:
```
[INFO] [telemetry_subscriber]: TelemetryArray Subscriber started
[INFO] [telemetry_subscriber]: 
--- TelemetryArray Message ---
Frame ID: sensor_frame
Timestamp: 1234567890.123456789
Data: [100, 200, 300, 400, 500]
Metadata: Sensor batch #0, timestamp: 1234567890.12
```

### Terminal 3 - Echo Messages:
```bash
ros2 topic echo /telemetry/data
```

Output:
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: sensor_frame
data:
- 100
- 200
- 300
- 400
- 500
metadata: Sensor batch #0, timestamp: 1234567890.12
---
header:
  stamp:
    sec: 1234567890
    nanosec: 223456789
  frame_id: sensor_frame
data:
- 101
- 201
- 301
- 401
- 501
metadata: Sensor batch #1, timestamp: 1234567890.22
---
```

### Get Message Info:
```bash
# Show message structure
ros2 interface show custom_interfaces/msg/TelemetryArray

# List all interfaces
ros2 interface list | grep custom_interfaces
```

---

## 8. Integration with Existing Node (servo_control.py)

To use `TelemetryArray` in your `servo_control.py`:

```python
from custom_interfaces.msg import TelemetryArray
from std_msgs.msg import Header

# In your publisher setup:
self.telemetry_pub = self.create_publisher(
    TelemetryArray,
    'servo/telemetry',
    10
)

# In your callback/timer function:
def publish_telemetry(self):
    msg = TelemetryArray()
    msg.header = Header()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'servo_controller'
    
    # Collect servo position data
    msg.data = [cfg['val'] for cfg in self.hw_map.values()]
    msg.metadata = f"Mode: {self.current_mode}, Armed: {self.is_armed}"
    
    self.telemetry_pub.publish(msg)
```

---

## 9. Troubleshooting

### "ImportError: cannot import name 'TelemetryArray'"
- **Solution**: Rebuild with `colcon build` and source the setup file again

### "Could not find all dependencies of message"  
- **Solution**: Ensure `std_msgs` is in your `build_depend` in package.xml

### Message appears empty
- **Solution**: Check that you're populating the message fields before publishing

### "Topic not found" in subscriber
- **Solution**: Ensure subscriber is listening to the correct topic name (`/telemetry/data`)

---

## Package Structure

```
src/
├── custom_interfaces/                # Message definition package
│   ├── msg/
│   │   ├── TelemetryArray.msg        # Custom message definition
│   │   ├── RcChannels.msg            # RC channels message
│   │   └── FlightMode.msg            # Flight mode message
│   ├── package.xml                   # Package manifest
│   ├── CMakeLists.txt               # Build configuration
│   └── custom_interfaces/
│       └── __init__.py
│
└── telemetry_example/                # Example node package
    ├── telemetry_example/
    │   ├── __init__.py
    │   ├── telemetry_publisher.py    # Publisher node
    │   └── telemetry_subscriber.py   # Subscriber node
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── resource/
        └── telemetry_example
```

---

## ROS 2 Jazzy Compatibility

This guide uses ROS 2 Jazzy (and later) standards:
✅ Package format 3 (`package.xml`)  
✅ Modern `setup.py` with entry points  
✅ `rosidl` for message generation  
✅ `ament_cmake` build system  
✅ No ROS 1 baggage  

