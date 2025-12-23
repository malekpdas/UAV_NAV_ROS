#!/usr/bin/env python3
"""
ROS2 Node for Garmin Lidar Lite v3HP.
Publishes range data to `lidar/range`.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from lidar_lite_ros.lidar_lite_lib import LidarLite

class LidarLiteNode(Node):
    def __init__(self):
        super().__init__('lidar_lite_node')
        
        # ROS Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frequency', 100.0) # Hz
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 40.0)
        
        # Hardware Config Parameters
        self.declare_parameter('preset', 'balanced')
        
        # Individual overrides (defaults to -1, which means "use preset value")
        self.declare_parameter('sig_count_val', -1)
        self.declare_parameter('acq_config_reg', -1)
        self.declare_parameter('threshold_bypass', -1)
        self.declare_parameter('ref_count_val', -1)
        
        self.bus_id = self.get_parameter('i2c_bus').value
        self.freq = self.get_parameter('frequency').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        
        # Determine Final Settings
        preset_name = self.get_parameter('preset').value
        if preset_name not in LidarLite.PRESETS:
            self.get_logger().warn(f"Unknown preset '{preset_name}', falling back to 'balanced'")
            preset_name = 'balanced'
            
        settings = LidarLite.PRESETS[preset_name].copy()
        
        # Check for individual overrides
        for key in settings.keys():
            val = self.get_parameter(key).value
            if val != -1:
                settings[key] = val
                self.get_logger().info(f"Overriding {key} from preset with: {val}")

        # Hardware
        self.lidar = LidarLite(bus_id=self.bus_id)
        if not self.lidar.connected:
            self.get_logger().error('Lidar Lite not detected!')
        else:
            if self.lidar.configure(settings):
                self.get_logger().info(f"Lidar Lite configured with preset '{preset_name}'")
            else:
                self.get_logger().warn('Failed to apply some Lidar Lite settings.')
            
        # Publisher
        self.pub_range = self.create_publisher(Range, 'lidar/range', 10)
        
        # Timer
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.tick)
        
        self.get_logger().info(f'Lidar Lite Node started. Bus: {self.bus_id}, Freq: {self.freq}Hz')

    def tick(self):
        dist = self.lidar.read_distance()
        
        if dist is not None:
             msg = Range()
             msg.header.stamp = self.get_clock().now().to_msg()
             msg.header.frame_id = self.frame_id
             msg.radiation_type = Range.INFRARED
             msg.field_of_view = 0.01 # Approx 8mrad ~ 0.5 deg
             msg.min_range = self.min_range
             msg.max_range = self.max_range
             msg.range = dist
             
             self.pub_range.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'lidar'):
            self.lidar.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarLiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
