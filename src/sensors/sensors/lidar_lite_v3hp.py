#!/usr/bin/env python3
"""
ROS2 Node for Garmin Lidar Lite v3HP.
Publishes range data to `lidar/range`.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from sensors.lite_v3hp.v3hp_lib import LidarLite, DEFAULT_ADDRESS

class LidarLiteNode(Node):
    def __init__(self):
        super().__init__('lidar_lite_node')
        
        self.declare_all_parameters()
        self.load_parameters()
        
        # Determine Final Settings
        if self.preset_name not in LidarLite.PRESETS:
            self.get_logger().warn(f"Unknown preset '{self.preset_name}', falling back to 'balanced'")
            self.preset_name = 'balanced'
            
        settings = LidarLite.PRESETS[self.preset_name].copy()
        
        # Check for individual overrides
        for key in settings.keys():
            val = self.get_parameter(f'sensor_config.{key}.value').value
            if val != -1:
                settings[key] = val
                self.get_logger().info(f"Overriding {key} from preset with: {val}")

        # Hardware
        self.lidar = LidarLite(1, DEFAULT_ADDRESS)
        if self.lidar.connected:
            self.get_logger().info(f'✅ Lidar Lite init OK on bus {1} at address 0x{DEFAULT_ADDRESS:02X}')
        else:
            self.get_logger().fatal(f'❌ Lidar Lite init FAILED on bus {1} at address 0x{DEFAULT_ADDRESS:02X}')
            self._ok = False
            return
        
        if self.lidar.configure(settings):
            self.get_logger().info(f'✅ Lidar Lite configured with preset {preset_name}')
        else:
            self.get_logger().warn('❌ Failed to apply some Lidar Lite settings.')
            
        # Publisher
        self.pub_range = self.create_publisher(Range, "lidar/range", 10)
        
        # Timer
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(f'Lidar Lite Node started. Freq: {self.freq}Hz')

    def declare_all_parameters(self):
        # ROS Parameters
        self.declare_parameter('frequency.value', 100.0) # Hz
        self.declare_parameter('frame_id.value', 'lidar_link')

        # Sensor Configuration Parameters
        self.declare_parameter('sensor_config.min_range.value', 0.05)
        self.declare_parameter('sensor_config.max_range.value', 40.0)
        self.declare_parameter('sensor_config.preset.value', 'balanced')
        self.declare_parameter('sensor_config.sig_count_val.value', -1)
        self.declare_parameter('sensor_config.acq_config_reg.value', -1)
        self.declare_parameter('sensor_config.threshold_bypass.value', -1)
        self.declare_parameter('sensor_config.ref_count_val.value', -1)

    def load_parameters(self):
        self.freq = self.get_parameter('frequency.value').value
        self.frame_id = self.get_parameter('frame_id.value').value
        
        self.min_range = self.get_parameter('sensor_config.min_range.value').value
        self.max_range = self.get_parameter('sensor_config.max_range.value').value
        self.preset_name = self.get_parameter('sensor_config.preset.value').value

    def tick(self):
        dist = self.lidar.read_distance()
        
        if dist is None:
            self.get_logger().warn(f"Lidar Distance is None", throttle_duration_sec=1.0)
            return
        
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.01
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = dist
                
        self.pub_range.publish(msg)
      

    def destroy(self):
        if hasattr(self, 'lidar'):
            self.lidar.close()            
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarLiteNode()

    if not getattr(node, '_ok', True):
        node.destroy()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
