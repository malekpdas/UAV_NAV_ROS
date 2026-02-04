#!/usr/bin/env python3
"""
rc_control.py - ROS2 Node for reading RC receiver signals
Reads 6 channels via lgpio and publishes to rc/channels and rc/mode
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import lgpio
import threading
import time

class RCReaderNode(Node):
    def __init__(self):
        super().__init__('rc_control')
        
        # Declare parameters
        self.declare_all_parameters()
        self.load_parameters()
        
        if len(self.pins) != 6:
            self.get_logger().error("Must provide exactly 6 GPIO pins!")
            self._ok = False
            return

        # Initialize GPIO
        self.h = lgpio.gpiochip_open(self.gpiochip)
        
        # Data Storage
        self.lock = threading.Lock()
        self.pulses = [1500] * 6
        self.last_rise = [None] * 6
        self.last_update = [time.time()] * 6
        self.failsafe = [False] * 6
        
        # Init Pins and Callbacks
        for i, pin in enumerate(self.pins):
            lgpio.gpio_claim_alert(self.h, pin, lgpio.BOTH_EDGES)
            lgpio.callback(self.h, pin, lgpio.BOTH_EDGES, self._make_callback(i))
            
        # Publishers
        self.pub_channels = self.create_publisher(Int32MultiArray, 'rc/channels', 10)
        self.pub_mode = self.create_publisher(String, 'rc/mode', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
        
        self.get_logger().info(f'RC Reader Node Started.')

    def declare_all_parameters(self):
        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('publish_rate', 50.0) # 50Hz is standard for servo
        self.declare_parameter('failsafe_timeout', 0.5)
        
        # Default Pins: [20, 25, 16, 12, 24, 21]
        # Ch 6: Mode Switch
        self.declare_parameter('gpio_pins', [20, 25, 16, 12, 24, 21])

    def load_parameters(self):
        # Get parameters
        self.gpiochip = self.get_parameter('gpiochip').value
        self.rate_hz = self.get_parameter('publish_rate').value
        self.failsafe_timeout = self.get_parameter('failsafe_timeout').value
        self.pins = self.get_parameter('gpio_pins').value

    def _make_callback(self, ch_idx):
        """Create a closure for specific channel callback"""
        def callback(chip, gpio, level, tick):
            if level == 1:
                self.last_rise[ch_idx] = tick
            elif level == 0 and self.last_rise[ch_idx] is not None:
                pulse = (tick - self.last_rise[ch_idx]) // 1000
                # Filter valid RC pulse range (approx 900-2100us)
                if 900 < pulse < 2100:
                    with self.lock:
                        self.pulses[ch_idx] = pulse
                        self.last_update[ch_idx] = time.time()
                        if self.failsafe[ch_idx]:
                            self.failsafe[ch_idx] = False
                            self.get_logger().info(f'Channel {ch_idx+1} Signal Restored')
        return callback

    def timer_callback(self):
        current_time = time.time()
        
        with self.lock:
            # Check Failsafe
            for i in range(6):
                if current_time - self.last_update[i] > self.failsafe_timeout:
                    if not self.failsafe[i]:
                        self.failsafe[i] = True
                        self.get_logger().warn(f'Channel {i+1} Failsafe Triggered')
                        pass

            # Prepare Messages
            # Msg 1: Channels 1-5
            msg_channels = Int32MultiArray()
            msg_channels.data = self.pulses[0:5] # First 5 channels
            
            # Msg 2: Mode (Channel 6)
            # Simple thresholding for switch
            ch6_val = self.pulses[5]
            mode_str = "MANUAL"
            if ch6_val > 1500:
                mode_str = "AUTO"
            
            msg_mode = String()
            msg_mode.data = mode_str
            
            self.pub_channels.publish(msg_channels)
            self.pub_mode.publish(msg_mode)

    def destroy_node(self):
        lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RCReaderNode()

    if not getattr(node, '_ok', True):
        rclpy.shutdown()
        return

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
