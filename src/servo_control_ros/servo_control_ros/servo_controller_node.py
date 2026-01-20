#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import lgpio
import time
import os
import sys

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # --- 1. REAL-TIME SCHEDULING OPTIMIZATION ---
        # Force the Linux kernel to prioritize this process above all non-RT tasks
        try:
            # 99 is the highest possible real-time priority in Linux
            param = os.sched_param(99)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            self.get_logger().info("RT: Process set to SCHED_FIFO with priority 99")
        except PermissionError:
            self.get_logger().error("RT: Permission Denied. RUN WITH SUDO to fix jitter!")
        
        self.declare_parameter('pwm_freq', 50) # 50Hz for standard servos

        # Topics
        self.declare_parameter('sub_topics.channels', 'rc/channels')
        self.declare_parameter('sub_topics.mode', 'rc/mode')
        
        self.gpiochip = self.get_parameter('gpiochip').value
        self.pwm_freq = self.get_parameter('pwm_freq').value

        # --- 3. PIN MAPPING ---
        # Note: Pins 12, 13, 18, 19 are hardware PWM capable on CM5
        # Format: [GPIO_PIN, RC_CHANNEL_INDEX, LAST_KNOWN_VALUE]
        self.output_map = {
            'rotor':    {'pin': 20, 'ch': 2, 'val': 1000},
            'rudder':   {'pin': 22, 'ch': 3, 'val': 1500},
            'elevator': {'pin': 24, 'ch': 1, 'val': 1500},
            'l_aileron':{'pin': 23, 'ch': 0, 'val': 1500},
            'r_aileron':{'pin': 27, 'ch': 0, 'val': 1500}
        }

        # --- 4. HARDWARE INIT ---
        try:
            self.h = lgpio.gpiochip_open(self.gpiochip)
            for name, cfg in self.output_map.items():
                lgpio.gpio_claim_output(self.h, cfg['pin'])
        except Exception as e:
            self.get_logger().error(f"GPIO Init Failed: {e}")
            sys.exit(1)

        self.is_armed = False
        self.current_mode = 'MANUAL'
        
        # --- 5. SUBSCRIPTIONS ---
        # History depth of 1 is critical: always process the NEWEST data, drop old packets
        self.create_subscription(Int32MultiArray, self.get_parameter('sub_topics.channels').value, self.rc_callback, 1)
        self.create_subscription(String, self.get_parameter('sub_topics.mode').value, self.mode_callback, 10)

        self.arm_esc()
        self.get_logger().info('Servo Controller Node Fully Initialized (RT Optimized)')

    def set_pwm(self, pin, pulse_us):
        """
        Calculates duty cycle for lgpio.
        Period at 50Hz = 20,000 microseconds.
        """
        # (Pulse / 20000) * 100 = Duty Cycle Percentage
        duty = (pulse_us / 200.0) # Simplified math for (pulse/20000)*100
        lgpio.tx_pwm(self.h, pin, self.pwm_freq, duty)

    def arm_esc(self):
        """Arming sequence: set throttle low for 5 seconds."""
        self.get_logger().info('Arming ESC: Sending low signal...')
        for name, cfg in self.output_map.items():
            safe_val = 900 if name == 'rotor' else 1500
            self.set_pwm(cfg['pin'], safe_val)
        
        # We use a simple sleep here as it's during initialization
        time.sleep(5.0)
        self.is_armed = True
        self.get_logger().info('ESC ARMED and Ready.')

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def rc_callback(self, msg):
        if not self.is_armed or self.current_mode != 'MANUAL':
            return
            
        data = msg.data
        # Iterate through our mapping and update pins
        for name, cfg in self.output_map.items():
            idx = cfg['ch']
            if 0 <= idx < len(data):
                pulse = data[idx]
                
                # Fast Clamping
                if pulse < 1000: pulse = 1000
                elif pulse > 2000: pulse = 2000
                
                # ONLY update hardware if the value has changed.
                # This significantly reduces bus traffic and prevents jitter
                self.get_logger(f"Setting {name} (Pin {cfg['pin']}) to {pulse}us, older value was {cfg['val']}us")
                if pulse != cfg['val']:
                    self.set_pwm(cfg['pin'], pulse)
                    cfg['val'] = pulse

    def destroy_node(self):
        self.get_logger().info('Shutting down: Setting safe outputs...')
        if hasattr(self, 'h'):
            # Set rotor to 1000 (Off) before closing
            self.set_pwm(self.output_map['rotor']['pin'], 1000)
            lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()