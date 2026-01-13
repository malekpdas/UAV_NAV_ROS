#!/usr/bin/env python3
"""
servo_controller_node.py - ROS2 Node for driving Servos/ESCs
Subscribes to RC or Controller commands and drives output via lgpio
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import lgpio
import time

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # Declare Parameters
        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('pwm_freq', 50)
        
        # Mapping: Function -> {pin, channel_idx, type}
        # Types: 'servo', 'esc', 'switch'
        # Default Map based on user request "rudder, elevator, right/left ailerones, Rotor"
        # We need 5 outputs. Let's assume some pins (User didn't specify all OUTPUT pins, only INPUT pins).
        # Prototype `servo_controller` used pin 24 (Physical 18?) -> GPIO 18 is Pin 12? No.
        # Prototype `esc_controller` used servo_pin=22 (GPIO 25), esc_pin=20 (GPIO ?).
        # Let's declare a configurable map.
        # Key: Output Name
        # Value: List [GPIO_PIN, INPUT_CHANNEL_INDEX (-1 if not mapped directly)]
        # Default Map:
        # - Rotor (ESC): GPIO 20 (Pin 38 usually? No wait, let's use safe defaults or what was used).
        #   From `esc_controller_node.py`: servo_pin=22, esc_pin=20.
        #   From `servo_controller_node.py`: servo_pin=24.
        # Let's map: 
        #   Rotor -> 20 (Ch 3 usually throttle, idx 2)
        #   Rudder -> 22 (Ch 4 usually rudder, idx 3)
        #   Elevator -> 24 (Ch 2 usually elevator, idx 1)
        #   L_Aileron -> 23 (Ch 1 usually aileron, idx 0)
        #   R_Aileron -> 27 (Aux? or Split?) -> Let's use 27.
        # Input Channel Indices (Standard Mode 2):
        # 0: Aileron (Roll)
        # 1: Elevator (Pitch)
        # 2: Throttle
        # 3: Rudder (Yaw)
        # 4: Aux 1
        # 5: Mode
        
        # Format: [GPIO_PIN, RC_CHANNEL_INDEX]
        default_map = {
            'rotor': [20, 2],       # ESC
            'rudder': [22, 3],      # Servo
            'elevator': [24, 1],    # Servo
            'l_aileron': [23, 0],   # Servo
            'r_aileron': [27, 0],   # Servo (Split from same channel for now or seperate?)
                                    # If 6 channels input, maybe ch5 is R_Aileron? 
                                    # Usually Ailerons are on Ch1 (Y-cable) or Ch1+Ch5/6.
                                    # Let's map to Ch1 (0) for both for now (Y-cable logic).
        }
        
        # We will parse a string-based param or just hardcode structure for now with params overrides
        # ROS2 params don't support dictionaries well. Using separate params.
        
        self.declare_parameter('pin_rotor', 20)
        self.declare_parameter('pin_rudder', 22)
        self.declare_parameter('pin_elevator', 24)
        self.declare_parameter('pin_l_aileron', 23)
        self.declare_parameter('pin_r_aileron', 27)

        self.declare_parameter('rc_ch_rotor', 2)
        self.declare_parameter('rc_ch_rudder', 3)
        self.declare_parameter('rc_ch_elevator', 1)
        self.declare_parameter('rc_ch_l_aileron', 0)
        self.declare_parameter('rc_ch_r_aileron', 0) 
        
        self.gpiochip = self.get_parameter('gpiochip').value
        self.pwm_freq = self.get_parameter('pwm_freq').value
        
        # Build Map
        self.output_map = {}
        for name in ['rotor', 'rudder', 'elevator', 'l_aileron', 'r_aileron']:
            pin = self.get_parameter(f'pin_{name}').value
            ch = self.get_parameter(f'rc_ch_{name}').value
            self.output_map[name] = {'pin': pin, 'rc_ch': ch, 'current_val': 1500}
            if name == 'rotor':
                self.output_map[name]['current_val'] = 1000 # Default throttle off

        # Init GPIO
        self.h = lgpio.gpiochip_open(self.gpiochip)
        for name, cfg in self.output_map.items():
            lgpio.gpio_claim_output(self.h, cfg['pin'])
        
        # Arming Sequence (For Rotor/ESC)
        self.is_armed = False
        self.arm_esc() 
        
        # Mode
        self.current_mode = 'MANUAL'
        
        # Subscribers
        self.create_subscription(Int32MultiArray, 'rc/channels', self.rc_callback, 10)
        self.create_subscription(String, 'rc/mode', self.mode_callback, 10)
        # self.create_subscription(..., 'controller_cmd', self.cmd_callback, 10) # TODO
        
        self.get_logger().info('Servo Controller Started. Waiting for input.')

    def arm_esc(self):
        """Perform ESC Arming: 0 throttle for 5 seconds"""
        rotor_pin = self.output_map['rotor']['pin']
        self.get_logger().info('Arming ESC (Sending 900us)...')
        
        # Set all to neutral/safe
        for name, cfg in self.output_map.items():
            # Use 900us for rotor arming as verified in test_motor_direct.py
            safe_val = 900 if name == 'rotor' else 1500
            self.set_pwm(cfg['pin'], safe_val)
            
        time.sleep(5.0) # Blocking sleep is okay in init usually, or use timer. 
                        # In ROS2 node init, simple sleep holds up spinning, but fine for startup sequence.
        
        self.is_armed = True
        self.get_logger().info('ESC ARMED.')

    def mode_callback(self, msg):
        previous = self.current_mode
        self.current_mode = msg.data
        if previous != self.current_mode:
            self.get_logger().info(f"Mode changed: {previous} -> {self.current_mode}")

    def rc_callback(self, msg):
        """Handle RC Input"""
        # Safety Check
        if not self.is_armed:
            return
            
        # If Manual Mode, pass through RC signals
        if self.current_mode == 'MANUAL':
            data = msg.data
            for name, cfg in self.output_map.items():
                ch_idx = cfg['rc_ch']
                if 0 <= ch_idx < len(data):
                    pulse = data[ch_idx]
                    
                    if name != 'rotor':
                        self.get_logger().debug(f'MANUAL: {name} (Pin {cfg["pin"]}) = {pulse}')
                    
                    # Basic clamping
                    pulse = max(1000, min(2000, pulse))
                    
                    # Update Output
                    self.set_pwm(cfg['pin'], pulse)
                    cfg['current_val'] = pulse

        elif self.current_mode == 'AUTO':
            # In Future: Listen to controller_cmd
            # Log periodically (throttle logs) to explain why sticks don't work
            # Using a simple counter or rate limit would be better, but for now just pass.
            # actually rclpy doesn't have logging.throttle easily accessible in simple nodes without clock?
            # We'll just define a counter or rely on the mode change log we added earlier.
            pass

    def set_pwm(self, pin, pulse_us):
        # 50Hz = 20ms = 20000us
        # Duty cycle % = (pulse / 20000) * 100
        # lgpio expects duty cycle percentage
        duty = (pulse_us / 20000.0) * 100.0
        lgpio.tx_pwm(self.h, pin, self.pwm_freq, duty)

    def destroy_node(self):
        # Disarm/Safing
        if hasattr(self, 'h'):
            rotor_pin = self.output_map['rotor']['pin']
            self.set_pwm(rotor_pin, 1000)
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
