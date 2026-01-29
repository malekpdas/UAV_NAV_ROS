#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32MultiArray, String
import lgpio
import time
import os
import sys

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # --- 1. REAL-TIME SCHEDULING OPTIMIZATION ---
        try:
            # Use priority 80-90 for better stability (99 can cause system issues)
            param = os.sched_param(85)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            self.get_logger().info("RT: Process set to SCHED_FIFO with priority 85")
        except PermissionError:
            self.get_logger().warn("RT: Permission Denied. Run with sudo for real-time scheduling")
        except Exception as e:
            self.get_logger().warn(f"RT: Could not set real-time priority: {e}")
        
        # --- 2. PARAMETER DECLARATIONS ---
        self.declare_parameter('pwm_freq', 50)
        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('sub_topics.channels', 'rc/channels')
        self.declare_parameter('sub_topics.mode', 'rc/mode')
        
        # Pin declarations
        self.declare_parameter('pin_rotor', 20)
        self.declare_parameter('pin_rudder', 24)
        self.declare_parameter('pin_elevator', 25)
        self.declare_parameter('pin_l_aileron', 23)
        self.declare_parameter('pin_r_aileron', 22)
        
        # Channel mapping declarations
        self.declare_parameter('rc_ch_rotor', 2)
        self.declare_parameter('rc_ch_rudder', 3)
        self.declare_parameter('rc_ch_elevator', 1)
        self.declare_parameter('rc_ch_l_aileron', 0)
        self.declare_parameter('rc_ch_r_aileron', 4)
        
        # Safety parameters
        self.declare_parameter('arming_duration', 3.0)
        self.declare_parameter('min_pulse_us', 1000)
        self.declare_parameter('max_pulse_us', 2000)
        self.declare_parameter('neutral_pulse_us', 1500)
        self.declare_parameter('rotor_min_us', 1000)
        
        # Get parameters
        self.gpiochip = self.get_parameter('gpiochip').value
        self.pwm_freq = self.get_parameter('pwm_freq').value
        self.arming_duration = self.get_parameter('arming_duration').value
        self.min_pulse = self.get_parameter('min_pulse_us').value
        self.max_pulse = self.get_parameter('max_pulse_us').value
        self.neutral_pulse = self.get_parameter('neutral_pulse_us').value
        self.rotor_min = self.get_parameter('rotor_min_us').value
        
        # Pre-calculate PWM period for efficiency
        self.pwm_period_us = 1_000_000 / self.pwm_freq  # 20,000 us for 50Hz

        # --- 3. PIN MAPPING ---
        self.output_map = {
            'rotor': {
                'pin': self.get_parameter('pin_rotor').value,
                'ch': self.get_parameter('rc_ch_rotor').value,
                'val': None,  # Track last value for change detection
                'safe': self.rotor_min
            },
            'rudder': {
                'pin': self.get_parameter('pin_rudder').value,
                'ch': self.get_parameter('rc_ch_rudder').value,
                'val': None,
                'safe': self.neutral_pulse
            },
            'elevator': {
                'pin': self.get_parameter('pin_elevator').value,
                'ch': self.get_parameter('rc_ch_elevator').value,
                'val': None,
                'safe': self.neutral_pulse
            },
            'l_aileron': {
                'pin': self.get_parameter('pin_l_aileron').value,
                'ch': self.get_parameter('rc_ch_l_aileron').value,
                'val': None,
                'safe': self.neutral_pulse
            },
            'r_aileron': {
                'pin': self.get_parameter('pin_r_aileron').value,
                'ch': self.get_parameter('rc_ch_r_aileron').value,
                'val': None,
                'safe': self.neutral_pulse
            }
        }

        # --- 4. HARDWARE INIT ---
        self.h = None
        try:
            self.h = lgpio.gpiochip_open(self.gpiochip)
            self.get_logger().info(f"GPIO chip {self.gpiochip} opened successfully")
            
            # Configure all pins as outputs
            for name, cfg in self.output_map.items():
                lgpio.gpio_claim_output(self.h, cfg['pin'])
                self.get_logger().info(f"Claimed GPIO pin {cfg['pin']} for {name}")
                
        except Exception as e:
            self.get_logger().error(f"GPIO Init Failed: {e}")
            if self.h is not None:
                lgpio.gpiochip_close(self.h)
            sys.exit(1)

        # State variables
        self.is_armed = False
        self.current_mode = 'MANUAL'
        self.last_rc_time = self.get_clock().now()
        
        # --- 5. SUBSCRIPTIONS ---
        self.create_subscription(
            Int32MultiArray,
            self.get_parameter('sub_topics.channels').value,
            self.rc_callback,
            1  # QoS depth of 1 for latest data only
        )
        self.create_subscription(
            String,
            self.get_parameter('sub_topics.mode').value,
            self.mode_callback,
            10
        )
        
        # Create timer for watchdog (failsafe)
        self.create_timer(0.5, self.watchdog_callback)

        # Arm ESC sequence
        self.arm_esc()
        self.get_logger().info('Servo Controller Node Ready')

    def set_pwm(self, pin, pulse_us):
        """
        Set PWM signal with optimized duty cycle calculation.
        
        Args:
            pin: GPIO pin number
            pulse_us: Pulse width in microseconds (1000-2000)
        """
        if self.h is None:
            return
            
        try:
            # Calculate duty cycle percentage
            # duty = (pulse_us / period_us) * 100
            duty = (pulse_us / self.pwm_period_us) * 100.0
            
            # Clamp duty cycle to valid range
            duty = max(0.0, min(100.0, duty))
            
            lgpio.tx_pwm(self.h, pin, self.pwm_freq, duty)
        except Exception as e:
            self.get_logger().error(f"PWM set failed on pin {pin}: {e}")

    def arm_esc(self):
        """
        ESC arming sequence: Send minimum throttle signal.
        """
        self.get_logger().info(f'Arming ESC: Sending safe signals for {self.arming_duration}s...')
        
        # Set all servos to safe positions
        for name, cfg in self.output_map.items():
            self.set_pwm(cfg['pin'], cfg['safe'])
            cfg['val'] = cfg['safe']
        
        # Wait for ESC to arm
        time.sleep(self.arming_duration)
        
        self.is_armed = True
        self.get_logger().info('ESC ARMED - System Ready')

    def mode_callback(self, msg):
        """Handle flight mode changes."""
        old_mode = self.current_mode
        self.current_mode = msg.data
        
        if old_mode != self.current_mode:
            self.get_logger().info(f'Mode changed: {old_mode} -> {self.current_mode}')
            
            # If switching away from MANUAL, set safe values
            if self.current_mode != 'MANUAL':
                self.set_safe_outputs()

    def rc_callback(self, msg):
        """
        Process RC channel data with optimizations:
        - Only update changed values
        - Bounds checking
        - Change detection to reduce GPIO operations
        """
        if not self.is_armed:
            return
            
        # Update last received time for watchdog
        self.last_rc_time = self.get_clock().now()
        
        # Only process in MANUAL mode
        # if self.current_mode != 'MANUAL':
        #     return
            
        data = msg.data
        
        # Process each mapped output
        for name, cfg in self.output_map.items():
            ch_idx = cfg['ch']
            
            # Validate channel index
            if ch_idx < 0 or ch_idx >= len(data):
                continue
            
            pulse = data[ch_idx]
            
            # Clamp to valid range
            pulse = max(self.min_pulse, min(self.max_pulse, pulse))
            
            # Only update if value has changed (reduces GPIO traffic)
            if pulse != cfg['val']:
                self.set_pwm(cfg['pin'], pulse)
                cfg['val'] = pulse
                # Reduce logging verbosity - only log at debug level
                self.get_logger().debug(
                    f"{name}: {pulse}us (pin {cfg['pin']}, ch {ch_idx})"
                )

    def watchdog_callback(self):
        """
        Failsafe watchdog: Set safe outputs if no RC data received recently.
        """
        if not self.is_armed:
            return
            
        time_since_rc = (self.get_clock().now() - self.last_rc_time).nanoseconds / 1e9
        
        # If no RC data for 1 second, trigger failsafe
        if time_since_rc > 1.0:
            self.get_logger().warn(
                f'RC signal lost ({time_since_rc:.1f}s) - Activating failsafe'
            )
            self.set_safe_outputs()

    def set_safe_outputs(self):
        """Set all outputs to safe values."""
        for name, cfg in self.output_map.items():
            safe_val = cfg['safe']
            if cfg['val'] != safe_val:
                self.set_pwm(cfg['pin'], safe_val)
                cfg['val'] = safe_val
                self.get_logger().info(f"Set {name} to safe value: {safe_val}us")

    def destroy_node(self):
        """Clean shutdown with safety measures."""
        self.get_logger().info('Shutting down - Setting safe outputs...')
        
        if self.h is not None:
            try:
                # Set all to safe values
                self.set_safe_outputs()
                
                # Small delay to ensure commands are sent
                time.sleep(0.1)
                
                # Release GPIO resources
                lgpio.gpiochip_close(self.h)
                self.get_logger().info('GPIO resources released')
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    
    # Use single-threaded executor for deterministic behavior
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()