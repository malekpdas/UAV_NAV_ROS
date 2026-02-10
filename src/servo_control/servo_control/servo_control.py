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
        super().__init__('servo_controller')
        
        self.declare_all_params()
        self.load_parameters()

        topics = [self.channels_topics, self.mode_topics]
        status, missing_topics = self.wait_for_topics(topics, self.timeout_sec, self.check_rate)
        if not status:
            self.get_logger().error(f'Topic(s) {missing_topics} not found, shutting down')
            self._ok = False
            return
        
        # Pre-calculate PWM period for efficiency
        self.pwm_period_us = 1_000_000 / self.pwm_freq  # 20,000 us for 50Hz

        # --- GPIO Connection INIT ---
        self.gpio_connection = None
        try:
            self.gpio_connection = lgpio.gpiochip_open(self.gpiochip)
            self.get_logger().info(f"GPIO chip {self.gpiochip} opened successfully")
            
            # Configure all pins as outputs
            for name, cfg in self.hw_map.items():
                lgpio.gpio_claim_output(self.gpio_connection, cfg['pin'])
                self.get_logger().info(f"Claimed GPIO pin {cfg['pin']} for {name}")  
        except Exception as e:
            self.get_logger().fatal(f"GPIO Init Failed: {e}")
            if self.gpio_connection is not None:
                lgpio.gpiochip_close(self.gpio_connection)
            self._ok = False
            return

        # State variables
        self.is_armed = False
        self.current_mode = 'MANUAL'
        self.last_rc_time = self.get_clock().now()
        
        # --- 5. SUBSCRIPTIONS ---
        self.create_subscription(
            Int32MultiArray,
            self.channels_topics,
            self.rc_callback,
            1  # QoS depth of 1 for latest data only
        )
        self.create_subscription(
            String,
            self.mode_topics,
            self.mode_callback,
            1
        )

        # --- 6. Create Publishers ---
        if self.publish_servos:
            self.servo_pub = self.create_publisher(
                Int32MultiArray,
                'servo/outputs',
                10
            )
        
        # Create timer for watchdog (failsafe)
        self._timer = self.create_timer(0.1, self.watchdog_callback)

        # Arm ESC sequence
        self.arm_esc()
        self.get_logger().info('Servo Controller Node Ready')

    def wait_for_topics(self, topics, timeout_sec, rate_hz):
        start = time.time()
        period = 1.0 / rate_hz

        while rclpy.ok():
            
            all_topics_ready = True
            missing_topics = []

            for topic in topics:
                pubs = self.get_publishers_info_by_topic(topic)
                if len(pubs) == 0:
                    all_topics_ready = False
                    missing_topics.append(topic)
            
            if all_topics_ready:
                return True, missing_topics  # topic is being published

            if time.time() - start > timeout_sec:
                return False, missing_topics  # timeout

            time.sleep(period)
       
    def declare_all_params(self):
        self.declare_parameter('gpiochip.value', 4)
        self.declare_parameter('pwm_freq.value', 50)

        # topics
        self.declare_parameter('sub_topics.channels.value', 'rc/channels')
        self.declare_parameter('sub_topics.mode.value', 'rc/mode')
        self.declare_parameter('timeout_sec.value', 5.0)
        self.declare_parameter('check_rate.value', 10.0)

        # Publish Servo
        self.declare_parameter('publish_servos.value', True)

        # Safety parameters
        self.declare_parameter('Safety.arming_duration.value', 5.0)
        self.declare_parameter('Safety.esc_min_pulse.value', 1000)
        self.declare_parameter('Safety.esc_max_pulse.value', 2000)
        self.declare_parameter('Safety.servo_min_pulse.value', 1000)
        self.declare_parameter('Safety.servo_max_pulse.value', 2000)
        
        # Pin Configuration
        self.declare_parameter('gpio_pins.l_aileron.value', 6)
        self.declare_parameter('gpio_pins.elevator.value', 19)
        self.declare_parameter('gpio_pins.esc_rotor.value', 5)
        self.declare_parameter('gpio_pins.rudder.value', 13)
        self.declare_parameter('gpio_pins.r_aileron.value', 26)
        
        # Channel Mapping (Index in rc_channels array: 0-4)
        self.declare_parameter('rc_channels.l_aileron.value', 0)
        self.declare_parameter('rc_channels.elevator.value', 1)
        self.declare_parameter('rc_channels.rotor.value', 2)
        self.declare_parameter('rc_channels.rudder.value', 3)
        self.declare_parameter('rc_channels.r_aileron.value', 4)

    def load_parameters(self):        
        # Get parameters
        self.gpiochip = self.get_parameter('gpiochip.value').value
        self.pwm_freq = self.get_parameter('pwm_freq.value').value

        # topics
        self.channels_topics = self.get_parameter('sub_topics.channels.value').value
        self.mode_topics = self.get_parameter('sub_topics.mode.value').value
        self.timeout_sec = self.get_parameter('timeout_sec.value').value
        self.check_rate = self.get_parameter('check_rate.value').value

        # Publish Servo
        self.publish_servos = self.get_parameter('publish_servos.value').value

        # Safety parameters
        self.arming_duration = self.get_parameter('Safety.arming_duration.value').value
        self.esc_min_pulse = self.get_parameter('Safety.esc_min_pulse.value').value
        self.esc_max_pulse = self.get_parameter('Safety.esc_max_pulse.value').value
        self.servo_min_pulse = self.get_parameter('Safety.servo_min_pulse.value').value
        self.servo_max_pulse = self.get_parameter('Safety.servo_max_pulse.value').value
        
        # Pin Configuration
        self.l_aileron_gpio_pin = self.get_parameter('gpio_pins.l_aileron.value').value
        self.elevator_gpio_pin = self.get_parameter('gpio_pins.elevator.value').value
        self.esc_rotor_gpio_pin = self.get_parameter('gpio_pins.esc_rotor.value').value
        self.rudder_gpio_pin = self.get_parameter('gpio_pins.rudder.value').value
        self.r_aileron_gpio_pin = self.get_parameter('gpio_pins.r_aileron.value').value
        
        # Channel Mapping (Index in rc_channels array: 0-4)
        self.l_aileron_rc_channel = self.get_parameter('rc_channels.l_aileron.value').value
        self.elevator_rc_channel = self.get_parameter('rc_channels.elevator.value').value
        self.esc_rotor_rc_channel = self.get_parameter('rc_channels.rotor.value').value
        self.rudder_rc_channel = self.get_parameter('rc_channels.rudder.value').value
        self.r_aileron_rc_channel = self.get_parameter('rc_channels.r_aileron.value').value

        # Full ESC and Servo Configuration
        self.hw_map = {
            'rotor': {
                'pin': self.esc_rotor_gpio_pin,
                'ch': self.esc_rotor_rc_channel,
                'val': None,  # Track last value for change detection
                'min': self.esc_min_pulse,
                'max': self.esc_max_pulse,
                'safe': min(self.esc_min_pulse, self.esc_max_pulse)
            },
            'rudder': {
                'pin': self.rudder_gpio_pin,
                'ch': self.rudder_rc_channel,
                'val': None,
                'min': self.servo_min_pulse,
                'max': self.servo_max_pulse,
                'safe': int((self.servo_min_pulse + self.servo_max_pulse) / 2)
            },
            'elevator': {
                'pin': self.elevator_gpio_pin,
                'ch': self.elevator_rc_channel,
                'val': None,
                'min': self.servo_min_pulse,
                'max': self.servo_max_pulse,
                'safe': int((self.servo_min_pulse + self.servo_max_pulse) / 2)
            },
            'l_aileron': {
                'pin': self.l_aileron_gpio_pin,
                'ch': self.l_aileron_rc_channel,
                'val': None,
                'min': self.servo_min_pulse,
                'max': self.servo_max_pulse,
                'safe': int((self.servo_min_pulse + self.servo_max_pulse) / 2)
            },
            'r_aileron': {
                'pin': self.r_aileron_gpio_pin,
                'ch': self.r_aileron_rc_channel,
                'val': None,
                'min': self.servo_min_pulse,
                'max': self.servo_max_pulse,
                'safe': int((self.servo_min_pulse + self.servo_max_pulse) / 2)
            }
        }

    def set_pwm(self, cfg):
        """
        Set PWM signal with optimized duty cycle calculation.
        
        Args:
            cfg: Configuration dictionary
        """

        pin = cfg['pin']
        pulse_us = cfg['val']
        min_pulse = cfg['min']
        max_pulse = cfg['max']

        pulse_us = max(min_pulse, min(max_pulse, pulse_us))
            
        try:
            # Calculate duty cycle percentage
            # duty = (pulse_us / period_us) * 100
            duty = (pulse_us / self.pwm_period_us) * 100.0
            
            # Clamp duty cycle to valid range
            duty = max(0, min(100.0, duty))
            
            lgpio.tx_pwm(self.gpio_connection, pin, self.pwm_freq, duty)
        except Exception as e:
            self.get_logger().error(f"PWM set failed on pin {pin}: {e}")

    def arm_esc(self):
        """
        ESC arming sequence: Send minimum throttle signal.
        """
        self.get_logger().info(f'Arming ESC: Sending safe signals for {self.arming_duration}s...')
        
        # Set all servos to safe positions
        for name, cfg in self.hw_map.items():
            cfg['val'] = cfg['safe']
            self.set_pwm(cfg)
        
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
        for name, cfg in self.hw_map.items():
            ch_idx = cfg['ch']
            
            # Validate channel index
            if ch_idx < 0 or ch_idx >= len(data):
                continue
            
            pulse = data[ch_idx]
            
            # Clamp to valid range
            pulse = max(self.esc_min_pulse, min(self.esc_max_pulse, pulse))
            
            # Only update if value has changed (reduces GPIO traffic)
            if pulse != cfg['val']:
                cfg['val'] = pulse
                self.set_pwm(cfg)
                # Reduce logging verbosity - only log at debug level
                self.get_logger().debug(
                    f"{name}: {pulse}us (pin {cfg['pin']}, ch {ch_idx})"
                )

        if self.publish_servos:
            # Publish current servo outputs
            pub_msg = Int32MultiArray()
            pub_msg.data = msg.data
            self.servo_pub.publish(pub_msg)

    def watchdog_callback(self):
        """
        Failsafe watchdog: Set safe outputs if no RC data received recently.
        """
        if not self.is_armed:
            return
            
        time_since_rc = (self.get_clock().now() - self.last_rc_time).nanoseconds / 1e9
        
        # If no RC data for 1 second, trigger failsafe
        if time_since_rc > 1.0:
            self.get_logger().warn(f'RC signal lost ({time_since_rc:.1f}s) - Activating failsafe', throttle_duration_sec=1.0)
            self.set_safe_outputs()

    def set_safe_outputs(self):
        """Set all outputs to safe values."""
        for name, cfg in self.hw_map.items():
            safe_val = cfg['safe']
            if cfg['val'] != safe_val:
                cfg['val'] = safe_val
                self.set_pwm(cfg)

    def destroy(self):
        """Clean shutdown with safety measures."""
        if self.gpio_connection is not None:
            try:
                # Set all to safe values
                self.set_safe_outputs()
                # Small delay to ensure commands are sent
                time.sleep(0.1)
                # Release GPIO resources
                lgpio.gpiochip_close(self.gpio_connection)
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")

        if self._timer is not None:
            self._timer.cancel()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()

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