#!/usr/bin/env python3
"""
ROS2 Node for the ZOE-M8Q GPS module using UBX protocol.
Supports both I2C and UART interfaces via configuration.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
import time

from sensors.zoe_ubx.ubx_lib import (
    I2CInterface, UARTInterface, GPSInterface,
    configure_rate, configure_nav_pvt_only, configure_uart_baudrate,
    read_from_interface, parse_ubx_stream, NAVPVTParser,
    DEFAULT_GPS_ADDR, DEFAULT_UART_PORT, DEFAULT_UART_BAUD
)


class ZoeM8QNode(Node):
    def __init__(self):
        super().__init__('zoe_m8q_node')

        self.declare_all_parameters()
        self.load_parameters()
        
        self.interface = None
        self._ok = True

        # Initialize interface based on config
        try:
            if self.interface_type.lower() == 'uart':
                # For UART, always use 115200 baud for reliability
                # Hardcoded: /dev/ttyAMA0 (RPi UART0)
                target_baud = 115200
                
                # Try connecting at target baud first (in case GPS already configured)
                try:
                    self.interface = UARTInterface(
                        port=DEFAULT_UART_PORT,
                        baudrate=target_baud
                    )
                    # Quick test - try to read some data
                    time.sleep(0.2)
                    test_data = self.interface.read(64)
                    if len(test_data) > 0 and (0xB5 in test_data or 0x24 in test_data):  # UBX or NMEA
                        self.get_logger().info(f'✅ Connected at {target_baud} baud')
                    else:
                        raise Exception("No GPS data at target baud")
                except:
                    # Fall back: Connect at default 9600, configure GPS, then switch
                    self.get_logger().info(f'⚙️ Connecting at 9600 baud, will configure to {target_baud}...')
                    self.interface = UARTInterface(
                        port=DEFAULT_UART_PORT,
                        baudrate=9600
                    )
                    configure_uart_baudrate(self.interface, target_baud)
                    self.get_logger().info(f'✅ Baud rate set to {target_baud}')
                
                self.get_logger().info(
                    f'✅ GPS configured: {self.rate_hz} Hz via UART on {DEFAULT_UART_PORT}'
                )
            else:  # Default to I2C
                # Hardcoded: bus 1, address 0x42
                self.interface = I2CInterface(
                    bus=1,
                    address=DEFAULT_GPS_ADDR
                )
                self.get_logger().info(
                    f'✅ GPS configured: {self.rate_hz} Hz via I2C on bus 1 @ 0x{DEFAULT_GPS_ADDR:02X}'
                )
            
            # Configure GPS module rate and messages
            configure_rate(self.interface, self.rate_hz)
            configure_nav_pvt_only(self.interface)
            time.sleep(0.25)  # Wait for config to take effect
            
        except Exception as e:
            self.get_logger().error(f'❌ GPS Config error: {e}')
            self._ok = False
            return

        self.buf = bytearray()
        
        # Timer
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self.tick)

    def declare_all_parameters(self):
        # Interface Selection
        self.declare_parameter('interface_type.value', 'uart')  # 'i2c' or 'uart'
        
        # GPS Parameters
        self.declare_parameter('rate_hz.value', 10.0)
        self.declare_parameter('frame_id.value', 'gps_link')
        
        # Sensor Variances (used if sensor accuracy is 0/invalid)
        self.declare_parameter('sensor_variance.pos_std.value', [4.0, 4.0, 10.0])  # m
        self.declare_parameter('sensor_variance.vel_std.value', 0.5)  # m/s

    def load_parameters(self):
        # Interface config
        self.interface_type = self.get_parameter('interface_type.value').value
        
        # GPS config
        self.rate_hz = float(self.get_parameter('rate_hz.value').value)
        self.frame_id = self.get_parameter('frame_id.value').value
        
        self.pos_std = self.get_parameter('sensor_variance.pos_std.value').value
        self.vel_std = self.get_parameter('sensor_variance.vel_std.value').value

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/gps_zoe_m8q/fix', 10)
        self.pub_vel = self.create_publisher(TwistWithCovarianceStamped, '/gps_zoe_m8q/vel', 10)

    def tick(self):
        if self.interface is None:
            return
            
        try:
            chunk = read_from_interface(self.interface)
            if not chunk:
                return
            self.buf.extend(chunk)
            
            packets, self.buf = parse_ubx_stream(self.buf)
            
            for cls_, id_, pl in packets:
                # NAV-PVT Class 0x01 ID 0x07
                if cls_ == 0x01 and id_ == 0x07:
                    data = NAVPVTParser.parse(pl)
                    if data:
                        self.publish_data(data)
                        
        except Exception as e:
            self.get_logger().error(f'❌ Error reading/parsing GPS: {e}')
            
    def publish_data(self, data):
        timestamp = self.get_clock().now().to_msg()
        
        # 1. NavSatFix
        fix_msg = NavSatFix()
        fix_msg.header.stamp = timestamp
        fix_msg.header.frame_id = self.frame_id
        
        fix_msg.latitude = float(data.lat)
        fix_msg.longitude = float(data.lon)
        fix_msg.altitude = float(data.height)  # Ellipsoid height
        
        # Status
        if data.fixType >= 2:  # 2D, 3D, GNSS+DR, Time all count as a fix
            fix_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
             
        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        # Covariance
        h_acc = data.hAcc if data.hAcc > 0 else self.pos_std[0]
        v_acc = data.vAcc if data.vAcc > 0 else self.pos_std[2]
        
        fix_msg.position_covariance = [
            h_acc ** 2, 0.0, 0.0,
            0.0, h_acc ** 2, 0.0,
            0.0, 0.0, v_acc ** 2
        ]
        fix_msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.pub_fix.publish(fix_msg)
        
        # 2. TwistWithCovarianceStamped (Velocity)
        vel_msg = TwistWithCovarianceStamped()
        vel_msg.header.stamp = timestamp
        vel_msg.header.frame_id = self.frame_id
        
        vel_msg.twist.twist.linear.x = float(data.velN)  # North
        vel_msg.twist.twist.linear.y = float(data.velE)  # East
        vel_msg.twist.twist.linear.z = float(data.velD)  # Down
        
        # Covariance
        s_acc = data.sAcc if data.sAcc > 0 else self.vel_std
        cov = s_acc ** 2
        
        vel_msg.twist.covariance[0] = cov
        vel_msg.twist.covariance[7] = cov
        vel_msg.twist.covariance[14] = cov
        
        self.pub_vel.publish(vel_msg)

    def destroy(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.interface is not None:
            self.interface.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZoeM8QNode()

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