#!/usr/bin/env python3
"""
ROS2 Node for the ZOE-M8Q GPS module using UBX protocol.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from smbus2 import SMBus
import time
import numpy as np
from sensors.zoe_ubx.ubx_lib import (
    set_rate_hz, enable_nav_pvt_only, read_chunk, parse_ubx_stream,
    NAVPVTParser, DEFAULT_GPS_ADDR
)

class ZoeM8QNode(Node):
    def __init__(self):
        super().__init__('zoe_m8q_node')

        self.declare_all_parameters()
        self.load_parameters()

        # I2C Setup
        try:
            self.bus = SMBus(1)
            set_rate_hz(self.bus, self.rate_hz)
            enable_nav_pvt_only(self.bus)
            self.get_logger().info(f'✅ Configured GPS: {self.rate_hz} Hz, UBX-NAV-PVT only on bus {1} at address 0x{DEFAULT_GPS_ADDR:02X}')
            time.sleep(0.25) # Wait for config to take effect
        except Exception as e:
            self.get_logger().error(f'❌ GPS Config error: {e}, on bus {1} at address 0x{DEFAULT_GPS_ADDR:02X}')
            self._ok = False
            return

        self.buf = bytearray()
        
        # Timer
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self.tick)

    def declare_all_parameters(self):
        # Parameters
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('frame_id', 'gps_link')

        
        # Sensor Variances (used if sensor accuracy is 0/invalid)
        self.declare_parameter('sensor_variance.pos_std', [4.0, 4.0, 10.0]) # m
        self.declare_parameter('sensor_variance.vel_std', [0.5]) # m/s (speed accuracy)

    def load_parameters(self):
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        
        self.pos_std = self.get_parameter('sensor_variance.pos_std').value
        self.vel_std = self.get_parameter('sensor_variance.vel_std').value

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/gps_zoe_m8q/fix', 10)
        self.pub_vel = self.create_publisher(TwistWithCovarianceStamped, '/gps_zoe_m8q/vel', 10)

    def tick(self):
        try:
            chunk = read_chunk(self.bus)
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
        fix_msg.altitude = float(data.height) # Ellipsoid height
        
        # Status
        # fixType: 0=NoFix, 1=DeadReckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=Time
        if data.fixType >= 2: # 2D, 3D, GNSS+DR, Time all count as a fix
            fix_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
             
        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        # Covariance (ENU)
        # data.hAcc and data.vAcc are in meters.
        # Covariance matrix diagonal: [LatVar, LonVar, AltVar]
        # Approximation: LatVar = LonVar = hAcc^2
        
        h_acc = data.hAcc
        v_acc = data.vAcc
        
        # Fallback if accuracy is reported as 0 (invalid)
        if h_acc <= 0: h_acc = self.pos_std[0]
        if v_acc <= 0: v_acc = self.pos_std[2]
        
        lat_cov = h_acc ** 2
        lon_cov = h_acc ** 2
        alt_cov = v_acc ** 2
        
        fix_msg.position_covariance = [
            lat_cov, 0.0, 0.0,
            0.0, lon_cov, 0.0,
            0.0, 0.0, alt_cov
        ]
        fix_msg.position_covariance_type = 2 # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.pub_fix.publish(fix_msg)
        
        # 2. TwistWithCovarianceStamped (Velocity)
        vel_msg = TwistWithCovarianceStamped()
        vel_msg.header.stamp = timestamp
        vel_msg.header.frame_id = self.frame_id
        
        vel_msg.twist.twist.linear.x = float(data.velN) # North
        vel_msg.twist.twist.linear.y = float(data.velE) # East
        vel_msg.twist.twist.linear.z = float(data.velD) # Down
        
        # Covariance
        # data.sAcc is speed accuracy (m/s)
        s_acc = data.sAcc
        if s_acc <= 0: s_acc = self.vel_std[0]
        
        # We assume uncorrelated error for x, y, z equal to speed accuracy
        cov = s_acc ** 2
        
        # 6x6 matrix, standard ROS ordering: (x, y, z, roll, pitch, yaw)
        # We populate diagonal for x, y, z. Angular velocity is unknown/zero.
        
        vel_msg.twist.covariance[0] = cov
        vel_msg.twist.covariance[7] = cov
        vel_msg.twist.covariance[14] = cov
        
        self.pub_vel.publish(vel_msg)
        
        if self.debug and data.fixType < 3:
             self.get_logger().warn(f"Bad Fix: {data.fixType}, Sats: {data.numSV}")

    def destroy(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.bus is not None:
            self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZoeM8QNode()

    if not getattr(node, '_ok', True):
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