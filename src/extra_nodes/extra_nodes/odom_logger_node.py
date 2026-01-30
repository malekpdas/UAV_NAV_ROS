#!/usr/bin/env python3
"""
Node for logging Odometry, Position, and Velocity data to CSV.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import csv
import os

class FullOdomLogger(Node):
    def __init__(self):
        super().__init__('full_odom_logger')

        # Robust QoS (Reliable)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.filename = 'uav_odom_full_log.csv'
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # FULL 3D HEADER
        header = [
            'timestamp_sec',
            'pos_x', 'pos_y', 'pos_z',
            'q_x', 'q_y', 'q_z', 'q_w',
            'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z'
        ]
        self.csv_writer.writerow(header)

        self.subscription = self.create_subscription(
            Odometry,
            '/ekf/odom',
            self.listener_callback,
            qos_profile=qos
        )
        
        self.get_logger().info(f'Logging ALL 3D data to {self.filename}...')

    def listener_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Position
        p = msg.pose.pose.position
        # Quaternion
        q = msg.pose.pose.orientation
        # Linear/Angular Velocity
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular

        row = [
            f'{t:.6f}',
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}',
            f'{lv.x:.6f}', f'{lv.y:.6f}', f'{lv.z:.6f}',
            f'{av.x:.6f}', f'{av.y:.6f}', f'{av.z:.6f}'
        ]
        
        self.csv_writer.writerow(row)

    def destroy_node(self):
        self.get_logger().info('Closing CSV...')
        self.csv_file.flush()
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FullOdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()