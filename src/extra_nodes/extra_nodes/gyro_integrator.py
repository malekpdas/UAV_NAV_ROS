#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time

class GyroIntegrator(Node):
    def __init__(self):
        super().__init__("gyro_integrator")

        self.sub = self.create_subscription(
            Imu, "/imu_bno085/data", self.imu_cb, 10
        )

        # roll, pitch, yaw in radians
        self.rpy = np.zeros(3)
        self.last_time = None

        self.get_logger().info("Gyro integrator started (no accel correction!)")

    def imu_cb(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        # Gyro from ROS IMU (should be rad/s)
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Integrate
        self.rpy[0] += gx * dt
        self.rpy[1] += gy * dt
        self.rpy[2] += gz * dt

        # Convert to degrees for printing
        rpy_deg = np.degrees(self.rpy)

        self.get_logger().info(
            f"Roll: {rpy_deg[0]:7.2f} deg | Pitch: {rpy_deg[1]:7.2f} deg | Yaw: {rpy_deg[2]:7.2f} deg"
        )


def main():
    rclpy.init()
    node = GyroIntegrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
