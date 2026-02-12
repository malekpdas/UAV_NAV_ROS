#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import imufusion
import time

class ImuFusionTest(Node):
    def __init__(self):
        super().__init__("imufusion_test")

        self.sub = self.create_subscription(Imu, "/imu_bno085/data", self.cb, 10)

        # IMUFusion AHRS
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NED,
            0.5,     # gain >0 so accel corrects roll/pitch
            500,    # gyro full scale in DEG/S
            10,       # accel rejection
            0,        # mag rejection (unused)
            500
        )

        self.last_time = None
        self.get_logger().info("IMUFusion gyro+accel test (mag disabled)")

    def cb(self, msg: Imu):
        t = time.time()
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t

        # Gyro MUST be rad/s
        gyro = np.degrees(np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]))

        # Accel MUST be m/s^2 (ROS standard)
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])/9.8066

        # No magnetometer
        mag = np.array([0.0, 0.0, 0.0])

        # Update AHRS
        self.ahrs.update(gyro, accel, mag, dt)

        # Output Euler
        roll, pitch, yaw = self.ahrs.quaternion.to_euler()

        self.get_logger().info(
            f"R={roll:7.2f}  P={pitch:7.2f}  Y={yaw:7.2f} | "
            f"|gyro|={np.linalg.norm(gyro):.4f} | "
        )
        self.get_logger().info(f"\nearth accel:\t{self.ahrs.earth_acceleration*9.8066}\ngravity:\t{self.ahrs.gravity*9.8066}")

def main():
    rclpy.init()
    node = ImuFusionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
