import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np
import time

from .ekf2_core import Ekf2Core, NUM_STATES
from .ekf2_states import *

class Ekf2Node(Node):
    def __init__(self):
        super().__init__('ekf2_node')
        
        # Params
        self.declare_parameter('enable_wind_estimation', True)
        self.declare_parameter('sensor_delay_ms', 0)
        
        # Core
        self.ekf = Ekf2Core()
        self.ekf.enable_wind = self.get_parameter('enable_wind_estimation').value
        
        # Subs
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.sub_gps_fix = self.create_subscription(NavSatFix, '/gps/fix', self.gps_fix_cb, 10)
        self.sub_gps_vel = self.create_subscription(TwistWithCovarianceStamped, '/gps/vel', self.gps_vel_cb, 10)
        self.sub_mag = self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)
        
        # Pubs
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.last_imu_time = 0.0
        
        self.get_logger().info("EKF2 Estimator Started")

    def imu_cb(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        dt = 0.02
        if self.last_imu_time > 0:
            dt = now - self.last_imu_time
        self.last_imu_time = now
        
        # PREDICT
        self.ekf.predict(gyro, accel, dt)
        
        # PUBLISH (Synchronous with IMU for high rate)
        self.publish_output(msg.header.stamp)

    def gps_fix_cb(self, msg):
        if msg.status.status < 0: return # No fix
        
        # Convert LLH to NED (Simplistic local tangent plane)
        # Needs a reference origin.
        if not hasattr(self, 'origin_lat'):
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.get_logger().info(f"Set EKF Origin: {self.origin_lat}, {self.origin_lon}")
            return
            
        # Flat earth approx for small area
        R_earth = 6378137.0
        d_lat = np.radians(msg.latitude - self.origin_lat)
        d_lon = np.radians(msg.longitude - self.origin_lon)
        lat0 = np.radians(self.origin_lat)
        
        pos_n = d_lat * R_earth
        pos_e = d_lon * R_earth * np.cos(lat0)
        pos_d = -(msg.altitude - self.origin_alt)
        
        pos_ned = np.array([pos_n, pos_e, pos_d])
        var_h = msg.position_covariance[0]
        var_v = msg.position_covariance[8]
        
        self.ekf.update_gps_pos(pos_ned, var_h, var_v)

    def gps_vel_cb(self, msg):
        # msg.twist.twist.linear is ENU (from our driver)
        # Convert to NED
        # ENU X -> NED E
        # ENU Y -> NED N
        # ENU Z -> NED -D
        
        vel_e = msg.twist.twist.linear.x
        vel_n = msg.twist.twist.linear.y
        vel_d = -msg.twist.twist.linear.z
        
        vel_ned = np.array([vel_n, vel_e, vel_d])
        var_s = msg.twist.covariance[0]
        
        self.ekf.update_gps_vel(vel_ned, var_s)

    def mag_cb(self, msg):
        # Correct with Mag
        # mag_body = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        # self.ekf.update_mag_heading(mag_body)
        pass

    def publish_output(self, stamp):
        s = self.ekf.state
        
        # Odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Pose
        odom.pose.pose.position.x = s[IX_POS_E] # Map NED to ENU (Odom is usually ENU)
        odom.pose.pose.position.y = s[IX_POS_N]
        odom.pose.pose.position.z = -s[IX_POS_D]
        
        # Quat (NED to ENU conversion needed?)
        # Base link is usually FRD or FLU.
        # If EKF is NED, and Base_link is FRD.
        # ROS Odom is ENU. 
        # Transformation:
        # P_enu = [P_ned.E, P_ned.N, -P_ned.D] ? No.
        # N=Y, E=X, D=-Z
        # So P_enu.X = E, P_enu.Y = N
        
        odom.pose.pose.orientation.w = s[IX_QUAT_W]
        odom.pose.pose.orientation.x = s[IX_QUAT_Y] # Swap logic needed for NED->ENU quat?
        odom.pose.pose.orientation.y = s[IX_QUAT_X]
        odom.pose.pose.orientation.z = -s[IX_QUAT_Z]
        # Note: Proper quat conversion is non-trivial. Leaving as placeholder mapping.
        
        # Twist
        odom.twist.twist.linear.x = s[IX_VEL_E]
        odom.twist.twist.linear.y = s[IX_VEL_N]
        odom.twist.twist.linear.z = -s[IX_VEL_D]
        
        self.pub_odom.publish(odom)
        
        # TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Ekf2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
