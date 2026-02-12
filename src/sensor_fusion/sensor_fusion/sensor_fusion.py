#!/usr/bin/env python3
"""
ROS2 Node for IMU/GPS integration using a Linear Kalman Filter.

Architecture:
- IMU measurements (cb_imu): Prediction step at high rate (~100Hz)
- GPS velocity (cb_gps_vel): Velocity update when available (~5-10Hz)
- GPS position (cb_gps_fix): Position update when available (~1-10Hz)

This separated update approach allows asynchronous sensor fusion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import imufusion
import numpy as np
from tf2_ros import TransformBroadcaster
import time

from sensor_fusion.lib.ekf_core import LinearKalmanFilter, RobustLPFilter
from sensor_fusion.lib.utils import lla_to_ned


class SensorFusionNode(Node):
    """
    ROS2 Node for IMU/GPS integration using a Linear Kalman Filter.
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Declare and load ROS2 parameters
        self.declare_all_parameters()
        self.load_parameters()

        topics = [
            self.imu_topic,
            self.mag_topic,
            self.gps_fix_topic,
            self.gps_vel_topic
        ]
        status, missing_topics = self.wait_for_topics(topics, self.timeout_sec, self.check_rate)
        if not status:
            self.get_logger().error(f'Topic(s) {missing_topics} not found, shutting down')
            self._ok = False
            return
        
        # AHRS Setup
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NED,
            self.ahrs_gain,
            self.ahrs_gyro_range,
            self.ahrs_accel_rejection,
            self.ahrs_mag_rejection,
            self.ahrs_rejection_timeout
        )

        # Kalman Filter for position/velocity estimation with bias
        kf_config = {
                'initial_pos_uncertainty': self.kf_initial_pos_uncertainty,
                'initial_vel_uncertainty': self.kf_initial_vel_uncertainty,
                'initial_bias_uncertainty': self.kf_initial_bias_uncertainty,
                'process_noise_pos': self.kf_process_noise_pos,
                'process_noise_vel': self.kf_process_noise_vel,
                'process_noise_bias': self.kf_process_noise_bias,
                'measurement_noise_pos': self.kf_measurement_noise_pos,
                'measurement_noise_vel': self.kf_measurement_noise_vel,
                'gravity': self.earth_gravity,
            }
        self.kf = LinearKalmanFilter(config=kf_config)
        self.kf_initialized = False
        
        self.last_imu_time = None
        self.current_gyro = np.zeros(3)
        
        self.latest_mag = None
        self.latest_gps_vel = None

        # ROS2 Publishers
        self.odom_pub = self.create_publisher(Odometry, '/fusion/odom', 10)
        if self.publish_acceleration:
            self.accel_pub = self.create_publisher(Imu, '/fusion/linear_acceleration', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS2 Subscriptions
        self.create_subscription(
            Imu, 
            self.imu_topic, 
            self.cb_imu, 
            10
        )
        self.create_subscription(
            MagneticField, 
            self.mag_topic, 
            self.cb_mag, 
            10
        )
        self.create_subscription(
            NavSatFix, 
            self.gps_fix_topic, 
            self.cb_gps_fix, 
            10
        )
        self.create_subscription(
            TwistWithCovarianceStamped, 
            self.gps_vel_topic, 
            self.cb_gps_vel, 
            10
        )

        # Low-pass filters
        self.lp_filter_accel = RobustLPFilter(alpha=self.lp_filter_accel_alpha)
        
        self.get_logger().info('IMU/GPS Fusion Node initialized')

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

    def declare_all_parameters(self):
        """Declare all ROS2 parameters with default values."""
        
        # Frame IDs
        self.declare_parameter('frames.map_frame.value', 'map')
        self.declare_parameter('frames.odom_frame.value', 'odom')
        self.declare_parameter('frames.base_link_frame.value', 'base_link')
        
        # Topics
        # Subscribed Topics
        self.declare_parameter('sub_topics.imu.value', '/imu/data')
        self.declare_parameter('sub_topics.mag.value', '/imu/mag')
        self.declare_parameter('sub_topics.gps_fix.value', '/gps/fix')
        self.declare_parameter('sub_topics.gps_vel.value', '/gps/vel')
        self.declare_parameter('sub_topics.timeout_sec.value', 5.0)
        self.declare_parameter('sub_topics.check_rate.value', 10.0)

        self.declare_parameter('publish_acceleration.value', False)
        
        # AHRS
        self.declare_parameter('ahrs.gain.value', 0.5)
        self.declare_parameter('ahrs.gyro_range.value', 500.0)
        self.declare_parameter('ahrs.accel_rejection.value', 10.0)
        self.declare_parameter('ahrs.mag_rejection.value', 10.0)
        self.declare_parameter('ahrs.rejection_timeout.value', 500)
        
        # Kalman Filter - Initial Uncertainties
        self.declare_parameter('kalman_filter.initial_pos_uncertainty.value', 100.0)
        self.declare_parameter('kalman_filter.initial_vel_uncertainty.value', 100.0)
        self.declare_parameter('kalman_filter.initial_bias_uncertainty.value', 1.0)
        
        # Kalman Filter - Process Noise
        self.declare_parameter('kalman_filter.process_noise_pos.value', 0.5)
        self.declare_parameter('kalman_filter.process_noise_vel.value', 0.5)
        self.declare_parameter('kalman_filter.process_noise_bias.value', 0.001)
        
        # Kalman Filter - Measurement Noise
        self.declare_parameter('kalman_filter.measurement_noise_pos.value', 25.0)
        self.declare_parameter('kalman_filter.measurement_noise_vel.value', 0.25)
        
        # LP Filters
        self.declare_parameter('lp_filters.accel_alpha.value', 0.9)
        
        # Earth Model
        self.declare_parameter('earth.gravity.value', 9.8066)
        self.declare_parameter('earth.radius.value', 6378137.0)
        self.declare_parameter('earth.flattening.value', 0.0033528106647474805)
        self.declare_parameter('earth.ref_pos.value', [0.0, 0.0, 0.0])

    def load_parameters(self):
        """Load all ROS2 parameters into a nested dictionary structure."""
        self.map_frame = self.get_parameter('frames.map_frame.value').value
        self.odom_frame = self.get_parameter('frames.odom_frame.value').value
        self.base_link_frame = self.get_parameter('frames.base_link_frame.value').value

        self.imu_topic = self.get_parameter('sub_topics.imu.value').value
        self.mag_topic = self.get_parameter('sub_topics.mag.value').value
        self.gps_fix_topic = self.get_parameter('sub_topics.gps_fix.value').value
        self.gps_vel_topic = self.get_parameter('sub_topics.gps_vel.value').value
        self.timeout_sec = self.get_parameter('sub_topics.timeout_sec.value').value
        self.check_rate = self.get_parameter('sub_topics.check_rate.value').value

        self.publish_acceleration = self.get_parameter('publish_acceleration.value').value

        self.ahrs_gain = self.get_parameter('ahrs.gain.value').value
        self.ahrs_gyro_range = self.get_parameter('ahrs.gyro_range.value').value
        self.ahrs_accel_rejection = self.get_parameter('ahrs.accel_rejection.value').value
        self.ahrs_mag_rejection = self.get_parameter('ahrs.mag_rejection.value').value
        self.ahrs_rejection_timeout = self.get_parameter('ahrs.rejection_timeout.value').value

        self.kf_initial_pos_uncertainty = self.get_parameter('kalman_filter.initial_pos_uncertainty.value').value
        self.kf_initial_vel_uncertainty = self.get_parameter('kalman_filter.initial_vel_uncertainty.value').value
        self.kf_initial_bias_uncertainty = self.get_parameter('kalman_filter.initial_bias_uncertainty.value').value
        self.kf_process_noise_pos = self.get_parameter('kalman_filter.process_noise_pos.value').value
        self.kf_process_noise_vel = self.get_parameter('kalman_filter.process_noise_vel.value').value
        self.kf_process_noise_bias = self.get_parameter('kalman_filter.process_noise_bias.value').value
        self.kf_measurement_noise_pos = self.get_parameter('kalman_filter.measurement_noise_pos.value').value
        self.kf_measurement_noise_vel = self.get_parameter('kalman_filter.measurement_noise_vel.value').value

        self.lp_filter_accel_alpha = self.get_parameter('lp_filters.accel_alpha.value').value

        self.earth_gravity = self.get_parameter('earth.gravity.value').value
        self.earth_radius = self.get_parameter('earth.radius.value').value
        self.earth_flattening = self.get_parameter('earth.flattening.value').value
        self.earth_ref_pos = self.get_parameter('earth.ref_pos.value').value

    def msg_to_sec(self, stamp):
        """Convert ROS timestamp to seconds."""
        return stamp.sec + stamp.nanosec * 1e-9

    def cb_mag(self, msg):
        """Magnetometer callback."""
        self.latest_mag = np.array([
            msg.magnetic_field.x, 
            msg.magnetic_field.y, 
            msg.magnetic_field.z
        ])

    def cb_gps_vel(self, msg: TwistWithCovarianceStamped):
        """Callback for GPS velocity - updates KF with velocity measurement."""
        gps_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.latest_gps_vel = gps_vel

        # Extract velocity covariance
        R = msg.twist.covariance
        R = np.array(R).reshape(6, 6)[:3, :3]
        R_vel = np.maximum(R, np.eye(3) * 0.1)  # (m/s)²
        
        # Update Kalman Filter with velocity measurement
        if self.kf_initialized:
            self.kf.update_velocity(gps_vel, R_vel)

    def cb_imu(self, msg):
        """IMU callback - runs prediction step at high rate."""
        if self.latest_mag is None:
            return
            
        t = self.msg_to_sec(msg.header.stamp)
        
        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        if dt <= 0:
            return

        # Extract IMU data
        gyro = np.degrees([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Apply low-pass filters
        accel = self.lp_filter_accel.update(accel)
        
        # Store raw gyro for bias estimation
        self.current_gyro = gyro.copy()
        
        # Update AHRS
        self.ahrs.update(gyro, accel, self.latest_mag, dt)
        
        # Get Earth Acceleration
        self.current_a_ned = (
            self.ahrs.earth_acceleration + 
            np.array([0, 0, self.earth_gravity])
        )
        
        # Kalman Filter Prediction Step
        if self.kf_initialized:
            self.kf.predict(self.current_a_ned, dt)
        
        self.last_imu_time = t
        
        # Publish odometry and TF
        self.publish_odometry(msg.header.stamp)

    def cb_gps_fix(self, msg: NavSatFix):
        """GPS position callback - updates KF with position measurement and logs data."""
        if np.isnan(msg.latitude) or self.latest_gps_vel is None:
            return
            
        if not self.kf_initialized:
            
            # Initialize Kalman Filter
            self.kf.initialize(np.zeros(3), self.latest_gps_vel)
            self.kf_initialized = True
            
            self.last_gps_vel = self.latest_gps_vel
            
            self.get_logger().info('Filter initialized with first GPS fix')
            return

        # GPS position in NED
        pos_gps = lla_to_ned(msg.latitude, msg.longitude, msg.altitude,
                                    self.earth_ref_pos, self.earth_radius)
        
        # Extract position covariance
        R = np.array(msg.position_covariance).reshape(3, 3)
        R_pos = np.maximum(R, np.eye(3) * 0.5)   # meters²
        
        # Kalman Filter Position Update (separate from velocity update)
        if self.kf_initialized:
            self.kf.update_position(pos_gps, R_pos)

    def publish_odometry(self, stamp):
        """Publish odometry message and TF transforms."""
        if not self.kf_initialized:
            return
        
        # Get state from Kalman Filter
        pos = self.kf.get_position()
        vel = self.kf.get_velocity()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame
        
        # Position (NED)
        odom.pose.pose.position.x = pos[0]  # (North)
        odom.pose.pose.position.y = pos[1]  # (East)
        odom.pose.pose.position.z = pos[2]  # (Down)
        
        # Position covariance (from Kalman Filter)
        P = self.kf.get_covariance()
        odom.pose.covariance[0]  = P[0, 0]  # x (North)
        odom.pose.covariance[7]  = P[1, 1]  # y (East)
        odom.pose.covariance[14] = P[2, 2]  # z (Down)
        
        # Orientation from AHRS (quaternion)
        q = self.ahrs.quaternion
        
        odom.pose.pose.orientation.x = q.x
        odom.pose.pose.orientation.y = q.y
        odom.pose.pose.orientation.z = q.z
        odom.pose.pose.orientation.w = q.w
        
        # Velocity (NED)
        odom.twist.twist.linear.x = vel[0]  # North
        odom.twist.twist.linear.y = vel[1]  # East
        odom.twist.twist.linear.z = vel[2]  # Down
        
        # Velocity covariance
        odom.twist.covariance[0]  = P[3, 3]  # vx (North)
        odom.twist.covariance[7]  = P[4, 4]  # vy (East)
        odom.twist.covariance[14] = P[5, 5]  # vz (Down)
        
        # Angular velocity from gyroscope (convert to rad/s, NED)
        gyro_rad = np.radians(self.current_gyro)
        odom.twist.twist.angular.x = float(gyro_rad[0])  # Roll rate
        odom.twist.twist.angular.y = float(gyro_rad[1])  # Pitch rate
        odom.twist.twist.angular.z = float(gyro_rad[2])  # Yaw rate
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish TF: odom -> base_link
        self.publish_tf(stamp, pos, q)
        
        # Optionally publish filtered acceleration
        if self.publish_acceleration:
            self.publish_filtered_imu(stamp)

    def publish_tf(self, stamp, pos_ned, q):
        """Publish TF transform from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        
        # Position (NED)
        t.transform.translation.x = pos_ned[0]  # North
        t.transform.translation.y = pos_ned[1]  # East
        t.transform.translation.z = pos_ned[2]  # Down
        
        # Orientation (already in NED from publish_odometry)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        
        self.tf_broadcaster.sendTransform(t)

    def publish_filtered_imu(self, stamp):
        """Publish filtered IMU data with gravity-removed acceleration."""
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.base_link_frame
        
        # Linear acceleration (gravity already removed by AHRS, convert NED to ENU)
        accel_ned = (
            self.ahrs.earth_acceleration + 
            np.array([0, 0, self.earth_gravity])
        ) - self.kf.x[6:9]
        imu_msg.linear_acceleration.x = float(accel_ned[0])  # North
        imu_msg.linear_acceleration.y = float(accel_ned[1])  # East
        imu_msg.linear_acceleration.z = float(accel_ned[2])  # Down
        
        # Angular velocity (convert from deg/s to rad/s)
        gyro_rad = np.radians(self.current_gyro)
        imu_msg.angular_velocity.x = float(gyro_rad[0])  # Roll rate
        imu_msg.angular_velocity.y = float(gyro_rad[1])  # Pitch rate
        imu_msg.angular_velocity.z = float(gyro_rad[2])  # Yaw rate
        
        # Orientation from AHRS
        q = self.ahrs.quaternion
        
        imu_msg.orientation.x = q.x
        imu_msg.orientation.y = q.y
        imu_msg.orientation.z = q.z
        imu_msg.orientation.w = q.w
        
        self.accel_pub.publish(imu_msg)

    def destroy(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = SensorFusionNode()

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