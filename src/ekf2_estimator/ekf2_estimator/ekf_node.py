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

from .ekf_core import LinearKalmanFilter, RobustLPFilter


class IMUGPSFusionNode(Node):
    """
    ROS2 Node for IMU/GPS integration using a Linear Kalman Filter.
    """
    
    def __init__(self):
        super().__init__('imu_gps_fusion_node')
        
        # Declare and load ROS2 parameters
        self.declare_all_parameters()
        self.load_parameters()
        
        # AHRS Setup
        self.offset = imufusion.Offset(self.params['ahrs']['offset_samples'])
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NED,
            self.params['ahrs']['gain'],
            self.params['ahrs']['gyro_range'],
            self.params['ahrs']['accel_rejection'],
            self.params['ahrs']['mag_rejection'],
            self.params['ahrs']['rejection_timeout']
        )

        # Kalman Filter for position/velocity estimation with bias
        self.kf = LinearKalmanFilter(self.params['kalman_filter'])
        self.kf_initialized = False
        
        # Simple integration state (for comparison)
        # self.state = np.zeros(6)  # [pn, pe, pd, vn, ve, vd]
        self.last_imu_time = None
        self.origin = None
        # self.current_a_ned = np.zeros(3)
        self.current_gyro = np.zeros(3)
        # self.current_euler = np.zeros(3)
        
        self.latest_mag = None
        self.latest_gps_vel = None
        # self.last_gps_vel = None
        # self.last_gps_t = None
        # self.last_published_time = None

        # ROS2 Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            self.params['topics']['odometry'],
            10
        )
        
        if self.params['topics']['publish_acceleration']:
            self.accel_pub = self.create_publisher(
                Imu,
                self.params['topics']['filtered_imu'],
                10
            )
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS2 Subscriptions
        self.create_subscription(
            Imu, 
            self.params['topics']['imu'], 
            self.cb_imu, 
            10
        )
        self.create_subscription(
            MagneticField, 
            self.params['topics']['mag'], 
            self.cb_mag, 
            10
        )
        self.create_subscription(
            NavSatFix, 
            self.params['topics']['gps_fix'], 
            self.cb_gps_fix, 
            10
        )
        self.create_subscription(
            TwistWithCovarianceStamped, 
            self.params['topics']['gps_vel'], 
            self.cb_gps_vel, 
            10
        )

        # Low-pass filters
        self.lp_filter_acc = RobustLPFilter(alpha=self.params['filters']['lowpass_alpha_acc'])
        
        self.get_logger().info('IMU/GPS Fusion Node initialized')
        self.log_parameters()

    def declare_all_parameters(self):
        """Declare all ROS2 parameters with default values."""
        
        # Frame IDs
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        
        # Topics
        self.declare_parameter('topics.imu', '/imu/data')
        self.declare_parameter('topics.mag', '/imu/mag')
        self.declare_parameter('topics.gps_fix', '/gps/fix')
        self.declare_parameter('topics.gps_vel', '/gps/vel')
        self.declare_parameter('topics.odometry', '/ekf/odometry')
        self.declare_parameter('topics.filtered_imu', '/ekf/filtered_imu')
        self.declare_parameter('topics.publish_acceleration', False)
        
        # AHRS
        self.declare_parameter('ahrs.offset_samples', 100)
        self.declare_parameter('ahrs.gain', 0.5)
        self.declare_parameter('ahrs.gyro_range', 500.0)
        self.declare_parameter('ahrs.accel_rejection', 10.0)
        self.declare_parameter('ahrs.mag_rejection', 10.0)
        self.declare_parameter('ahrs.rejection_timeout', 500)
        
        # Kalman Filter - Initial Uncertainties
        self.declare_parameter('kalman_filter.initial_pos_uncertainty', 100.0)
        self.declare_parameter('kalman_filter.initial_vel_uncertainty', 100.0)
        self.declare_parameter('kalman_filter.initial_bias_uncertainty', 1.0)
        
        # Kalman Filter - Process Noise
        self.declare_parameter('kalman_filter.process_noise_pos', 0.5)
        self.declare_parameter('kalman_filter.process_noise_vel', 0.5)
        self.declare_parameter('kalman_filter.process_noise_bias', 0.001)
        
        # Kalman Filter - Measurement Noise
        self.declare_parameter('kalman_filter.measurement_noise_pos', 25.0)
        self.declare_parameter('kalman_filter.measurement_noise_vel', 0.25)
        
        # Filters
        self.declare_parameter('filters.lowpass_alpha_acc', 0.9)
        
        # Earth Model
        self.declare_parameter('earth.gravity', 9.8066)
        self.declare_parameter('earth.radius', 6378137.0)
        self.declare_parameter('earth.flattening', 0.0033528106647474805)

    def load_parameters(self):
        """Load all ROS2 parameters into a nested dictionary structure."""
        self.params = {
            'frames': {
                'map': self.get_parameter('map_frame').value,
                'odom': self.get_parameter('odom_frame').value,
                'base_link': self.get_parameter('base_link_frame').value,
            },
            'topics': {
                'imu': self.get_parameter('topics.imu').value,
                'mag': self.get_parameter('topics.mag').value,
                'gps_fix': self.get_parameter('topics.gps_fix').value,
                'gps_vel': self.get_parameter('topics.gps_vel').value,
                'odometry': self.get_parameter('topics.odometry').value,
                'filtered_imu': self.get_parameter('topics.filtered_imu').value,
                'publish_acceleration': self.get_parameter('topics.publish_acceleration').value,
            },
            'ahrs': {
                'offset_samples': self.get_parameter('ahrs.offset_samples').value,
                'gain': self.get_parameter('ahrs.gain').value,
                'gyro_range': self.get_parameter('ahrs.gyro_range').value,
                'accel_rejection': self.get_parameter('ahrs.accel_rejection').value,
                'mag_rejection': self.get_parameter('ahrs.mag_rejection').value,
                'rejection_timeout': self.get_parameter('ahrs.rejection_timeout').value,
            },
            'kalman_filter': {
                'initial_pos_uncertainty': self.get_parameter('kalman_filter.initial_pos_uncertainty').value,
                'initial_vel_uncertainty': self.get_parameter('kalman_filter.initial_vel_uncertainty').value,
                'initial_bias_uncertainty': self.get_parameter('kalman_filter.initial_bias_uncertainty').value,
                'process_noise_pos': self.get_parameter('kalman_filter.process_noise_pos').value,
                'process_noise_vel': self.get_parameter('kalman_filter.process_noise_vel').value,
                'process_noise_bias': self.get_parameter('kalman_filter.process_noise_bias').value,
                'measurement_noise_pos': self.get_parameter('kalman_filter.measurement_noise_pos').value,
                'measurement_noise_vel': self.get_parameter('kalman_filter.measurement_noise_vel').value,
            },
            'filters': {
                'lowpass_alpha_acc': self.get_parameter('filters.lowpass_alpha_acc').value,
            },
            'earth': {
                'gravity': self.get_parameter('earth.gravity').value,
                'radius': self.get_parameter('earth.radius').value,
                'flattening': self.get_parameter('earth.flattening').value,
            }
        }

    def log_parameters(self):
        """Log key parameters for debugging."""
        self.get_logger().info('=== Configuration ===')
        self.get_logger().info(f"Frames: {self.params['frames']['odom']} -> {self.params['frames']['base_link']}")
        self.get_logger().info(f"IMU Topic: {self.params['topics']['imu']}")
        self.get_logger().info(f"GPS Fix Topic: {self.params['topics']['gps_fix']}")
        self.get_logger().info(f"GPS Vel Topic: {self.params['topics']['gps_vel']}")
        self.get_logger().info(f"Odometry Topic: {self.params['topics']['odometry']}")
        self.get_logger().info(f"Publish Filtered Accel: {self.params['topics']['publish_acceleration']}")
        self.get_logger().info(f"AHRS Gain: {self.params['ahrs']['gain']}")
        self.get_logger().info(f"KF Process Noise (pos): {self.params['kalman_filter']['process_noise_pos']}")
        self.get_logger().info(f"KF Measurement Noise (pos): {self.params['kalman_filter']['measurement_noise_pos']}")

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
        # Convert to NED frame
        gps_vel = np.array([
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.x,
            -msg.twist.twist.linear.z
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
        accel = self.lp_filter_acc.update(accel)
        
        # Store raw gyro for bias estimation
        self.current_gyro = gyro.copy()
        
        # Update AHRS
        self.ahrs.update(gyro, accel, self.latest_mag, dt)
        
        # Get Earth Acceleration
        self.current_a_ned = (
            self.ahrs.earth_acceleration + 
            np.array([0, 0, self.params['earth']['gravity']])
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
            
        if self.origin is None:
            # Initialize origin and states
            self.origin = [msg.latitude, msg.longitude, msg.altitude]
            
            # Initialize Kalman Filter
            self.kf.initialize(np.zeros(3), self.latest_gps_vel)
            self.kf_initialized = True
            
            self.last_gps_vel = self.latest_gps_vel
            
            self.get_logger().info('Filter initialized with first GPS fix')
            return

        # GPS position in NED
        pos_gps = self.lla_to_ned(msg.latitude, msg.longitude, msg.altitude)
        
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
        odom.header.frame_id = self.params['frames']['odom']
        odom.child_frame_id = self.params['frames']['base_link']
        
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
        if self.params['topics']['publish_acceleration']:
            self.publish_filtered_imu(stamp)

    def publish_tf(self, stamp, pos_ned, q):
        """Publish TF transform from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.params['frames']['odom']
        t.child_frame_id = self.params['frames']['base_link']
        
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
        imu_msg.header.frame_id = self.params['frames']['base_link']
        
        # Linear acceleration (gravity already removed by AHRS, convert NED to ENU)
        accel_ned = (
            self.ahrs.earth_acceleration + 
            np.array([0, 0, self.params['earth']['gravity']])
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

    def lla_to_ned(self, lat, lon, alt):
        """Convert latitude, longitude, altitude to local NED coordinates."""
        R = self.params['earth']['radius']
        f = self.params['earth']['flattening']
        e2 = 2*f - f**2
        
        lat_rad, lon_rad = np.radians(lat), np.radians(lon)
        lat0_rad = np.radians(self.origin[0])
        lon0_rad = np.radians(self.origin[1])
        
        N = R / np.sqrt(1 - e2 * np.sin(lat0_rad)**2)
        M = R * (1 - e2) / (1 - e2 * np.sin(lat0_rad)**2)**1.5
        
        return np.array([
            (lat_rad - lat0_rad) * (M + self.origin[2]),
            (lon_rad - lon0_rad) * (N + self.origin[2]) * np.cos(lat0_rad),
            -(alt - self.origin[2])
        ])

def main():
    rclpy.init()
    node = IMUGPSFusionNode()
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