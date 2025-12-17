import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

from .ekf_core import HybridEstimator
from .utils import lla_to_ned, quat_to_rot_mat, rotate_vector, quat_conjugate

class HybridEKFNode(Node):
    def __init__(self):
        super().__init__('ekf2_node')
        
        # Parameters
        self.load_parameters()
        
        # State
        self.estimator = HybridEstimator(dt_pred=0.01)
        self.estimator.Q_accel = self.accel_noise**2
        self.estimator.Q_gb = self.gyro_bias_rw**2
        self.estimator.Q_ab = self.accel_bias_rw**2
        self.estimator.Q_mb = self.mag_bias_rw**2
        
        # Configure AHRS
        self.estimator.set_ahrs_settings(
            gain=float(self.ahrs_gain),
            accel_rejection=float(self.ahrs_accel_rejection),
            mag_rejection=float(self.ahrs_mag_rejection)
        )
        
        # Initialization State
        self.origin_set = False
        self.origin_lla = None
        self.initialized = False
        
        # Sensor Data Buffer (for synchronization)
        self.latest_mag = None
        self.latest_gyro = None
        self.latest_accel = None
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_accel = self.create_publisher(AccelWithCovarianceStamped, 'accel/filtered', 10)
        self.pub_heading_est = self.create_publisher(Float64, 'heading/estimated', 10)
        self.pub_heading_gps = self.create_publisher(Float64, 'heading/gps', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.cb_mag, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/gps/vel', self.cb_gps_vel, 10)
        
        self.last_imu_time = 0.0
        self.get_logger().info("🚀 Hybrid EKF+AHRS Node Started. Waiting for GPS origin...")

    def load_parameters(self):
        # Frame IDs
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        
        # Noise Parameters
        self.declare_parameter('gyro_noise_density', 0.001)
        self.declare_parameter('accel_noise_density', 0.02)
        self.declare_parameter('gyro_bias_random_walk', 1.0e-5)
        self.declare_parameter('accel_bias_random_walk', 1.0e-4)
        self.declare_parameter('mag_bias_random_walk', 1.0e-5)
        
        # AHRS Parameters
        self.declare_parameter('ahrs_gain', 0.5)
        self.declare_parameter('ahrs_accel_rejection', 10.0)
        self.declare_parameter('ahrs_mag_rejection', 10.0)
        
        # Gate Thresholds
        self.declare_parameter('gate_pos_nis', 25.0)
        self.declare_parameter('gate_vel_nis', 25.0)
        self.declare_parameter('gate_heading_nis', 9.0)
        
        # GPS Heading
        self.declare_parameter('yaw_speed_threshold', 2.0)
        self.declare_parameter('gps_heading_noise', 0.1)
        
        # Magnetometer Reference
        self.declare_parameter('mag_ref_ned', [1.0, 0.0, 0.0])

        # Load values
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_link_frame').value
        
        self.gyro_noise = self.get_parameter('gyro_noise_density').value
        self.accel_noise = self.get_parameter('accel_noise_density').value
        self.gyro_bias_rw = self.get_parameter('gyro_bias_random_walk').value
        self.accel_bias_rw = self.get_parameter('accel_bias_random_walk').value
        self.mag_bias_rw = self.get_parameter('mag_bias_random_walk').value
        
        self.ahrs_gain = self.get_parameter('ahrs_gain').value
        self.ahrs_accel_rejection = self.get_parameter('ahrs_accel_rejection').value
        self.ahrs_mag_rejection = self.get_parameter('ahrs_mag_rejection').value
        
        self.gate_pos = self.get_parameter('gate_pos_nis').value
        self.gate_vel = self.get_parameter('gate_vel_nis').value
        self.gate_heading = self.get_parameter('gate_heading_nis').value
        
        self.yaw_speed_thresh = self.get_parameter('yaw_speed_threshold').value
        self.gps_heading_noise = self.get_parameter('gps_heading_noise').value
        
        self.mag_ref = np.array(self.get_parameter('mag_ref_ned').value)
        if np.linalg.norm(self.mag_ref) > 0:
            self.mag_ref = self.mag_ref / np.linalg.norm(self.mag_ref)  # Normalize
        
        self.get_logger().info(f"AHRS Gain: {self.ahrs_gain}, Accel Rejection: {self.ahrs_accel_rejection}°")
        self.get_logger().info(f"Mag Reference (NED): {self.mag_ref}")

    def cb_mag(self, msg):
        """Magnetometer Callback - Buffer for synchronization"""
        mag = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        
        # Normalize magnetometer reading
        mag_norm = np.linalg.norm(mag)
        if mag_norm > 0:
            self.latest_mag = mag / mag_norm
        else:
            self.get_logger().warn("Zero magnetometer reading", throttle_duration_sec=1.0)

    def cb_imu(self, msg):
        """IMU Callback - Main Prediction Loop"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract IMU data
        self.latest_gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.latest_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Wait for origin
        if not self.origin_set:
            self.get_logger().warn("Waiting for GPS to set origin...", throttle_duration_sec=3.0)
            return
        
        # Wait for magnetometer
        if self.latest_mag is None:
            self.get_logger().warn("Waiting for magnetometer data...", throttle_duration_sec=3.0)
            return
        
        # Initialize timestamp
        if self.last_imu_time == 0.0:
            self.last_imu_time = t
            self.initialized = True
            self.get_logger().info("✅ Estimator initialized and running")
            return
        
        # Calculate dt
        dt = t - self.last_imu_time
        if dt <= 0 or dt > 0.5:  # Sanity check
            self.get_logger().warn(f"Invalid dt={dt:.4f}s, resetting")
            self.last_imu_time = t
            return
        
        self.last_imu_time = t
        
        # Prediction Step
        try:
            self.estimator.predict(
                self.latest_gyro,
                self.latest_accel,
                self.latest_mag,
                dt
            )
        except Exception as e:
            self.get_logger().error(f"Prediction error: {e}")
            return
        
        # Check AHRS flags
        flags = self.estimator.get_flags()
        if flags['initialising']:
            self.get_logger().info("AHRS Initializing...", throttle_duration_sec=1.0)
        
        # Publish
        self.publish_odometry(msg.header.stamp, self.latest_accel)
        self.publish_heading()

    def cb_gps_fix(self, msg):
        """GPS Position Callback"""
        if msg.status.status < 0:
            self.get_logger().warn(f"GPS No Fix (Status {msg.status.status})", throttle_duration_sec=2.0)
            return
        
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        
        # Set origin on first valid fix
        if not self.origin_set:
            self.origin_lla = [lat, lon, alt]
            self.origin_set = True
            self.get_logger().info(f"🌍 Origin Set: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m")
            return
        
        if not self.initialized:
            return
        
        # Convert to NED
        pos_ned = lla_to_ned(lat, lon, alt, 
                             self.origin_lla[0], 
                             self.origin_lla[1], 
                             self.origin_lla[2])
        
        # Position covariance
        if msg.position_covariance[0] > 0:
            cov = np.diag([
                msg.position_covariance[0],
                msg.position_covariance[4],
                msg.position_covariance[8]
            ])
        else:
            cov = np.eye(3) * 1.0  # Default 1m std
        
        # Update
        success, nis = self.estimator.update_gps_pos_ned(pos_ned, cov, self.gate_pos)
        if not success:
            self.estimator.faults['gps_pos'] += 1
            self.get_logger().warn(
                f"GPS Pos Rejected | NIS={nis:.1f} | Faults={self.estimator.faults['gps_pos']}",
                throttle_duration_sec=1.0
            )
        else:
            if self.estimator.faults['gps_pos'] > 0:
                self.get_logger().info(f"GPS Pos Accepted | NIS={nis:.1f}")
            self.estimator.faults['gps_pos'] = 0

    def cb_gps_vel(self, msg):
        """GPS Velocity Callback"""
        if not self.initialized:
            return
        
        # Extract velocity (assumed NED frame)
        vel_ned = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Velocity covariance
        cov_full = np.array(msg.twist.covariance).reshape(6, 6)
        cov = cov_full[0:3, 0:3]
        
        if cov[0, 0] <= 0:
            cov = np.eye(3) * 0.1  # Default 0.1 m/s std
        
        # Update velocity
        success, nis = self.estimator.update_gps_vel_ned(vel_ned, cov, self.gate_vel)
        if not success:
            self.estimator.faults['gps_vel'] += 1
            self.get_logger().warn(
                f"GPS Vel Rejected | NIS={nis:.1f}",
                throttle_duration_sec=1.0
            )
        else:
            self.estimator.faults['gps_vel'] = 0
        
        # GPS Heading Update (from velocity)
        speed_2d = np.linalg.norm(vel_ned[0:2])
        if speed_2d > self.yaw_speed_thresh:
            heading_gps = np.arctan2(vel_ned[1], vel_ned[0])
            
            # Publish GPS heading for comparison
            heading_msg = Float64()
            heading_msg.data = np.rad2deg(heading_gps)
            self.pub_heading_gps.publish(heading_msg)
            
            # Heading noise scales with velocity uncertainty
            sigma_v = np.sqrt(max(cov[0, 0], cov[1, 1]))
            sigma_heading = max(sigma_v / speed_2d, self.gps_heading_noise)
            
            success_h, nis_h = self.estimator.update_gps_heading(
                heading_gps, 
                self.gate_heading, 
                sigma_heading
            )
            
            if not success_h:
                self.estimator.faults['gps_heading'] += 1
                if self.estimator.faults['gps_heading'] % 10 == 0:
                    self.get_logger().warn(
                        f"GPS Heading Rejected | NIS={nis_h:.1f} | Faults={self.estimator.faults['gps_heading']}",
                        throttle_duration_sec=2.0
                    )
            else:
                self.estimator.faults['gps_heading'] = 0

    def publish_heading(self):
        """Publish estimated heading for diagnostics"""
        euler = self.estimator.get_euler()  # Returns [roll, pitch, yaw] in degrees
        heading_msg = Float64()
        heading_msg.data = float(euler[2])  # Yaw in degrees
        self.pub_heading_est.publish(heading_msg)

    def publish_odometry(self, stamp, raw_accel):
        """Publish Odometry and TF"""
        # Get state
        pos = self.estimator.x[0:3]
        vel = self.estimator.x[3:6]
        q = self.estimator.get_quaternion()
        
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        
        # Orientation
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        
        # Velocity (convert NED to Body frame)
        q_inv = quat_conjugate(q)
        v_body = rotate_vector(vel, q_inv)
        
        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.linear.z = float(v_body[2])
        
        # Covariance
        P = self.estimator.P
        R_nb = quat_to_rot_mat(q)  # NED to Body
        R_bn = R_nb.T  # Body to NED
        
        # Pose covariance (in NED frame)
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = P[0:3, 0:3]  # Position
        # Attitude uncertainty from AHRS (approximate)
        pose_cov[3:6, 3:6] = np.eye(3) * (np.radians(2.0))**2  # ~2 degree std
        odom.pose.covariance = pose_cov.flatten().tolist()
        
        # Twist covariance (in Body frame)
        P_vel_ned = P[3:6, 3:6]
        P_vel_body = R_nb @ P_vel_ned @ R_nb.T
        
        twist_cov = np.zeros((6, 6))
        twist_cov[0:3, 0:3] = P_vel_body
        twist_cov[3:6, 3:6] = P[6:9, 6:9]  # Gyro bias
        odom.twist.covariance = twist_cov.flatten().tolist()
        
        self.pub_odom.publish(odom)
        
        # TF: map -> odom (identity for now)
        t_map = TransformStamped()
        t_map.header.stamp = stamp
        t_map.header.frame_id = self.map_frame
        t_map.child_frame_id = self.odom_frame
        t_map.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map)
        
        # TF: odom -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = stamp
        t_base.header.frame_id = self.odom_frame
        t_base.child_frame_id = self.base_frame
        t_base.transform.translation.x = pos[0]
        t_base.transform.translation.y = pos[1]
        t_base.transform.translation.z = pos[2]
        t_base.transform.rotation.w = q[0]
        t_base.transform.rotation.x = q[1]
        t_base.transform.rotation.y = q[2]
        t_base.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(t_base)
        
        # Filtered Acceleration (Body frame, gravity removed)
        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self.base_frame
        
        g_ned = np.array([0.0, 0.0, -9.80665])
        g_body = rotate_vector(g_ned, q_inv)
        
        ba = self.estimator.x[9:12]
        a_linear_body = (raw_accel - ba) - g_body
        
        accel_msg.accel.accel.linear.x = a_linear_body[0]
        accel_msg.accel.accel.linear.y = a_linear_body[1]
        accel_msg.accel.accel.linear.z = a_linear_body[2]
        
        # Covariance
        accel_cov = np.zeros((6, 6))
        accel_cov[0:3, 0:3] = P[9:12, 9:12]  # Accel bias uncertainty
        accel_msg.accel.covariance = accel_cov.flatten().tolist()
        
        self.pub_accel.publish(accel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridEKFNode()
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