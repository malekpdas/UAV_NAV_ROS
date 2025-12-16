import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import time

from ekf2_estimator.ekf_core import EKF2Core
from ekf2_estimator.utils import lla_to_ned, euler_to_quat

class EKF2Node(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # Parameters
        self.load_parameters()
        
        # State
        self.ekf = EKF2Core(dt_pred=0.01)
        self.ekf.Q_gyro = self.gyro_noise**2
        self.ekf.Q_accel = self.accel_noise**2
        self.ekf.Q_gb = self.gyro_bias_rw**2
        self.ekf.Q_ab = self.accel_bias_rw**2
        self.ekf.Q_mb = self.mag_bias_rw**2
        
        # Initialization State
        self.origin_set = False
        self.origin_lla = None # [lat, lon, alt]
        self.imu_aligned = False
        self.imu_buffer = []
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_accel = self.create_publisher(AccelWithCovarianceStamped, 'accel/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.cb_mag, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/gps/vel', self.cb_gps_vel, 10)
        
        # Timer
        # self.create_timer(0.01, self.tick) # Actually we drive on IMU callback
        self.last_imu_time = 0.0

        self.get_logger().info("EKF2 Node Initiated. Waiting for GPS...")

    def load_parameters(self):
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        
        self.declare_parameter('gyro_noise_density', 0.001)
        self.declare_parameter('accel_noise_density', 0.02)
        self.declare_parameter('gyro_bias_random_walk', 1.0e-5)
        self.declare_parameter('accel_bias_random_walk', 1.0e-4)
        self.declare_parameter('mag_bias_random_walk', 1.0e-5)
        
        self.declare_parameter('gate_pos_nis', 25.0)
        self.declare_parameter('gate_vel_nis', 25.0)
        self.declare_parameter('gate_mag_nis', 25.0)
        self.declare_parameter('gate_yaw_nis', 25.0)
        
        self.declare_parameter('yaw_speed_threshold', 2.0)
        self.declare_parameter('mag_ref_ned', [1.0, 0.0, 0.0])

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_link_frame').value
        
        self.gyro_noise = self.get_parameter('gyro_noise_density').value
        self.accel_noise = self.get_parameter('accel_noise_density').value
        self.gyro_bias_rw = self.get_parameter('gyro_bias_random_walk').value
        self.accel_bias_rw = self.get_parameter('accel_bias_random_walk').value
        self.mag_bias_rw = self.get_parameter('mag_bias_random_walk').value
        
        self.gate_pos = self.get_parameter('gate_pos_nis').value
        self.gate_vel = self.get_parameter('gate_vel_nis').value
        self.gate_mag = self.get_parameter('gate_mag_nis').value
        self.gate_yaw = self.get_parameter('gate_yaw_nis').value
        
        self.yaw_speed_thresh = self.get_parameter('yaw_speed_threshold').value
        
        self.mag_ref = self.get_parameter('mag_ref_ned').value
        self.get_logger().info(f"Mag Ref (True NED): {self.mag_ref}")

    def cb_imu(self, msg):
        # Debug Log
        # self.get_logger().info("IMU Callback", throttle_duration_sec=1.0)
        
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        # Update process noise from message covariance if available
        # Covariance is row-major 3x3 (9 elements)
        if msg.angular_velocity_covariance[0] > 0:
            # Assume diagonal, take max or average of diagonal
            cov_diag = [msg.angular_velocity_covariance[0], msg.angular_velocity_covariance[4], msg.angular_velocity_covariance[8]]
            noise_var = np.mean(cov_diag)
            # Q is noise variance. 
            self.ekf.Q_gyro = noise_var

        if msg.linear_acceleration_covariance[0] > 0:
            cov_diag = [msg.linear_acceleration_covariance[0], msg.linear_acceleration_covariance[4], msg.linear_acceleration_covariance[8]]
            noise_var = np.mean(cov_diag)
            self.ekf.Q_accel = noise_var

        if not self.origin_set or not self.imu_aligned:
             if not self.origin_set:
                 self.get_logger().warn("Waiting for GPS Fix to set Local Origin...", throttle_duration_sec=3.0)
             elif not self.imu_aligned:
                 self.get_logger().info(f"Origin Set. Attempting Alignment with Accel: {accel}")
                 self.do_initial_alignment(accel)
             return

        if self.last_imu_time == 0.0:
            self.get_logger().info("Resetting IMU timestamp")
            self.last_imu_time = t
            return


        dt = t - self.last_imu_time
        if dt <= 0: return # Skip duplicate/backward
        
        self.last_imu_time = t
        
        # Prediction
        self.ekf.predict(gyro, accel, dt)
        
        # Publish
        self.publish_odometry(msg.header.stamp, accel)

    def cb_gps_fix(self, msg):
        if msg.status.status < 0: 
            self.get_logger().warn(f"GPS No Fix (Status {msg.status.status})", throttle_duration_sec=1.0)
            return
        
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        
        if not self.origin_set:
            self.origin_lla = [lat, lon, alt]
            self.origin_set = True
            self.get_logger().info(f"🌍 Global Origin Set: {lat}, {lon}, {alt}")
            return

        if not self.imu_aligned: return

        # Update
        pos_ned = lla_to_ned(lat, lon, alt, self.origin_lla[0], self.origin_lla[1], self.origin_lla[2])
        
        # Covariance (Diagonal approximation)
        cov = np.diag([msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8]])
        
        success, nis = self.ekf.update_gps_pos_ned(pos_ned, cov, self.gate_pos)
        if not success:
            self.get_logger().warn(f"GPS Pos Rejected NIS={nis:.1f}")

    def cb_gps_vel(self, msg):
        if not self.imu_aligned: return
        
        vel_ned = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) # Assuming NED 
        # CAUTION: Ensure Topic provides NED! Prompt says "Linear velocity in NED".
        
        # Covariance (6x6). Take top left 3x3 for linear
        cov = np.array(msg.twist.covariance).reshape(6,6)[0:3, 0:3]
        
        success, nis = self.ekf.update_gps_vel_ned(vel_ned, cov, self.gate_vel)
        if not success:
             self.get_logger().warn(f"GPS Vel Rejected NIS={nis:.1f}")
        
        # Conditional Yaw Update
        # speed2d = np.linalg.norm(vel_ned[0:2])
        # if speed2d > self.yaw_speed_thresh:
        #     yaw_gps = np.arctan2(vel_ned[1], vel_ned[0])
        #     # Use velocity variance approx for yaw noise? 
        #     # sigma_yaw ~= sigma_vel / speed
        #     sigma_v = np.sqrt(cov[0,0]) # Approx
        #     sigma_yaw = sigma_v / speed2d
            
        #     suc_y, nis_y = self.ekf.update_gps_yaw_ned(yaw_gps, self.gate_yaw, sigma_yaw)
        #     if not suc_y:
        #          self.get_logger().warn(f"GPS Yaw Rejected NIS={nis_y:.1f}")

    def cb_mag(self, msg):
        if not self.imu_aligned: return
        
        mag_body = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        cov = np.diag([1e9, 1e-2, 1e9]) # Default
        
        # Check msg covariance
        if msg.magnetic_field_covariance[0] > 0:
             cov = np.array(msg.magnetic_field_covariance).reshape(3,3)
        
        success, nis = self.ekf.update_mag(mag_body, cov, self.gate_mag, self.mag_ref)
        if not success:
             # self.get_logger().warn(f"Mag Rejected NIS={nis:.1f}")
             pass

    def do_initial_alignment(self, accel):
        """
        Align R/P to gravity.
        accel measures Reaction Force (Upwards).
        In NED, Gravity Vector is Down (+Z).
        Reaction Force for level flight is Up (-Z).
        So Down Vector = -accel.
        """
        
        # Pitch: atan2(x, sqrt(y^2+z^2))
        # Roll: atan2(y, z)
        
        pitch = np.arctan2(accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        roll = np.arctan2(accel[1], -accel[2])
        
        # Initial Yaw? From Mag if available. For now 0.
        yaw = 0.0
        
        q_init = euler_to_quat(roll, pitch, yaw)
        self.ekf.x[6:10] = q_init
        self.imu_aligned = True
        self.get_logger().info(f"✅ EKF Initialized. R/P: {np.rad2deg(roll):.1f}/{np.rad2deg(pitch):.1f}")

    def publish_odometry(self, stamp, raw_accel):
        # 1. Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.ekf.x[0]
        odom.pose.pose.position.y = self.ekf.x[1]
        odom.pose.pose.position.z = self.ekf.x[2]
        
        # Orientation
        odom.pose.pose.orientation.w = self.ekf.x[6]
        odom.pose.pose.orientation.x = self.ekf.x[7]
        odom.pose.pose.orientation.y = self.ekf.x[8]
        odom.pose.pose.orientation.z = self.ekf.x[9]
        
        # Velocity (in Body or Nav? Odometry Twist is usually Body).
        # EKF State is Vel NED (Nav).
        # We should rotate Vel to Body for Twist message
        v_ned = self.ekf.x[3:6]
        q = self.ekf.x[6:10]
        
        # Rotate NED velocity to Body: v_b = R_nav_to_body * v_n = R(q)^T * v_n 
        # My rotate_vector(v, q) does q*v*q_inv (Body -> Nav).
        # So I need q_inv * v * q.
        from ekf2_estimator.utils import quat_conjugate, rotate_vector
        q_inv = quat_conjugate(q)
        v_body = rotate_vector(v_ned, q_inv)
        
        odom.twist.twist.linear.x = v_body[0]
        odom.twist.twist.linear.y = v_body[1]
        odom.twist.twist.linear.z = v_body[2]
        
        # --- Covariance ---
        P = self.ekf.P
        
        # 1. Pose Covariance (6x6) in Frame "odom_frame" (NED)
        # Position Error (0:3) is already in NED.
        # Attitude Error (6:9) is in Body (based on Error State Mechanization).
        # We need Attitude Error in NED.
        # dTheta_ned = R_body_to_nav * dTheta_body = R(q)^T * dTheta_body??
        # Usually: w_nav = R * w_body. So errors map similarly.
        # q is NED to Body. So R(q) is NED to Body. R(q)^T is Body to NED.
        # So dTheta_ned = q_inv * dTheta_body * q
        
        from ekf2_estimator.utils import quat_to_rot_mat
        
        q_nb = self.ekf.x[6:10] # NED to Body
        R_nb = quat_to_rot_mat(q_nb) # NED to Body
        R_bn = R_nb.T # Body to NED
        
        P_pos_ned = P[0:3, 0:3]
        P_att_body = P[6:9, 6:9]
        
        # Cov_att_ned = R_bn * Cov_att_body * R_bn^T
        P_att_ned = R_bn @ P_att_body @ R_bn.T
        
        # Populate pose_cov (Row-major 6x6)
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = P_pos_ned
        pose_cov[3:6, 3:6] = P_att_ned
        # Ignoring cross-terms for simplicity/safety from rotation mismatch
        
        odom.pose.covariance = pose_cov.flatten().tolist()
        
        # 2. Twist Covariance (6x6) in Child Frame "base_frame" (Body)
        # Velocity Error (3:6) is in NED.
        # Gyro Bias Error (9:12) is in Body.
        # Twist Linear = v_body. Error = R_nb * v_ned ...
        # d(v_body) = R_nb * d(v_ned) + d(R_nb) * v_ned ... gets complex.
        # Approximation: Just rotate variances.
        
        P_vel_ned = P[3:6, 3:6]
        P_vel_body = R_nb @ P_vel_ned @ R_nb.T
        
        P_bg = P[9:12, 9:12] # Gyro Bias Error
        
        twist_cov = np.zeros((6, 6))
        twist_cov[0:3, 0:3] = P_vel_body
        twist_cov[3:6, 3:6] = P_bg 
        
        odom.twist.covariance = twist_cov.flatten().tolist()
        
        self.pub_odom.publish(odom)
        
        # 2. TF Broadcasting
        # map -> odom (Static - assuming aligned origins for now)
        t_map = TransformStamped()
        t_map.header.stamp = stamp
        t_map.header.frame_id = self.map_frame
        t_map.child_frame_id = self.odom_frame
        t_map.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map)
        
        # odom -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = stamp
        t_base.header.frame_id = self.odom_frame
        t_base.child_frame_id = self.base_frame
        t_base.transform.translation.x = self.ekf.x[0]
        t_base.transform.translation.y = self.ekf.x[1]
        t_base.transform.translation.z = self.ekf.x[2]
        t_base.transform.rotation.w = self.ekf.x[6]
        t_base.transform.rotation.x = self.ekf.x[7]
        t_base.transform.rotation.y = self.ekf.x[8]
        t_base.transform.rotation.z = self.ekf.x[9]
        self.tf_broadcaster.sendTransform(t_base)

        # 3. Accel Filtered
        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self.base_frame
        
        # a_body = accel_meas - bias + g_body
        # q_inv rotates Nav -> Body
        g_ned = np.array([0.0, 0.0, 9.80665])
        g_body = rotate_vector(g_ned, q_inv)
        
        ba = self.ekf.x[13:16]
        a_linear_body = (raw_accel - ba) + g_body
        
        accel_msg.accel.accel.linear.x = a_linear_body[0]
        accel_msg.accel.accel.linear.y = a_linear_body[1]
        accel_msg.accel.accel.linear.z = a_linear_body[2]
        
        self.pub_accel.publish(accel_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EKF2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
