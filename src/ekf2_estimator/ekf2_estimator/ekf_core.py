import numpy as np
from ekf2_estimator.utils import quat_mult, quat_normalize, rotate_vector, euler_to_quat, quat_conjugate, wrap_pi

class EKF2Core:
    def __init__(self, dt_pred=0.01):
        # Configuration
        self.dt = dt_pred
        
        # State Vector (19)
        # 0-2: Pos (N, E, D)
        # 3-5: Vel (N, E, D)
        # 6-9: Quat(w, x, y, z) (NED to Body)
        # 10-12: Gyro Bias
        # 13-15: Accel Bias
        # 16-18: Mag Bias
        self.x = np.zeros(19)
        self.x[6] = 1.0 # Init Quaternion to identity [1,0,0,0]
        
        # Error State Covariance (18x18)
        # 0-2: dPos
        # 3-5: dVel
        # 6-8: dTheta (Attitude Error)
        # 9-11: dGb
        # 12-14: dAb
        # 15-17: dMb
        self.P = np.eye(18) * 1e-2
        
        # Tuning Parameters (Set by Node)
        self.Q_gyro = 1e-3
        self.Q_accel = 1e-2
        self.Q_gb = 1e-5
        self.Q_ab = 1e-4
        self.Q_mb = 1e-5
        
        # Constants
        self.g = np.array([0, 0, 9.80665])
        
        # Fault Counters
        self.faults = {
            'gps_pos': 0,
            'gps_vel': 0,
            'mag_yaw': 0,
            'gps_yaw': 0
        }

    def predict(self, gyro, accel, dt):
        """
        Strapdown INS Prediction Step
        gyro: raw rad/s
        accel: raw m/s^2
        """
        self.dt = dt
        
        # 1. Unpack State
        pos = self.x[0:3]
        vel = self.x[3:6]
        q = self.x[6:10]
        bg = self.x[10:13]
        ba = self.x[13:16]
        bm = self.x[16:19]
        
        # 2. Correct Measurements
        omega = gyro - bg
        a_body = accel - ba
        
        # 3. Attitude Propagation (Quaternion Kinematics)
        # dq/dt = 0.5 * q * Omega
        # Small angle approximation for speed
        angle = np.linalg.norm(omega) * dt
        if angle > 1e-6:
             axis = omega / np.linalg.norm(omega)
             sin_half = np.sin(angle/2)
             cos_half = np.cos(angle/2)
             dq = np.array([cos_half, axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half])
        else:
             dq = np.array([1.0, 0.5*omega[0]*dt, 0.5*omega[1]*dt, 0.5*omega[2]*dt])
             dq = dq / np.linalg.norm(dq)
             
        q_new = quat_mult(q, dq)
        q_new = quat_normalize(q_new)
        
        # 4. Rotate Accel to NED
        # R_nb = R(q)
        # a_ned = R_nb * a_body
        a_ned = rotate_vector(a_body, q)
        
        # 5. Velocity Integration
        vel_new = vel + (a_ned + self.g) * dt
        
        # 6. Position Integration
        pos_new = pos + vel * dt + 0.5 * (a_ned + self.g) * dt**2
        
        # 7. Update State
        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        self.x[6:10] = q_new
        # Biases stay constant (random walk added in Covariance)
        
        # 8. Covariance Propagation (Process Noise)
        # Jacobian F (18x18)
        F = np.eye(18)
        
        # Pos w.r.t Vel
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Vel w.r.t Att (Cross product with accel)
        # d(R*a)/d(theta) approx -R * [a_body x]
        # Construct skew symmetric matrix for a_body
        ax, ay, az = a_body
        S_a = np.array([
            [0, -az, ay],
            [az, 0, -ax],
            [-ay, ax, 0]
        ])
        
        # Rotation matrix from q
        # We need R_nb (Body to Nav). 
        # rotate_vector does q * v * q_inv.
        # This is strictly R(q).
        
        # F[3:6, 6:9] = - R * S_a * dt
        # R * v is just rotate_vector(v, q)
        # Let's col-wise rotate S_a
        col0 = rotate_vector(S_a[:, 0], q)
        col1 = rotate_vector(S_a[:, 1], q)
        col2 = rotate_vector(S_a[:, 2], q)
        mat_R_Sa = np.column_stack((col0, col1, col2))
        
        F[3:6, 6:9] = -mat_R_Sa * dt
        
        # Vel w.r.t Accel Bias
        # d(v)/d(ba) = -R * dt
        # -R is rotation of negative identity
        r_col0 = rotate_vector(np.array([-1,0,0]), q)
        r_col1 = rotate_vector(np.array([0,-1,0]), q)
        r_col2 = rotate_vector(np.array([0,0,-1]), q)
        mat_nR = np.column_stack((r_col0, r_col1, r_col2))
        
        F[3:6, 12:15] = mat_nR * dt
        
        # Att w.r.t Att
        # Identity - [omega x] * dt
        wx, wy, wz = omega
        S_w = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])
        F[6:9, 6:9] = np.eye(3) - S_w * dt
        
        # Att w.r.t Gyro Bias
        # -Identity * dt
        F[6:9, 9:12] = -np.eye(3) * dt
        
        # Q matrix (Process Noise)
        # Simplified diagonal approximation scaled by dt
        # V_i = [input noise (gyro, accel), bias random walk]
        # Scale by dt^2 or dt depending on order
        Q = np.zeros((18, 18))
        Q[0:3, 0:3] = 0 # Pos noise from vel
        Q[3:6, 3:6] = np.eye(3) * self.Q_accel * dt**2 
        Q[6:9, 6:9] = np.eye(3) * self.Q_gyro * dt**2
        Q[9:12, 9:12] = np.eye(3) * self.Q_gb * dt
        Q[12:15, 12:15] = np.eye(3) * self.Q_ab * dt
        Q[15:18, 15:18] = np.eye(3) * self.Q_mb * dt
        
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T) # Symmetrize

    def update(self, z, H, R, gate_threshold):
        """
        Generic EKF Update
        z: measurement residual (innovation)
        H: Jacobian
        R: Measurement Noise
        gate_threshold: NIS threshold
        """
        # S = H P H.T + R
        S = H @ self.P @ H.T + R
        
        # NIS
        # y.T * S_inv * y
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            print("Singular matrix in update steps")
            return False, 0.0

        nis = z.T @ S_inv @ z
        
        if nis > gate_threshold:
            return False, nis
        
        # Kalman Gain K = P H.T S_inv
        K = self.P @ H.T @ S_inv
        
        # State Correction dx = K * z
        dx = K @ z
        
        # Apply Correction
        # Pos, Vel
        self.x[0:6] += dx[0:6]
        
        # Attitude (Multiplicative)
        # dx_theta is rotation vector
        d_theta = dx[6:9]
        angle = np.linalg.norm(d_theta)
        if angle > 1e-6:
             axis = d_theta / angle
             sin_h = np.sin(angle/2)
             cos_h = np.cos(angle/2)
             dq_corr = np.array([cos_h, axis[0]*sin_h, axis[1]*sin_h, axis[2]*sin_h])
        else:
             dq_corr = np.array([1.0, 0.5*d_theta[0], 0.5*d_theta[1], 0.5*d_theta[2]])
             dq_corr /= np.linalg.norm(dq_corr)
             
        self.x[6:10] = quat_mult(self.x[6:10], dq_corr)
        self.x[6:10] = quat_normalize(self.x[6:10])
        
        # Biases
        self.x[10:19] += dx[9:18]
        
        # Covariance Update
        # P = (I - K H) P
        I = np.eye(18)
        self.P = (I - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        
        return True, nis

    # --- Specific Updates ---

    def update_gps_pos_ned(self, pos_ned, cov, gate):
        # Measurement: p_ned
        # Model: p_est (State 0:3)
        # H: [I3 0 ...]
        z = pos_ned - self.x[0:3]
        H = np.zeros((3, 18))
        H[0:3, 0:3] = np.eye(3)
        R = cov if cov is not None else np.eye(3)
        
        return self.update(z, H, R, gate)

    def update_gps_vel_ned(self, vel_ned, cov, gate):
        # Measurement: v_ned
        # Model: v_est (State 3:6)
        z = vel_ned - self.x[3:6]
        H = np.zeros((3, 18))
        H[0:3, 3:6] = np.eye(3)
        R = cov if cov is not None else np.eye(3)
        
        return self.update(z, H, R, gate)

    def update_mag(self, mag_body_meas, R_cov, gate, mag_ref_ned):
        """
        Magnetometer Update (3D Vector Matching)
        Model: m_body = R(q)^T * (m_ref_ned) + b_mag
        """

        # check mag magnitude
        if abs(np.linalg.norm(mag_body_meas)-1.0) > 0.15:
            return False, None

        # Predicted Measurement
        q = self.x[6:10]
        b_mag = self.x[16:19]
        
        # R_bn = R(q). We need R_nb = R(q)^T to rotate NED to Body.
        # rotate_vector does q * v * q_inv -> Body to Nav if q is q_nb
        # BUT self.x[6:10] is q_nb (NED to Body? No, typically q_nb means Body to Nav in strapdown?
        # Let's check predict: a_ned = rotate_vector(a_body, q).
        # This implies "q" rotates from Body Frame to NED Frame. (q_bn)
        # So q is Body->NED.
        # To rotate NED->Body, we use q_conjugate.
        
        q_nb = quat_conjugate(q) # NED to Body
        m_body_pred = rotate_vector(mag_ref_ned, q_nb) + b_mag
        
        z = mag_body_meas - m_body_pred
        
        # Jacobian H (3x18)
        H = np.zeros((3, 18))
        
        # H w.r.t Att Error (dTheta)
        # d(m_body)/d(theta) approx [m_body_ideal x]
        # m_body_ideal = R_nb * m_ref (without bias)
        m_ideal = m_body_pred - b_mag
        mx, my, mz = m_ideal
        S_m = np.array([
            [0, -mz, my],
            [mz, 0, -mx],
            [-my, mx, 0]
        ])
        H[0:3, 6:9] = S_m
        
        # H w.r.t Mag Bias (Identity)
        H[0:3, 15:18] = np.eye(3)
        
        return self.update(z, H, R_cov, gate)
        
    def update_gps_yaw_ned(self, measurement_yaw, gate, meas_noise):
        """
        1D Yaw update (GPS Ground Track)
        Measurement is in NED (North=0, East=90)
        """
        # Predicted Yaw from Quaternion
        q = self.x[6:10]
        # Convert q to Euler (Yaw is 3rd element)
        _, _, yaw_est = quat_to_euler(q)
        
        # Residual with wrap
        res = measurement_yaw - yaw_est
        res = wrap_pi(res)
        z = np.array([res])
        
        # Jacobian H (1x18). Only dTheta_z matters
        # d(yaw)/d(theta_z) = 1
        H = np.zeros((1, 18))
        H[0, 8] = 1.0 
        
        R = np.array([[meas_noise**2]])
        
        return self.update(z, H, R, gate)
