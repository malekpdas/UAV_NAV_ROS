import numpy as np
import imufusion
from ekf2_estimator.utils import quat_mult, quat_normalize, rotate_vector, quat_to_euler, quat_conjugate, wrap_pi

class HybridEstimator:
    """
    Hybrid Estimator: imufusion AHRS + EKF
    - AHRS (Madgwick): Attitude estimation from IMU + Mag
    - EKF: Position, Velocity, and Bias estimation
    """
    def __init__(self, dt_pred=0.01):
        # Configuration
        self.dt = dt_pred
        
        # AHRS (Madgwick Algorithm)
        self.ahrs = imufusion.Ahrs()
        
        # Settings - recovery_trigger_period is in SAMPLES, not seconds
        # 5 seconds at 100Hz = 500 samples
        recovery_samples = int(5.0 / dt_pred)  # Convert seconds to samples
        
        self.ahrs_settings = imufusion.Settings(
            imufusion.CONVENTION_NED,  # NED convention
            0.5,  # gain
            500.0,  # gyroscope range (deg/s)
            10.0,   # acceleration rejection (deg)
            10.0,   # magnetic rejection (deg)
            recovery_samples  # recovery trigger period (samples) - MUST BE INT
        )
        
        # Apply settings
        self.ahrs.settings = self.ahrs_settings
        
        # EKF State Vector (15)
        # 0-2: Position (N, E, D)
        # 3-5: Velocity (N, E, D)
        # 6-8: Gyro Bias (rad/s)
        # 9-11: Accel Bias (m/s^2)
        # 12-14: Mag Bias (uT or normalized)
        self.x = np.zeros(15)
        
        # Error State Covariance (15x15)
        self.P = np.eye(15) * 1e-2
        
        # Tuning Parameters
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
            'gps_heading': 0
        }

    def predict(self, gyro, accel, mag, dt):
        """
        Prediction Step:
        1. Update AHRS for attitude
        2. Propagate EKF state (pos, vel) using current attitude
        """
        self.dt = dt
        
        # 1. Unpack State
        pos = self.x[0:3]
        vel = self.x[3:6]
        bg = self.x[6:9]
        ba = self.x[9:12]
        bm = self.x[12:15]
        
        # 2. Correct Measurements
        gyro_corrected = gyro - bg
        accel_corrected = accel - ba
        mag_corrected = mag - bm
        
        # 3. Update AHRS (Madgwick)
        # imufusion expects deg/s for gyro
        gyro_deg = np.rad2deg(gyro_corrected)
        
        # Ensure inputs are contiguous numpy arrays (imufusion requirement)
        gyro_deg = np.ascontiguousarray(gyro_deg, dtype=np.float32)
        accel_corrected = np.ascontiguousarray(accel_corrected, dtype=np.float32)
        mag_corrected = np.ascontiguousarray(mag_corrected, dtype=np.float32)
        
        self.ahrs.update(
            gyro_deg,
            accel_corrected,
            mag_corrected,
            dt
        )
        
        # 4. Get Quaternion from AHRS
        q_ahrs = self.ahrs.quaternion
        q = np.array([q_ahrs.w, q_ahrs.x, q_ahrs.y, q_ahrs.z])
        
        # 5. Rotate Accel to NED
        a_ned = rotate_vector(accel_corrected, q)
        
        # 6. Velocity Integration
        vel_new = vel + (a_ned + self.g) * dt
        
        # 7. Position Integration
        pos_new = pos + vel * dt + 0.5 * (a_ned + self.g) * dt**2
        
        # 8. Update EKF State
        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        # Biases stay constant (random walk added in covariance)
        
        # 9. Covariance Propagation
        F = np.eye(15)
        
        # Pos w.r.t Vel
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Vel w.r.t Accel Bias
        # d(v)/d(ba) = -R * dt
        r_col0 = rotate_vector(np.array([-1, 0, 0]), q)
        r_col1 = rotate_vector(np.array([0, -1, 0]), q)
        r_col2 = rotate_vector(np.array([0, 0, -1]), q)
        mat_nR = np.column_stack((r_col0, r_col1, r_col2))
        F[3:6, 9:12] = mat_nR * dt
        
        # Process Noise
        Q = np.zeros((15, 15))
        Q[3:6, 3:6] = np.eye(3) * self.Q_accel * dt
        Q[6:9, 6:9] = np.eye(3) * self.Q_gb * dt
        Q[9:12, 9:12] = np.eye(3) * self.Q_ab * dt
        Q[12:15, 12:15] = np.eye(3) * self.Q_mb * dt
        
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)  # Symmetrize

    def update(self, z, H, R, gate_threshold):
        """
        Generic EKF Update
        z: measurement residual (innovation)
        H: Jacobian
        R: Measurement Noise
        gate_threshold: NIS threshold
        """
        # Innovation Covariance
        S = H @ self.P @ H.T + R
        
        # NIS Check
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            print("Singular matrix in update step")
            return False, 0.0

        nis = z.T @ S_inv @ z
        
        if nis > gate_threshold:
            return False, nis
        
        # Kalman Gain
        K = self.P @ H.T @ S_inv
        
        # State Correction
        dx = K @ z
        self.x += dx
        
        # Covariance Update (Joseph form for numerical stability)
        I = np.eye(15)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        
        return True, nis

    # --- Specific Updates ---

    def update_gps_pos_ned(self, pos_ned, cov, gate):
        """GPS Position Update"""
        z = pos_ned - self.x[0:3]
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        R = cov if cov is not None else np.eye(3)
        
        return self.update(z, H, R, gate)

    def update_gps_vel_ned(self, vel_ned, cov, gate):
        """GPS Velocity Update"""
        z = vel_ned - self.x[3:6]
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        R = cov if cov is not None else np.eye(3)
        
        return self.update(z, H, R, gate)

    def update_gps_heading(self, heading_meas, gate, meas_noise):
        """
        GPS Heading Update (e.g., from dual antenna or velocity)
        Compares with AHRS attitude
        Updates magnetometer bias to align heading
        """
        # Get current heading from AHRS
        euler = self.get_euler()
        heading_est = np.deg2rad(euler[2])
        
        # Residual with wrap
        res = heading_meas - heading_est
        res = wrap_pi(res)
        z = np.array([res])
        
        # Jacobian H (1x15)
        # Heading depends on mag bias through AHRS
        # Approximate: d(heading)/d(mag_bias) ≈ sensitivity
        # For simplicity, we update mag bias proportionally
        H = np.zeros((1, 15))
        
        # Sensitivity to mag bias (empirical: ~1 rad per unit mag bias)
        # This is a linearization around current state
        H[0, 12:15] = np.array([0.1, 0.1, 0.0])  # Primarily affects x, y components
        
        R = np.array([[meas_noise**2]])
        
        return self.update(z, H, R, gate)

    def get_quaternion(self):
        """Get current quaternion from AHRS"""
        q_ahrs = self.ahrs.quaternion
        return np.array([q_ahrs.w, q_ahrs.x, q_ahrs.y, q_ahrs.z])
    
    def get_euler(self):
        """Get current Euler angles from AHRS (deg)"""
        euler = self.ahrs.quaternion.to_euler()
        return np.array([euler[0], euler[1], euler[2]])
    
    def get_flags(self):
        """Get AHRS internal flags"""
        flags = self.ahrs.flags
        return {
            'initialising': flags.initialising,
            'angular_rate_recovery': flags.angular_rate_recovery,
            'acceleration_recovery': flags.acceleration_recovery,
            'magnetic_recovery': flags.magnetic_recovery
        }
    
    def set_ahrs_settings(self, gain=None, gyro_range=None, 
                          accel_rejection=None, mag_rejection=None):
        """Update AHRS settings"""
        if gain is not None:
            self.ahrs_settings.gain = gain
        if gyro_range is not None:
            self.ahrs_settings.gyroscope_range = gyro_range
        if accel_rejection is not None:
            self.ahrs_settings.acceleration_rejection = accel_rejection
        if mag_rejection is not None:
            self.ahrs_settings.magnetic_rejection = mag_rejection
        
        self.ahrs.settings = self.ahrs_settings