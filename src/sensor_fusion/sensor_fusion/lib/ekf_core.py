"""
Linear Kalman Filter for IMU/GPS integration with acceleration bias estimation.
State: [pn, pe, pd, vn, ve, vd, ban, bae, bad] (9D)

Supports separated measurement updates:
- update_position(z_pos): Update with GPS position only
- update_velocity(z_vel): Update with GPS velocity only  
- update(z_pos, z_vel): Combined update with both measurements
"""

import numpy as np

class LPFilter:
    """Low-pass filter for sensor data smoothing."""
    
    def __init__(self, alpha=0.9):
        self.prev = None
        self.alpha = alpha

    def update(self, current):
        if self.prev is None:
            self.prev = current
            return self.prev.copy()
        
        self.prev = self.alpha * self.prev + (1 - self.alpha) * current
        return self.prev.copy()
    
class RobustLPFilter:
    def __init__(self, alpha=0.95, max_delta=2.0):
        """
        alpha     : LP smoothing (0.9-0.98 recommended)
        max_delta : max allowed change per sample
        """
        self.alpha = alpha
        self.max_delta = max_delta
        self.prev = None

    def update(self, current):
        current = np.asarray(current)

        if self.prev is None:
            self.prev = current.copy()
            return self.prev

        delta = current - self.prev
        delta = np.clip(delta, -self.max_delta, self.max_delta)

        filtered = self.prev + delta
        self.prev = self.alpha * self.prev + (1 - self.alpha) * filtered
        return self.prev.copy()

class LinearKalmanFilter:
    """
    Linear Kalman Filter for IMU/GPS integration with acceleration bias estimation.
    
    State vector (9D):
        [pn, pe, pd, vn, ve, vd, ban, bae, bad]
        - position (NED): pn, pe, pd
        - velocity (NED): vn, ve, vd
        - acceleration bias: ban, bae, bad
    
    This allows asynchronous sensor fusion when position and velocity
    arrive at different times or rates.
    """
    
    def __init__(self, config=None):
        """
        Initialize the Kalman Filter.
        
        Args:
            config: Dictionary with filter parameters (optional)
        """
        # Default configuration
        default_config = {
            'initial_pos_uncertainty_h': 2.0,
            'initial_pos_uncertainty_v': 5.0,
            'initial_vel_uncertainty_h': 1.0,
            'initial_vel_uncertainty_v': 0.5,
            'initial_bias_uncertainty': 0.01,
            'process_noise_pos_h': 0.1,
            'process_noise_pos_v': 0.01,
            'process_noise_vel_h': 0.01,
            'process_noise_vel_v': 0.005,
            'process_noise_bias': 0.001,
            'measurement_noise_pos_h': 25.0,
            'measurement_noise_pos_v': 50.0,
            'measurement_noise_vel_h': 0.25,
            'measurement_noise_vel_v': 0.1,
            'gravity': 9.8066,
        }
        
        # Merge with provided config
        self.config = default_config
        if config is not None:
            self.config.update(config)
        
        # State: [position(3), velocity(3), accel_bias(3)]
        self.x = np.zeros(9)
        
        # State covariance matrix
        self.P = np.eye(9)
        self.P[0:2, 0:2] *= self.config['initial_pos_uncertainty_h']
        self.P[2, 2]     *= self.config['initial_pos_uncertainty_v']
        self.P[3:5, 3:5] *= self.config['initial_vel_uncertainty_h']
        self.P[5, 5]     *= self.config['initial_vel_uncertainty_v']
        self.P[6:9, 6:9] *= self.config['initial_bias_uncertainty']
        
        # Process noise covariance (base spectral density, scaled by dt in predict)
        self.Q = np.zeros(9)
        self.Q[0] = self.config['process_noise_pos_h']
        self.Q[1] = self.config['process_noise_pos_h']
        self.Q[2] = self.config['process_noise_pos_v']
        self.Q[3] = self.config['process_noise_vel_h']
        self.Q[4] = self.config['process_noise_vel_h']
        self.Q[5] = self.config['process_noise_vel_v']
        self.Q[6] = self.config['process_noise_bias']
        self.Q[7] = self.config['process_noise_bias']
        self.Q[8] = self.config['process_noise_bias']

    def predict(self, a_measured, dt):
        """
        Prediction step with IMU acceleration measurement.
        
        Args:
            a_measured: Acceleration in NED frame from AHRS (m/s²)
            dt: Time step (s)
        """
        # Extract state components
        p = self.x[0:3]
        v = self.x[3:6]
        ba = self.x[6:9]
        
        # Correct acceleration by removing estimated bias
        a_corrected = a_measured - ba
        
        # State transition matrix F
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt               # p += v*dt
        F[0:3, 6:9] = -0.5 * dt**2 * np.eye(3)     # p -= 0.5*ba*dt²
        F[3:6, 6:9] = -dt * np.eye(3)              # v -= ba*dt
        
        # State prediction (motion model)
        self.x[0:3] = p + v * dt + 0.5 * a_corrected * dt**2
        self.x[3:6] = v + a_corrected * dt
        # Biases remain constant: self.x[6:9] = ba
        
        # Covariance prediction with dt-scaled process noise
        # Note: Position noise scaled by dt (random walk) to model drift/uncertainty growth
        # instead of dt^3/3 (integrated accel noise) to prevent covariance collapse.
        Q_scaled = np.diag([
            self.Q[0] * dt,           # pos_h (drift/diffusion)
            self.Q[1] * dt,           # pos_h
            self.Q[2] * dt,           # pos_v
            self.Q[3] * dt,           # vel_h
            self.Q[4] * dt,           # vel_h
            self.Q[5] * dt,           # vel_v
            self.Q[6] * dt,           # bias
            self.Q[7] * dt,           # bias
            self.Q[8] * dt,           # bias
        ])
        self.P = F @ self.P @ F.T + Q_scaled
        
    def update_position(self, z_pos, R=None):
        """
        Measurement update with GPS position only.
        
        Args:
            z_pos: GPS position [pn, pe, pd] (m)
            R: Measurement noise covariance (3x3), optional
        """
        # Measurement vector (3D: pos only)
        z = z_pos
        
        # Measurement matrix H (3x9)
        H = np.zeros((3, 9))
        H[0:3, 0:3] = np.eye(3)  # Measure position only
        
        # Measurement noise (position only)
        if R is None:
            R = np.diag([
                self.config['measurement_noise_pos_h'],
                self.config['measurement_noise_pos_h'],
                self.config['measurement_noise_pos_v'],
            ])
        
        # Innovation (measurement residual)
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state estimate
        self.x = self.x + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
    def update_velocity(self, z_vel, R=None):
        """
        Measurement update with GPS velocity only.
        
        Args:
            z_vel: GPS velocity [vn, ve, vd] (m/s)
            R: Measurement noise covariance (3x3), optional
        """
        # Measurement vector (3D: vel only)
        z = z_vel
        
        # Measurement matrix H (3x9)
        H = np.zeros((3, 9))
        H[0:3, 3:6] = np.eye(3)  # Measure velocity only
        
        # Measurement noise (velocity only)
        if R is None:
            R = np.diag([
                self.config['measurement_noise_vel_h'],
                self.config['measurement_noise_vel_h'],
                self.config['measurement_noise_vel_v'],
            ])
        
        # Innovation (measurement residual)
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state estimate
        self.x = self.x + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def update(self, z_pos, z_vel, R=None):
        """
        Measurement update with both GPS position and velocity (combined update).
        
        Args:
            z_pos: GPS position [pn, pe, pd] (m)
            z_vel: GPS velocity [vn, ve, vd] (m/s)
            R: Measurement noise covariance (6x6), optional
        """
        # Measurement vector (6D: pos + vel)
        z = np.concatenate([z_pos, z_vel])
        
        # Measurement matrix H (6x9)
        H = np.zeros((6, 9))
        H[0:3, 0:3] = np.eye(3)  # Measure position
        H[3:6, 3:6] = np.eye(3)  # Measure velocity

        if R is None:
            R = np.diag([
                self.config['measurement_noise_pos_h'],
                self.config['measurement_noise_pos_h'],
                self.config['measurement_noise_pos_v'],
                self.config['measurement_noise_vel_h'],
                self.config['measurement_noise_vel_h'],
                self.config['measurement_noise_vel_v'],
            ])
        
        # Innovation (measurement residual)
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state estimate
        self.x = self.x + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
    def initialize(self, pos, vel):
        """
        Initialize filter with GPS position and velocity.
        
        Args:
            pos: Initial position [pn, pe, pd] (m)
            vel: Initial velocity [vn, ve, vd] (m/s)
        """
        self.x[0:3] = pos
        self.x[3:6] = vel
        self.x[6:9] = np.zeros(3)  # Zero accel bias initially
        
    def get_state(self):
        """Returns full state vector."""
        return self.x.copy()
    
    def get_position(self):
        """Returns position [pn, pe, pd]."""
        return self.x[0:3].copy()
    
    def get_velocity(self):
        """Returns velocity [vn, ve, vd]."""
        return self.x[3:6].copy()
    
    def get_accel_bias(self):
        """Returns acceleration bias [ban, bae, bad]."""
        return self.x[6:9].copy()
    
    def get_covariance(self):
        """Returns state covariance matrix."""
        return self.P.copy()