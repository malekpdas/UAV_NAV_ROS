#!/usr/bin/env python3
"""
ROS2 Node for interfacing with the Hillcrest BNO085 9-axis IMU.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensors.bno085.bno085_lib import BNO085, BNO085_I2C_ADDR_DEFAULT

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')

        self.declare_all_parameters()
        self.load_parameters()

        self.imu = BNO085(1, BNO085_I2C_ADDR_DEFAULT)
        if self.imu.begin():
            self.get_logger().info(f'✅ BNO085 init OK on bus {1} at address 0x{BNO085_I2C_ADDR_DEFAULT:02X}')
        else:
            self.get_logger().fatal(f'❌ BNO085 init FAILED on bus {1} at address 0x{BNO085_I2C_ADDR_DEFAULT:02X}')
            self._ok = False
            return

        # Publishers
        self.pub_imu = self.create_publisher(Imu, "imu_bno085/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "imu_bno085/mag", 10)
        
        # Internal Calibration State (Matches BMX160 behavior)
        self.calibrating = self.bias_removal
        self.calibration_samples = {'accel': [], 'gyro': []}
        self.calibration_end_time = None
        
        if self.calibrating:
             self.get_logger().info(f'⚠ Starting Bias Calibration ({self.bias_duration_sec}s). Keep vehicle LEVEL and STATIONARY.')
             self.calibration_end_time = self.get_clock().now().nanoseconds / 1e9 + self.bias_duration_sec

        # Timer
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.tick)

    def declare_all_parameters(self):
        # imu
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Sensor Calibration
        self.declare_parameter('sensor_calibration.bias_removal', False)
        self.declare_parameter('sensor_calibration.bias_duration_sec', 2.0)
        self.declare_parameter('sensor_calibration.accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_calibration.gyro_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_calibration.mag_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_calibration.mag_transform', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        
        # Sensor Variance Parameters
        self.declare_parameter('sensor_variance.accel', [0.01, 0.01, 0.01])
        self.declare_parameter('sensor_variance.gyro', [0.001, 0.001, 0.001])
        self.declare_parameter('sensor_variance.mag', [0.001, 0.001, 0.001])
        
        # IMU Rotation (Mounting)
        self.declare_parameter('transformation.imu_rotation', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('transformation.mag_decl', 0.0)

    def load_parameters(self):
        # imu
        self.rate_hz = self.get_parameter('rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        # Transformations
        self._imu_rot = np.array(self.get_parameter('transformation.imu_rotation').value).reshape(3, 3)
        self.mag_dec_angle = self.get_parameter('transformation.mag_decl').value
        self._mag_dec_rot = R.from_euler('xyz', [0, 0, self.mag_dec_angle], degrees=True).as_matrix()
        
        # Sensor Calibration
        self.bias_removal = self.get_parameter('sensor_calibration.bias_removal').value
        self.bias_duration_sec = self.get_parameter('sensor_calibration.bias_duration_sec').value
        self._accel_bias = np.array(self.get_parameter('sensor_calibration.accel_bias').value)
        self._gyro_bias = np.array(self.get_parameter('sensor_calibration.gyro_bias').value)
        self._mag_bias = np.array(self.get_parameter('sensor_calibration.mag_bias').value)
        self._mag_transform = np.array(self.get_parameter('sensor_calibration.mag_transform').value).reshape(3, 3)
        
        # Sensor Variances
        self._accel_cov = np.diag(self.get_parameter('sensor_variance.accel').value).flatten()
        self._gyro_cov = np.diag(self.get_parameter('sensor_variance.gyro').value).flatten()
        self._mag_cov = np.diag(self.get_parameter('sensor_variance.mag').value).flatten()

    def tick(self):
        try:
            data = self.imu.get_data()
            # In raw mode, get_data returns dict with 'accel', 'gyro', 'mag'
            
            # Check if we have valid data (non-zero or updated)
            # Since get_data polls, if no packet, the dict values might be stale or zero.
            # But get_data updates the internal dict in place.
            pass # Continues below

            now = self.get_clock().now().to_msg()
            
            # Extract Raw Data
            accel = data['accel'] # m/s^2
            gyro = data['gyro']   # rad/s
            mag = data['mag']     # uT (driver parses as uT?)
            # Wait, driver parses int16 * Q_POINT.
            # Q_POINT_8 (accel) -> 2^-8 = 1/256 = 0.00390625 m/s^2 (Wait, Q point unit is usually G or m/s^2 depending on report)
            # SHTP Ref manual: Accel report (0x01) units are m/s^2 with Q-point 8.
            # Gyro report (0x02) units are rad/s with Q-point 9.
            # Mag report (0x03) units are uT with Q-point 4.
            # My driver uses these Q points. So units are correct.
            
            ax, ay, az = accel[0], accel[1], accel[2]
            gx, gy, gz = gyro[0], gyro[1], gyro[2]
            mx, my, mz = mag[0], mag[1], mag[2]

            # Calibration Logic (Startup Bias Removal)
            if self.calibrating:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec < self.calibration_end_time:
                    self.calibration_samples['accel'].append([ax, ay, az])
                    self.calibration_samples['gyro'].append([gx, gy, gz])
                else:
                    # Finish calibration
                    accel_data = np.array(self.calibration_samples['accel'])
                    gyro_data = np.array(self.calibration_samples['gyro'])
                    
                    if len(accel_data) > 0:
                        self._gyro_bias = np.mean(gyro_data, axis=0)
                        
                        # Accel bias: assume Z is Gravity (9.80665)
                        gravity = self._imu_rot @ np.array([0.0, 0.0, -9.80665])
                        a_mean = np.mean(accel_data, axis=0)
                        self._accel_bias = a_mean - gravity
                        
                        self.get_logger().info(f'Calibration Done. Gyro Bias: {self._gyro_bias}, Accel Bias: {self._accel_bias}')
                    else:
                        self.get_logger().warn('Calibration failed: no data collected.')
                    
                    self.calibrating = False
                return

            # IMU Message
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id
            
            # Apply Bias (Sensor Frame)
            ax_cal = ax - self._accel_bias[0]
            ay_cal = ay - self._accel_bias[1]
            az_cal = az - self._accel_bias[2]
            
            gx_cal = gx - self._gyro_bias[0]
            gy_cal = gy - self._gyro_bias[1]
            gz_cal = gz - self._gyro_bias[2]
            
            # Apply Rotation (Mounting -> Body)
            accel_vec = np.array([ax_cal, ay_cal, az_cal])
            gyro_vec = np.array([gx_cal, gy_cal, gz_cal])
            
            accel_body = self._imu_rot @ accel_vec
            gyro_body = self._imu_rot @ gyro_vec
            
            imu_msg.linear_acceleration.x = accel_body[0]
            imu_msg.linear_acceleration.y = accel_body[1]
            imu_msg.linear_acceleration.z = accel_body[2]
            imu_msg.linear_acceleration_covariance = self._accel_cov
            
            imu_msg.angular_velocity.x = gyro_body[0]
            imu_msg.angular_velocity.y = gyro_body[1]
            imu_msg.angular_velocity.z = gyro_body[2]
            imu_msg.angular_velocity_covariance = self._gyro_cov
            
            # No Orientation provided (Raw Mode)
            imu_msg.orientation_covariance[0] = -1.0 
            
            # Magnetic Field Message
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id
            
            # Mag Processing
            B = np.array([mx, my, mz])
            
            # Rotation
            B_body = self._imu_rot @ B
            
            # Soft/Hard Iron 
            B_bias = self._imu_rot @ self._mag_bias
            B_calib = (B_body - B_bias) @ self._mag_transform.T
            
            # Declination alignment (if needed)
            B_true = self._mag_dec_rot.T @ B_calib
            
            mag_msg.magnetic_field.x = B_true[0] * 1e-6 # Convert uT to Tesla
            mag_msg.magnetic_field.y = B_true[1] * 1e-6
            mag_msg.magnetic_field.z = B_true[2] * 1e-6
            mag_msg.magnetic_field_covariance = self._mag_cov

            if rclpy.ok():
                self.pub_imu.publish(imu_msg)
                self.pub_mag.publish(mag_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error: {e}')

    def destroy(self):
        if hasattr(self, 'imu'):
            self.imu.close()
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()

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
