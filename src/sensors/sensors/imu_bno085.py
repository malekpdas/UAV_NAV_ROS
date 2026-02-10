#!/usr/bin/env python3
"""
ROS2 Node for interfacing with the Hillcrest BNO085 9-axis IMU.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensors.bno085.bno085_lib import BNO085

import time

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')

        self.declare_all_parameters()
        self.load_parameters()

        try:
            self.imu = BNO085(rate_hz=self.rate_hz)
            self.get_logger().info(f'✅ BNO085 init OK (rate: {self.rate_hz}Hz) on bus 1')
        except Exception as e:
            self.get_logger().fatal(f'❌ BNO085 init FAILED: {e}')
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

        # Timer: Fixed-rate publication to match the configured rate_hz
        period = 1.0 / self.rate_hz
        self.last_update_time = 0.0
        self.timer = self.create_timer(period, self.tick)

    def declare_all_parameters(self):
        # imu
        self.declare_parameter('rate_hz.value', 100.0)
        self.declare_parameter('frame_id.value', 'imu_link')
        
        # Sensor Calibration
        self.declare_parameter('sensor_calibration.bias_removal.value', False)
        self.declare_parameter('sensor_calibration.bias_duration_sec.value', 2.0)
        self.declare_parameter('sensor_calibration.accel_bias.value', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_calibration.gyro_bias.value', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_calibration.mag_bias.value', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_calibration.mag_transform.value', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        
        # Sensor Variance Parameters
        self.declare_parameter('sensor_variance.accel.value', [0.01, 0.01, 0.01])
        self.declare_parameter('sensor_variance.gyro.value', [0.001, 0.001, 0.001])
        self.declare_parameter('sensor_variance.mag.value', [0.001, 0.001, 0.001])
        
        # IMU Rotation (Mounting)
        self.declare_parameter('transformation.imu_rotation.value', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('transformation.mag_decl.value', 0.0)

    def load_parameters(self):
        # imu
        self.rate_hz = self.get_parameter('rate_hz.value').value
        self.frame_id = self.get_parameter('frame_id.value').value

        # Transformations
        self.imu_rot = np.array(self.get_parameter('transformation.imu_rotation.value').value).reshape(3, 3)
        self.mag_dec_angle = self.get_parameter('transformation.mag_decl.value').value
        self.mag_dec_rot = R.from_euler('xyz', [0, 0, self.mag_dec_angle], degrees=True).as_matrix()
        
        # Sensor Calibration
        self.bias_removal = self.get_parameter('sensor_calibration.bias_removal.value').value
        self.bias_duration_sec = self.get_parameter('sensor_calibration.bias_duration_sec.value').value
        self.accel_bias = np.array(self.get_parameter('sensor_calibration.accel_bias.value').value)
        self.gyro_bias = np.array(self.get_parameter('sensor_calibration.gyro_bias.value').value)
        self.mag_bias = np.array(self.get_parameter('sensor_calibration.mag_bias.value').value)
        self.mag_transform = np.array(self.get_parameter('sensor_calibration.mag_transform.value').value).reshape(3, 3)
        
        # Sensor Variances
        self.accel_cov = np.diag(self.get_parameter('sensor_variance.accel.value').value).flatten()
        self.gyro_cov = np.diag(self.get_parameter('sensor_variance.gyro.value').value).flatten()
        self.mag_cov = np.diag(self.get_parameter('sensor_variance.mag.value').value).flatten()

    def tick(self):
        try:
            # Drain all reports from the internal buffer. 
            accel, gyro, mag = self.imu.read_latest_snapshot()

            if self.calibrating:
                self.calibrate_biases(accel, gyro)
                return
            
            # Publish exactly once per tick to maintain the configured frequency
            if any(x is None for x in [accel, gyro, mag]): # Ensure we have received at least one sample ever
                self.get_logger().warn("Received None data from BNO085 sensor")
                return
            self.publish_data(accel, gyro, mag)
                
        except Exception as e:
            self.get_logger().warn(f'Error in tick: {e}')

    def calibrate_biases(self, accel, gyro):
        ax, ay, az = accel
        gx, gy, gz = gyro
        # Calibration Logic (Startup Bias Removal)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self.calibration_end_time:
            self.calibration_samples['accel'].append([ax, ay, az])
            self.calibration_samples['gyro'].append([gx, gy, gz])
        else:
            # Finish calibration
            accel_data = np.array(self.calibration_samples['accel'])
            gyro_data = np.array(self.calibration_samples['gyro'])
            
            if len(accel_data) > 0:
                self.gyro_bias = np.mean(gyro_data, axis=0)
                
                # Accel bias: assume Z is Gravity (9.80665)
                gravity = self.imu_rot @ np.array([0.0, 0.0, -9.80665])
                a_mean = np.mean(accel_data, axis=0)
                self.accel_bias = a_mean - gravity
                
                self.get_logger().info(f'Calibration Done. Gyro Bias: {self.gyro_bias}, Accel Bias: {self.accel_bias}')
            else:
                self.get_logger().warn('Calibration failed: no data collected.')
            
            self.calibrating = False

    def publish_data(self, accel, gyro, mag):
        """Callback to publish ROS messages for each received report."""
        now = self.get_clock().now().to_msg()
        
        ax, ay, az = accel[0], accel[1], accel[2]
        gx, gy, gz = gyro[0], gyro[1], gyro[2]
        mx, my, mz = mag[0], mag[1], mag[2]

        # IMU Message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        
        # Apply Bias (Sensor Frame)
        ax_cal = ax - self.accel_bias[0]
        ay_cal = ay - self.accel_bias[1]
        az_cal = az - self.accel_bias[2]
        
        gx_cal = gx - self.gyro_bias[0]
        gy_cal = gy - self.gyro_bias[1]
        gz_cal = gz - self.gyro_bias[2]
        
        # Apply Rotation (Mounting -> Body)
        accel_vec = np.array([ax_cal, ay_cal, az_cal])
        gyro_vec = np.array([gx_cal, gy_cal, gz_cal])
        
        accel_body = self.imu_rot @ accel_vec
        gyro_body = self.imu_rot @ gyro_vec
        
        imu_msg.linear_acceleration.x = accel_body[0]
        imu_msg.linear_acceleration.y = accel_body[1]
        imu_msg.linear_acceleration.z = accel_body[2]
        imu_msg.linear_acceleration_covariance = self.accel_cov
        
        imu_msg.angular_velocity.x = gyro_body[0]
        imu_msg.angular_velocity.y = gyro_body[1]
        imu_msg.angular_velocity.z = gyro_body[2]
        imu_msg.angular_velocity_covariance = self.gyro_cov
        
        # No Orientation provided (Raw Mode)
        imu_msg.orientation_covariance[0] = -1.0 
        
        # Magnetic Field Message
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self.frame_id
        
        # Mag Processing
        B = np.array([mx, my, mz])
        
        # Rotation
        B_body = self.imu_rot @ B
        
        # Soft/Hard Iron 
        B_calib = (B_body - self.mag_bias) @ self.mag_transform.T
        
        # Declination alignment (if needed)
        B_true = self.mag_dec_rot.T @ B_calib
        
        mag_msg.magnetic_field.x = B_true[0]
        mag_msg.magnetic_field.y = B_true[1]
        mag_msg.magnetic_field.z = B_true[2]
        mag_msg.magnetic_field_covariance = self.mag_cov

        if rclpy.ok():
            self.pub_imu.publish(imu_msg)
            self.pub_mag.publish(mag_msg)
            
    def destroy(self):
        if hasattr(self, 'imu'):
            self.imu.close_sensor()
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
