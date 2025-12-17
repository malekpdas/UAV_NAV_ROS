import rclpy
import numpy as np
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, MagneticField
from bmx160_ros.bmx160_lib import BMX160, GyroRange, AccelRange

class BMX160Node(Node):
    def __init__(self):
        super().__init__('bmx160_node')

        # Parameters
        self.declare_parameter('bus', 1)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('bias_removal', True)
        self.declare_parameter('bias_duration_sec', 2.0)
        self.declare_parameter('accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_bias', [0.0, 0.0, 0.0])
        
        # Sensor Variance Parameters
        self.declare_parameter('accel_variance', [0.001, 0.001, 0.001])
        self.declare_parameter('gyro_variance', [0.0001, 0.0001, 0.0001])
        self.declare_parameter('mag_variance', [1e-3, 1e-3, 1e-3])
        self.declare_parameter('mag_declination_angle', 0.0)
        
        # Magnetometer Calibration Parameters
        self.declare_parameter('mag_bias', [0.0, 0.0, 0.0])
        # Default identity matrix for 3x3 transform, flattened
        self.declare_parameter('mag_transform', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        
        # IMU Rotation (Mounting)
        self.declare_parameter('imu_rotation_transformation', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])

        # IMU Rotation
        self._imu_rot = np.array(self.get_parameter('imu_rotation_transformation').value).reshape(3, 3)

        self.bus = self.get_parameter('bus').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.bias_removal = self.get_parameter('bias_removal').value
        self.bias_duration_sec = self.get_parameter('bias_duration_sec').value

        self.mag_dec_angle = self.get_parameter('mag_declination_angle').value
        self._mag_dec_rot = R.from_euler('xyz', [0, 0, self.mag_dec_angle], degrees=True).as_matrix()

        # Initialize Sensor
        self.imu = BMX160(self.bus)
        if self.imu.begin():
            self.get_logger().info('✅ BMX160 init OK')
    
            # Configure gyro range (±500°/s is good for most applications)
            self.imu.set_gyro_range(GyroRange.DPS_500)
            
            # Configure accel range (±4g is good for most applications)
            self.imu.set_accel_range(AccelRange.G_4)
            
            self.get_logger().info(f'✅ Gyro configured: {GyroRange.DPS_500} sensitivity')
            self.get_logger().info(f'✅ Accel configured: {AccelRange.G_4} sensitivity')

        else:
            self.get_logger().error('❌ BMX160 init FAILED (check I2C)')
            raise RuntimeError('BMX160 init failed')

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Calibration State
        self.calibrating = self.bias_removal
        self.calibration_samples = {
            'accel': [],
            'gyro': []
        }
        self.calibration_end_time = None
        
        # Biases
        self._accel_bias = np.array(self.get_parameter('accel_bias').value)
        self._gyro_bias = np.array(self.get_parameter('gyro_bias').value)

        # Variances
        self._accel_cov = np.diag(self.get_parameter('accel_variance').value).flatten()
        self._gyro_cov = np.diag(self.get_parameter('gyro_variance').value).flatten()
        self._mag_cov = np.diag(self.get_parameter('mag_variance').value).flatten()
        
        # Magnetometer Calibration (Loaded from params initially)
        self._mag_bias = np.array(self.get_parameter('mag_bias').value)
        self._mag_transform = np.array(self.get_parameter('mag_transform').value).reshape(3, 3)
        

        # Parameter Callback
        self.sub_params = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.on_param_event,
            10
        )

        # Timer
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.01
        self.timer = self.create_timer(period, self.tick)
        
        if self.calibrating:
             self.get_logger().info(f'⚠ Starting Bias Calibration ({self.bias_duration_sec}s). Keep vehicle LEVEL and STATIONARY.')
             self.calibration_end_time = self.get_clock().now().nanoseconds / 1e9 + self.bias_duration_sec

    def tick(self):
        try:
            d = self.imu.get_all_data()
            if not d:
                return

            # Raw Data
            # Magnetometer: uT
            mx, my, mz = float(d[0]), float(d[1]), float(d[2])
            
            # Mapping
            ax_ms2, ay_ms2, az_ms2 = float(d[6]), float(d[7]), float(d[8])
            gx_dps, gy_dps, gz_dps = float(d[3]), float(d[4]), float(d[5])
            
            gx_rad = np.deg2rad(gx_dps)
            gy_rad = np.deg2rad(gy_dps)
            gz_rad = np.deg2rad(gz_dps)

            if self.calibrating:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec < self.calibration_end_time:
                    self.calibration_samples['accel'].append([ax_ms2, ay_ms2, az_ms2])
                    self.calibration_samples['gyro'].append([gx_rad, gy_rad, gz_rad])
                else:
                    # Finish calibration
                    accel_data = np.array(self.calibration_samples['accel'])
                    gyro_data = np.array(self.calibration_samples['gyro'])
                    
                    if len(accel_data) > 0:
                        self._gyro_bias = np.mean(gyro_data, axis=0)

                        # Accel bias: assume Z is Gravity (9.80665)
                        # We want readings to be [0, 0, +g] when level.
                        # measured = true + bias => bias = measured - true
                        
                        gravity = self._imu_rot @ np.array([0.0, 0.0, 9.80665])
                        a = accel_data + gravity
                        self._accel_bias = np.mean(a, axis=0)
                        
                        self.get_logger().info(f'Calibration Done. Gyro Bias: {self._gyro_bias}, Accel Bias: {self._accel_bias}')
                    else:
                        self.get_logger().warn('Calibration failed: no data collected.')
                    
                    self.calibrating = False
                return

            # Apply Logic
            # IMU Message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Apply bias removal (Sensor Frame)
            ax_calib = ax_ms2 - self._accel_bias[0]
            ay_calib = ay_ms2 - self._accel_bias[1]
            az_calib = az_ms2 - self._accel_bias[2]
            
            gx_calib = gx_rad - self._gyro_bias[0]
            gy_calib = gy_rad - self._gyro_bias[1]
            gz_calib = gz_rad - self._gyro_bias[2]
            
            # Apply Rotation (Mounting -> Body)
            accel_vec = np.array([ax_calib, ay_calib, az_calib])
            gyro_vec = np.array([gx_calib, gy_calib, gz_calib])
            
            accel_body = self._imu_rot @ accel_vec
            gyro_body = self._imu_rot @ gyro_vec

            imu_msg.linear_acceleration.x = accel_body[0]
            imu_msg.linear_acceleration.y = accel_body[1]
            imu_msg.linear_acceleration.z = accel_body[2]
            
            imu_msg.angular_velocity.x = gyro_body[0]
            imu_msg.angular_velocity.y = gyro_body[1]
            imu_msg.angular_velocity.z = gyro_body[2]
            
            # Set covariance
            imu_msg.orientation_covariance[0] = -1.0 # Orientation not provided
            imu_msg.linear_acceleration_covariance = self._accel_cov
            imu_msg.angular_velocity_covariance = self._gyro_cov
            
            self.pub_imu.publish(imu_msg)

            # Magnetic Field Message
            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp # Sync
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field_covariance = self._mag_cov
            
            # Calibration Logic from user request
            B = np.array([mx, my, mz])
            
            # Apply Rotation (Mounting -> Body)
            B_body = self._imu_rot @ B

            # Soft Iron / Bias (Sensor Frame)
            B_bias = self._imu_rot @ self._mag_bias
            B_calib = (B_body - B_bias) @ self._mag_transform.T

            B_true_north = self._mag_dec_rot.T @ B_calib

            mag_msg.magnetic_field.x = B_true_north[0]
            mag_msg.magnetic_field.y = B_true_north[1]
            mag_msg.magnetic_field.z = B_true_north[2]

            self.pub_mag.publish(mag_msg)

        except Exception as e:
            self.get_logger().warn(f'read/publish error: {e}')

    def on_param_event(self, event):
        for changed in event.changed_parameters:
            if changed.name == "mag_bias":
                if len(changed.value.double_array_value) == 3:
                     self._mag_bias = np.array(changed.value.double_array_value)
                     self.get_logger().info(f"Updated mag_bias: {self._mag_bias}")
            elif changed.name == "mag_transform":
                if len(changed.value.double_array_value) == 9:
                    self._mag_transform = np.array(changed.value.double_array_value).reshape(3, 3)
                    self.get_logger().info(f"Updated mag_transform parameters")
            elif changed.name == "imu_rotation_transformation":
                 if len(changed.value.double_array_value) == 9:
                    self._imu_rot = np.array(changed.value.double_array_value).reshape(3, 3)
                    self.get_logger().info(f"Updated imu_rotation_transformation")
            elif changed.name == "accel_bias":
                if len(changed.value.double_array_value) == 3:
                     self._accel_bias = np.array(changed.value.double_array_value)
                     self.get_logger().info(f"Updated accel_bias: {self._accel_bias}")
            elif changed.name == "gyro_bias":
                if len(changed.value.double_array_value) == 3:
                     self._gyro_bias = np.array(changed.value.double_array_value)
                     self.get_logger().info(f"Updated gyro_bias: {self._gyro_bias}")
            elif changed.name == "accel_variance":
                if len(changed.value.double_array_value) == 3:
                     self._accel_cov = np.diag(changed.value.double_array_value).flatten()
                     self.get_logger().info("Updated accel_variance")
            elif changed.name == "gyro_variance":
                if len(changed.value.double_array_value) == 3:
                     self._gyro_cov = np.diag(changed.value.double_array_value).flatten()
                     self.get_logger().info("Updated gyro_variance")
            elif changed.name == "mag_variance":
                if len(changed.value.double_array_value) == 3:
                     self._mag_cov = np.diag(changed.value.double_array_value).flatten()
                     self.get_logger().info("Updated mag_variance")
            elif changed.name == "mag_declination_angle":
                self._mag_cov = changed.value.double_value
                self.get_logger().info("Updated mag_declination_angle")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BMX160Node()
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
