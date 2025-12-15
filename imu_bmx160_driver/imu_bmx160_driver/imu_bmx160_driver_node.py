import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from .bmx160_i2c import BMX160

class Bmx160DriverNode(Node):
    def __init__(self):
        super().__init__('imu_bmx160_driver')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('accel_range', 4) # g
        self.declare_parameter('gyro_range', 2000) # deg/s
        self.declare_parameter('accel_std', 0.01)
        self.declare_parameter('gyro_std', 0.001)
        self.declare_parameter('mag_std', 0.1)

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Sensor Init
        self.sensor = BMX160(bus_num=self.i2c_bus, address=self.i2c_address)
        
        if not self.sensor.open():
            self.get_logger().error(f"Could not open I2C bus {self.i2c_bus}")
            return

        if not self.sensor.check_id():
            self.get_logger().warn(f"BMX160 ID mismatch or not found at {hex(self.i2c_address)}. Continuing anyway...")

        self.sensor.init_device(
            accel_range=self.get_parameter('accel_range').value,
            gyro_range=self.get_parameter('gyro_range').value
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Timer
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info("BMX160 Driver Initialized")

    def timer_callback(self):
        data = self.sensor.read_all_data()
        if data is None:
            self.get_logger().warn("Failed to read sensor data")
            return

        now = self.get_clock().now().to_msg()

        # IMU Message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        
        # Accel
        imu_msg.linear_acceleration.x = data['accel'][0]
        imu_msg.linear_acceleration.y = data['accel'][1]
        imu_msg.linear_acceleration.z = data['accel'][2]
        
        # Gyro
        imu_msg.angular_velocity.x = data['gyro'][0]
        imu_msg.angular_velocity.y = data['gyro'][1]
        imu_msg.angular_velocity.z = data['gyro'][2]
        
        # Covariance (Parameters)
        acc_std = self.get_parameter('accel_std').value
        gyro_std = self.get_parameter('gyro_std').value
        acc_cov = acc_std * acc_std
        gyro_cov = gyro_std * gyro_std
        
        imu_msg.linear_acceleration_covariance = [
            acc_cov, 0.0, 0.0,
            0.0, acc_cov, 0.0,
            0.0, 0.0, acc_cov
        ]
        imu_msg.angular_velocity_covariance = [
            gyro_cov, 0.0, 0.0,
            0.0, gyro_cov, 0.0,
            0.0, 0.0, gyro_cov
        ]
        
        # Orientation unknown - standard is to set -1 to first element if provided, 
        # but pure IMU usually leaves it 0 or identity if not fusing yet? 
        # REP-145: If no orientation estimate, set [0] to -1.
        imu_msg.orientation_covariance[0] = -1.0
        
        self.imu_pub.publish(imu_msg)

        # Mag Message
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self.frame_id
        
        mag_msg.magnetic_field.x = data['mag'][0] * 1e-6 # uT to Tesla
        mag_msg.magnetic_field.y = data['mag'][1] * 1e-6
        mag_msg.magnetic_field.z = data['mag'][2] * 1e-6
        
        mag_std = self.get_parameter('mag_std').value
        mag_cov = (mag_std * 1e-6) ** 2
        mag_msg.magnetic_field_covariance = [
            mag_cov, 0.0, 0.0,
            0.0, mag_cov, 0.0,
            0.0, 0.0, mag_cov
        ]
        
        self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bmx160DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
