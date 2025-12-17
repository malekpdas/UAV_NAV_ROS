import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import numpy as np
import math

class TopicPlotter(Node):
    def __init__(self):
        super().__init__('ekf2_node')
        
        # Data storage
        self.data = {
            'imu': {'t': [], 'accel': [], 'accel_std': [], 'gyro': [], 'gyro_std': [], 'accel_mag': []},
            'mag': {'t': [], 'field': [], 'mag_mag': []},
            'gps': {'lat': [], 'lon': [], 'alt': []},
            'odom': {'t': [], 'pos': [], 'rpy': []} # rpy in degrees
        }
        self.start_time = None

        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.cb_mag, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps_fix, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        
        # Optional subs (initialized for completeness)
        self.create_subscription(TwistWithCovarianceStamped, '/gps/vel', lambda m: None, 10)
        self.create_subscription(AccelWithCovarianceStamped, '/accel/filtered', lambda m: None, 10)
        self.create_subscription(Float64, '/heading/estimated', lambda m: None, 10)
        self.create_subscription(Float64, '/heading/gps', lambda m: None, 10)
        
        self.get_logger().info("🚀 Hybrid EKF Node Started. Collecting data for plotting...")

    def get_t(self, header):
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def cb_imu(self, msg: Imu):
        t = self.get_t(msg.header)
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [math.degrees(msg.angular_velocity.x), 
                math.degrees(msg.angular_velocity.y), 
                math.degrees(msg.angular_velocity.z)]
        
        # Extract 3-sigma uncertainty (sqrt of diagonal of covariance)
        acc_std = [3 * math.sqrt(msg.linear_acceleration_covariance[i]) for i in [0, 4, 8]]
        gyro_std = [3 * math.sqrt(math.degrees(math.degrees(msg.angular_velocity_covariance[i]))) for i in [0, 4, 8]]

        self.data['imu']['t'].append(t)
        self.data['imu']['accel'].append(acc)
        self.data['imu']['accel_std'].append(acc_std)
        self.data['imu']['gyro'].append(gyro)
        self.data['imu']['gyro_std'].append(gyro_std)
        self.data['imu']['accel_mag'].append(np.linalg.norm(acc))

    def cb_mag(self, msg: MagneticField):
        t = self.get_t(msg.header)
        field = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
        self.data['mag']['t'].append(t)
        self.data['mag']['field'].append(field)
        self.data['mag']['mag_mag'].append(np.linalg.norm(field))

    def cb_gps_fix(self, msg: NavSatFix):
        if not math.isnan(msg.latitude):
            self.data['gps']['lat'].append(msg.latitude)
            self.data['gps']['lon'].append(msg.longitude)
            self.data['gps']['alt'].append(msg.altitude)

    def cb_odom(self, msg: Odometry):
        t = self.get_t(msg.header)
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        
        # Quaternion to Euler (Roll, Pitch, Yaw)
        q = msg.pose.pose.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(sinp)) if abs(sinp) <= 1 else math.degrees(math.copysign(math.pi/2, sinp))

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.data['odom']['t'].append(t)
        self.data['odom']['pos'].append(pos)
        self.data['odom']['rpy'].append([roll, pitch, yaw])

    def plot_all(self):
        self.get_logger().info("📊 Generating plots...")
        
        # 1. IMU Plots
        if self.data['imu']['t']:
            fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
            t = self.data['imu']['t']
            acc = np.array(self.data['imu']['accel'])
            acc_std = np.array(self.data['imu']['accel_std'])
            gyro = np.array(self.data['imu']['gyro'])
            gyro_std = np.array(self.data['imu']['gyro_std'])

            for i, label in enumerate(['X', 'Y', 'Z']):
                axs[0].plot(t, acc[:, i], label=f'Accel {label}')
                axs[0].fill_between(t, acc[:, i]-acc_std[:, i], acc[:, i]+acc_std[:, i], alpha=0.2)
            axs[0].set_ylabel('Accel (m/s²)')
            axs[0].legend()
            axs[0].grid()

            axs[1].plot(t, self.data['imu']['accel_mag'], color='black', label='Magnitude')
            axs[1].set_ylabel('Accel Mag')
            axs[1].grid()
            
            for i, label in enumerate(['R', 'P', 'Y']):
                axs[2].plot(t, gyro[:, i], label=f'Gyro {label}')
                axs[2].fill_between(t, gyro[:, i]-gyro_std[:, i], gyro[:, i]+gyro_std[:, i], alpha=0.2)
            axs[2].set_ylabel('Gyro (DPS)')
            axs[2].set_xlabel('Time (s)')
            axs[2].legend()
            axs[2].grid()
            fig.suptitle('IMU Data (with 3-sigma bounds)')

        # 2. Mag Plots
        if self.data['mag']['t']:
            fig2, axs2 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
            mag_f = np.array(self.data['mag']['field'])
            for i, label in enumerate(['X', 'Y', 'Z']):
                axs2[0].plot(self.data['mag']['t'], mag_f[:, i], label=f'Mag {label}')
            axs2[0].set_ylabel('Field (Tesla)')
            axs2[0].legend()
            axs2[0].grid()
            axs2[1].plot(self.data['mag']['t'], self.data['mag']['mag_mag'], color='red')
            axs2[1].set_ylabel('Mag Magnitude')
            axs2[1].grid()
            fig2.suptitle('Magnetic Field')

        # 3. GPS Plot
        if self.data['gps']['lat']:
            plt.figure(figsize=(8, 6))
            sc = plt.scatter(self.data['gps']['lon'], self.data['gps']['lat'], c=self.data['gps']['alt'], cmap='terrain')
            plt.colorbar(sc, label='Altitude (m)')
            plt.xlabel('Longitude')
            plt.ylabel('Latitude')
            plt.title('GPS Fix Path')
            plt.grid()

        # 4. Odometry Plots
        if self.data['odom']['t']:
            fig4 = plt.figure(figsize=(12, 10))
            # 2D Scatter
            ax_pos = plt.subplot(2, 2, 1)
            pos = np.array(self.data['odom']['pos'])
            sc_odom = ax_pos.scatter(pos[:, 0], pos[:, 1], c=pos[:, 2], cmap='viridis')
            plt.colorbar(sc_odom, ax=ax_pos, label='Z (m)')
            ax_pos.set_title('Odom 2D Path (X/Y)')

            # RPY
            rpy = np.array(self.data['odom']['rpy'])
            t_odom = self.data['odom']['t']
            titles = ['Roll (deg)', 'Pitch (deg)', 'Heading/Yaw (deg)']
            for i in range(3):
                ax = plt.subplot(2, 2, i+2)
                ax.plot(t_odom, rpy[:, i])
                ax.set_title(titles[i])
                ax.grid()
            plt.tight_layout()

        plt.show()

def main():
    rclpy.init()
    node = TopicPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping... generating plots.")
    finally:
        node.plot_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()