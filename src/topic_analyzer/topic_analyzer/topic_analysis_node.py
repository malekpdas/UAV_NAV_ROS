#!/usr/bin/env python3
"""
Node for comparing GPS and Odometry data with visualization.
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import numpy as np
import math

def quat_to_euler(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class OdomGpsComparator(Node):
    def __init__(self):
        super().__init__('odom_gps_comparator')

        # Data storage - no window trimming, keeps everything until end
        self.data = {
            'gps_t': [], 'gps_x': [], 'gps_y': [], 'gps_z': [],
            'gps_x_std': [], 'gps_y_std': [], 'gps_z_std': [],
            'gps_vt': [], 'gps_vx': [], 'gps_vy': [], 'gps_vz': [],
            'gps_vx_std': [], 'gps_vy_std': [], 'gps_vz_std': [],
            'odom_t': [], 'odom_x': [], 'odom_y': [], 'odom_z': [],
            'odom_x_std': [], 'odom_y_std': [], 'odom_z_std': [],
            'odom_vx': [], 'odom_vy': [], 'odom_vz': [],
            'odom_vx_std': [], 'odom_vy_std': [], 'odom_vz_std': [],
            'imu_t': [], 'imu_ax': [], 'imu_ay': [], 'imu_az': [],
            'imu_ax_std': [], 'imu_ay_std': [], 'imu_az_std': [],
            'roll': [], 'pitch': [], 'yaw': []
        }

        self.gps_origin = None

        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/gps/vel', self.cb_gps_vel, 10)
        self.create_subscription(Odometry, '/ekf/odom', self.cb_odom, 10)
        self.create_subscription(Imu, '/ekf/filtered_imu', self.cb_imu, 50)

        self.get_logger().info('Data collection started. Press Ctrl+C to stop and plot.')

    def msg_to_dt(self, stamp):
        return datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9)

    # ---------------- CALLBACKS (Data Collection) ----------------
    def cb_gps_fix(self, msg):
        if math.isnan(msg.latitude): return
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude, msg.altitude)

        R = 6378137.0
        lat0, lon0 = map(math.radians, self.gps_origin[:2])
        lat, lon = map(math.radians, [msg.latitude, msg.longitude])

        self.data['gps_t'].append(self.msg_to_dt(msg.header.stamp))
        self.data['gps_x'].append((lat - lat0) * R)
        self.data['gps_y'].append((lon - lon0) * R * math.cos(lat0))
        self.data['gps_z'].append(msg.altitude - self.gps_origin[2])
        self.data['gps_x_std'].append(math.sqrt(msg.position_covariance[4]))
        self.data['gps_y_std'].append(math.sqrt(msg.position_covariance[0]))
        self.data['gps_z_std'].append(math.sqrt(msg.position_covariance[8]))

    def cb_gps_vel(self, msg):
        self.data['gps_vt'].append(self.msg_to_dt(msg.header.stamp))
        self.data['gps_vx'].append(msg.twist.twist.linear.y)
        self.data['gps_vy'].append(msg.twist.twist.linear.x)
        self.data['gps_vz'].append(-msg.twist.twist.linear.z)
        self.data['gps_vx_std'].append(math.sqrt(msg.twist.covariance[0]))
        self.data['gps_vy_std'].append(math.sqrt(msg.twist.covariance[7]))
        self.data['gps_vz_std'].append(math.sqrt(msg.twist.covariance[14]))

    def cb_odom(self, msg):
        self.data['odom_t'].append(self.msg_to_dt(msg.header.stamp))
        self.data['odom_x'].append(msg.pose.pose.position.x)
        self.data['odom_y'].append(msg.pose.pose.position.y)
        self.data['odom_z'].append(-msg.pose.pose.position.z)
        self.data['odom_x_std'].append(math.sqrt(msg.pose.covariance[0]))
        self.data['odom_y_std'].append(math.sqrt(msg.pose.covariance[7]))
        self.data['odom_z_std'].append(math.sqrt(msg.pose.covariance[14]))
        self.data['odom_vx'].append(msg.twist.twist.linear.x)
        self.data['odom_vy'].append(msg.twist.twist.linear.y)
        self.data['odom_vz'].append(msg.twist.twist.linear.z)
        self.data['odom_vx_std'].append(math.sqrt(msg.twist.covariance[0]))
        self.data['odom_vy_std'].append(math.sqrt(msg.twist.covariance[7]))
        self.data['odom_vz_std'].append(math.sqrt(msg.twist.covariance[14]))
        q = msg.pose.pose.orientation
        r, p, y = quat_to_euler([q.x, q.y, q.z, q.w])
        self.data['roll'].append(math.degrees(r))
        self.data['pitch'].append(math.degrees(p))
        self.data['yaw'].append(math.degrees(y))

    def cb_imu(self, msg):
        self.data['imu_t'].append(self.msg_to_dt(msg.header.stamp))
        self.data['imu_ax'].append(msg.linear_acceleration.x)
        self.data['imu_ay'].append(msg.linear_acceleration.y)
        self.data['imu_az'].append(msg.linear_acceleration.z)
        cov = msg.linear_acceleration_covariance
        self.data['imu_ax_std'].append(math.sqrt(cov[0]) if cov[0] > 0 else 0)
        self.data['imu_ay_std'].append(math.sqrt(cov[4]) if cov[4] > 0 else 0)
        self.data['imu_az_std'].append(math.sqrt(cov[8]) if cov[8] > 0 else 0)

    # ---------------- FINAL PLOT ----------------
    def generate_final_plot(self):
        if not self.data['odom_t']:
            self.get_logger().warn("No data collected to plot.")
            return

        fig, axs = plt.subplots(3, 4, figsize=(18, 9), sharex=True)
        fig.suptitle('GPS vs EKF State Summary', fontsize=16)

        ot = np.array(self.data['odom_t'])
        gt = np.array(self.data['gps_t'])
        gvt = np.array(self.data['gps_vt'])
        it = np.array(self.data['imu_t'])

        dims, vdims, adims, att = ['x','y','z'], ['vx','vy','vz'], ['ax','ay','az'], ['roll','pitch','yaw']

        for i in range(3):
            # Column 0: Position
            ax = axs[i, 0]
            o, s = np.array(self.data[f'odom_{dims[i]}']), np.array(self.data[f'odom_{dims[i]}_std'])
            ax.plot(ot, o, 'r', label='Odom')
            ax.fill_between(ot, o-s, o+s, color='r', alpha=0.15)
            if len(gt):
                g, gs = np.array(self.data[f'gps_{dims[i]}']), np.array(self.data[f'gps_{dims[i]}_std'])
                ax.step(gt, g, 'g', where='post', label='GPS')
                ax.fill_between(gt, g-gs, g+gs, color='g', alpha=0.15, step='post')
            ax.set_ylabel(f'{dims[i].upper()} [m]')
            ax.grid(True, linestyle=':')

            # Column 1: Velocity
            ax = axs[i, 1]
            o, s = np.array(self.data[f'odom_{vdims[i]}']), np.array(self.data[f'odom_{vdims[i]}_std'])
            ax.plot(ot, o, 'r')
            ax.fill_between(ot, o-s, o+s, color='r', alpha=0.15)
            if len(gvt):
                g, gs = np.array(self.data[f'gps_{vdims[i]}']), np.array(self.data[f'gps_{vdims[i]}_std'])
                ax.step(gvt, g, 'g', where='post')
                ax.fill_between(gvt, g-gs, g+gs, color='g', alpha=0.15, step='post')
            ax.set_ylabel(f'{vdims[i]} [m/s]')
            ax.grid(True, linestyle=':')

            # Column 2: Accel
            ax = axs[i, 2]
            a, s = np.array(self.data[f'imu_{adims[i]}']), np.array(self.data[f'imu_{adims[i]}_std'])
            ax.plot(it, a, 'b')
            ax.fill_between(it, a-s, a+s, color='b', alpha=0.2)
            ax.set_ylabel(f'{adims[i]} [m/s²]')
            ax.grid(True, linestyle=':')

            # Column 3: Attitude
            ax = axs[i, 3]
            a = np.array(self.data[att[i]])
            ax.plot(ot[:len(a)], a, 'k')
            ax.set_ylabel(f'{att[i]} [deg]')
            ax.grid(True, linestyle=':')

        for j in range(4):
            axs[2, j].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            fig.autofmt_xdate()

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

def main():
    rclpy.init()
    node = OdomGpsComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down and generating plot...')
    finally:
        # Plotting happens after spin stops
        node.generate_final_plot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()