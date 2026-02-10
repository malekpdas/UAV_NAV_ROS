#!/usr/bin/env python3
"""
Robust MotionCal-style magnetometer calibration for ROS2 + Data Logging

Key features:
- Subscribes to: imu/mag (sensor_msgs/msg/MagneticField)
- LOGS raw data to 'mag_log.txt' for offline processing
- Collects raw magnetometer samples
- Uses NONLINEAR magnitude-constrained calibration
- Estimates:
    * Hard-iron bias b
    * Full soft-iron matrix A (SPD by construction)
- **Constraint:** Forces calibrated data to lie on the Unit Sphere (Radius = 1.0)
- Stops automatically when spherical coverage gap < threshold
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from collections import deque
from scipy.optimize import least_squares
import time


def unit(v):
    n = la.norm(v)
    return v if n < 1e-9 else v / n


def compute_gap_percent(vectors, bins=32):
    if len(vectors) < 10:
        return 100.0

    V = np.array([unit(v) for v in vectors])
    theta = np.arccos(np.clip(V[:, 2], -1.0, 1.0))
    phi = np.arctan2(V[:, 1], V[:, 0])

    H, _, _ = np.histogram2d(theta, phi, bins=bins)
    return 100.0 * np.sum(H == 0) / H.size


def set_equal_2d(ax, x, y, pad=0.05):
    xmin, xmax = np.min(x), np.max(x)
    ymin, ymax = np.min(y), np.max(y)

    dx = xmax - xmin
    dy = ymax - ymin
    d = max(dx, dy) * (1.0 + pad)

    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)

    ax.set_xlim(cx - d / 2, cx + d / 2)
    ax.set_ylim(cy - d / 2, cy + d / 2)
    ax.set_aspect('equal', adjustable='box')


# -----------------------------------------------------------------------------
# Robust nonlinear magnetometer calibration (Fixed for Unit Sphere)
# -----------------------------------------------------------------------------

def mag_calibrate_nonlinear(M):
    """
    Nonlinear magnitude-constrained calibration:
        || A (m - b) || = 1.0
    """
    
    # 1. Initial Bias Guess (average of min/max is robust)
    M_min = np.min(M, axis=0)
    M_max = np.max(M, axis=0)
    b0 = 0.5 * (M_max + M_min)

    # Robustness check: Ensure data is not static
    if np.allclose(M_max, M_min):
        b = np.array([0.0, 0.0, 0.0])
        A = np.eye(3)
        return b, A

    # 2. Initial Scale Guess: inverse of current radius
    M_centered = M - b0
    avg_radius = np.mean(la.norm(M_centered, axis=1))
    
    # Robustness check: Ensure non-zero average radius
    scale_guess = 1.0 / avg_radius if avg_radius > 1e-9 else 1.0

    # 3. Initialize L (Cholesky factor of A)
    L0 = np.eye(3) * np.sqrt(scale_guess)

    # Parameter vector:
    # [b0, b1, b2, L00, L10, L11, L20, L21, L22]
    p0 = np.array([
        b0[0], b0[1], b0[2],
        L0[0, 0],
        L0[1, 0], L0[1, 1],
        L0[2, 0], L0[2, 1], L0[2, 2]
    ])

    def unpack(p):
        b = p[0:3]
        L = np.zeros((3, 3))
        L[0, 0] = p[3]
        L[1, 0] = p[4]; L[1, 1] = p[5]
        L[2, 0] = p[6]; L[2, 1] = p[7]; L[2, 2] = p[8]
        return b, L

    def residuals(p):
        b, L = unpack(p)
        A = L @ L.T
        Mc = (M - b) @ A.T
        radii = la.norm(Mc, axis=1)
        return radii - 1.0

    res = least_squares(
        residuals,
        p0,
        method='lm',
        max_nfev=500
    )

    b, L = unpack(res.x)
    A = L @ L.T
    return b, A


# -----------------------------------------------------------------------------
# ROS2 Node
# -----------------------------------------------------------------------------

class MagCalNode(Node):

    def __init__(self):
        super().__init__('mag_motioncal_nonlinear')

        self.declare_all_parameters()
        self.load_parameters()

        topics = [self.mag_topic]
        status, missing_topics = self.wait_for_topics(topics, self.timeout_sec, self.check_rate)
        if not status:
            self.get_logger().error(f'Topic(s) {missing_topics} not found, shutting down')
            self._ok = False
            return

        self.sub = self.create_subscription(
            MagneticField,
            self.mag_topic,
            self.mag_cb,
            50
        )

        self.samples = deque(maxlen=self.max_samples)
        self.counter = 0
        self.done = False

        self.bias = None
        self.A = None
        
        plt.ion()
        self.fig, self.axes = plt.subplots(1, 3, figsize=(12, 4))
        self.ax_xy, self.ax_xz, self.ax_yz = self.axes

    def wait_for_topics(self, topics, timeout_sec, rate_hz):
            start = time.time()
            period = 1.0 / rate_hz

            while rclpy.ok():
                
                all_topics_ready = True
                missing_topics = []

                for topic in topics:
                    pubs = self.get_publishers_info_by_topic(topic)
                    if len(pubs) == 0:
                        all_topics_ready = False
                        missing_topics.append(topic)
                
                if all_topics_ready:
                    return True, missing_topics  # topic is being published

                if time.time() - start > timeout_sec:
                    return False, missing_topics  # timeout

                time.sleep(period)
                
    def declare_all_parameters(self):
        self.declare_parameter('topics.mag.value', 'imu/mag')
        self.declare_parameter('timeout_sec.value', 5.0)
        self.declare_parameter('check_rate.value', 10.0)

        self.declare_parameter('max_samples.value', 50000)
        self.declare_parameter('min_samples.value', 400)
        self.declare_parameter('angular_bins.value', 6)
        self.declare_parameter('gap_threshold_percent.value', 1.0)
        self.declare_parameter('plot_every_n.value', 50)

    def load_parameters(self):
        self.mag_topic = self.get_parameter('topics.mag.value').value
        self.timeout_sec = self.get_parameter('timeout_sec.value').value
        self.check_rate = self.get_parameter('check_rate.value').value

        self.max_samples = self.get_parameter('max_samples.value').value
        self.min_samples = self.get_parameter('min_samples.value').value
        self.angular_bins = self.get_parameter('angular_bins.value').value
        self.gap_threshold_percent = self.get_parameter('gap_threshold_percent.value').value
        self.plot_every_n = self.get_parameter('plot_every_n.value').value

    def mag_cb(self, msg):
        if self.done:
            return

        m = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

        self.samples.append(m)
        self.counter += 1

        if self.counter % self.plot_every_n == 0:
            self.update()

    def update(self):
        if len(self.samples) < self.min_samples:
            # Clear axes and show status if not enough data
            for ax in self.axes:
                ax.cla()
                ax.text(0.5, 0.5, f"Collecting data: {len(self.samples)}/{self.min_samples}", 
                        horizontalalignment='center', verticalalignment='center', transform=ax.transAxes)
            self.fig.suptitle("COLLECTING DATA (Move sensor in all orientations!)")
            plt.pause(0.001)
            return

        M = np.array(self.samples)
        
        # Calculate Hard-Iron Bias estimate for plotting (Center of data mass)
        M_min = np.min(M, axis=0)
        M_max = np.max(M, axis=0)
        M_avg = 0.5 * (M_max + M_min) # This is a better center estimate than mean

        mags = la.norm(M, axis=1)
        wobble = np.std(mags)
        variance = np.mean(np.var(M, axis=0))
        gap_pct = compute_gap_percent(M, self.angular_bins)

        for ax in self.axes:
            ax.cla()

        # Raw plots
        self.ax_xy.scatter(M[:, 0], M[:, 1], s=2)
        self.ax_xz.scatter(M[:, 0], M[:, 2], s=2)
        self.ax_yz.scatter(M[:, 1], M[:, 2], s=2)
        
        # Highlight estimated hard-iron bias (the center)
        self.ax_xy.plot(M_avg[0], M_avg[1], 'rx', markersize=10, label='Est. Bias')
        self.ax_xz.plot(M_avg[0], M_avg[2], 'rx', markersize=10)
        self.ax_yz.plot(M_avg[1], M_avg[2], 'rx', markersize=10)
        
        self.ax_xy.legend()

        set_equal_2d(self.ax_xy, M[:, 0], M[:, 1])
        set_equal_2d(self.ax_xz, M[:, 0], M[:, 2])
        set_equal_2d(self.ax_yz, M[:, 1], M[:, 2])

        self.ax_xy.set_title('XY raw (Look for centered circle)')
        self.ax_xz.set_title('XZ raw')
        self.ax_yz.set_title('YZ raw')

        # Use title to show the critical metrics
        self.fig.suptitle(
            f"N={len(M)} | Gap={gap_pct:.2f}% (Target < {self.gap_threshold_percent}%) | Wobble={wobble:.3e}"
        )

        plt.pause(0.001)

        # Stop criterion
        if gap_pct < self.gap_threshold_percent:
            self.finish()

    def finish(self):
        self.get_logger().info('Calibration complete — running nonlinear solver')
        self.done = True

        M = np.array(self.samples)
        self.bias, self.A = mag_calibrate_nonlinear(M)

        Mc = (M - self.bias) @ self.A.T
        
        # Validation stats
        radii = la.norm(Mc, axis=1)
        mean_r = np.mean(radii)
        std_r = np.std(radii)

        print('\n===== MAG CALIBRATION RESULT =====')
        print('Hard-iron bias:')
        print(self.bias)
        print('\nSoft-iron matrix A:')
        print(self.A)
        print('\nCHECK:')
        print(f'Mean Radius (target=1.0): {mean_r:.5f}')
        print(f'Std Deviation (target=0.0): {std_r:.5e}')
        print('=================================\n')

        # Final calibrated plots
        for ax in self.axes:
            ax.cla()

        self.ax_xy.scatter(Mc[:, 0], Mc[:, 1], s=2)
        self.ax_xz.scatter(Mc[:, 0], Mc[:, 2], s=2)
        self.ax_yz.scatter(Mc[:, 1], Mc[:, 2], s=2)
        
        # Add unit circle to calibrated plot for visual validation
        t = np.linspace(0, 2 * np.pi, 100)
        self.ax_xy.plot(np.cos(t), np.sin(t), 'r--', alpha=0.5, label='Unit Circle')
        self.ax_xz.plot(np.cos(t), np.sin(t), 'r--', alpha=0.5)
        self.ax_yz.plot(np.cos(t), np.sin(t), 'r--', alpha=0.5)

        set_equal_2d(self.ax_xy, Mc[:, 0], Mc[:, 1])
        set_equal_2d(self.ax_xz, Mc[:, 0], Mc[:, 2])
        set_equal_2d(self.ax_yz, Mc[:, 1], Mc[:, 2])

        self.ax_xy.set_title('XY calibrated')
        self.ax_xz.set_title('XZ calibrated')
        self.ax_yz.set_title('YZ calibrated')

        plt.ioff()
        plt.show()

# -----------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MagCalNode()

    if not getattr(node, '_ok', True):
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node() 
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()