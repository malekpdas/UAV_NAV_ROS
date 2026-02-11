#!/usr/bin/env python3
"""
Node to plot real-time orientation (roll, pitch, yaw) from Odometry data.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.spatial.transform import Rotation as R

class OdomOrientationPlotter(Node):
    def __init__(self):
        super().__init__('odom_orientation_plotter')
        
        # Declare Parameters
        self.declare_all_parameters()
        self.load_parameters()    

        self.subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        
        # Plotting Setup
        plt.ion()
        self.fig = plt.figure(facecolor='black') # Set window background to black
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('black') # Set plot background to black
        
        # Store orientation as a rotation object
        self.current_rotation = R.from_euler('xyz', [0, 0, 0])
        
        # Timer to update plot
        self.timer = self.create_timer(self.update_period, self.update_plot)

    def declare_all_parameters(self):
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('update_period', 0.03)
        self.declare_parameter('box_dimensions.length', 4.0)
        self.declare_parameter('box_dimensions.width', 2.0)
        self.declare_parameter('box_dimensions.height', 0.8)
        self.declare_parameter('colors.face_colors', ['#444444', '#00FF00', '#FF0000', '#FF0000', '#0000FF', '#0000FF'])
        self.declare_parameter('colors.text_color', '#00FF00')
        self.declare_parameter('colors.edge_color', 'white')

    def load_parameters(self):
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.update_period = self.get_parameter('update_period').get_parameter_value().double_value
        self.l = self.get_parameter('box_dimensions.length').get_parameter_value().double_value
        self.w = self.get_parameter('box_dimensions.width').get_parameter_value().double_value
        self.h = self.get_parameter('box_dimensions.height').get_parameter_value().double_value
        self.face_colors = self.get_parameter('colors.face_colors').get_parameter_value().string_array_value
        self.text_color = self.get_parameter('colors.text_color').get_parameter_value().string_value
        self.edge_color = self.get_parameter('colors.edge_color').get_parameter_value().string_value

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_rotation = R.from_quat([q.x, q.y, q.z, q.w])

    def update_plot(self):
        self.ax.cla()

        # 1. Background and Axis Removal
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        self.ax.axis('off')
        self.ax.grid(False)
        
        # 2. Camera and Limits
        self.ax.view_init(elev=0, azim=0)
        self.ax.set_xlim([-4, 4])
        self.ax.set_ylim([-4, 4])
        self.ax.set_zlim([-4, 4])
        # self.ax.set_box_aspect([1,1,1])
        
        # Invert for NED
        self.ax.invert_xaxis()
        self.ax.invert_zaxis()
        
        # 3. Calculate Roll, Pitch, Yaw for display
        # 'xyz' corresponds to Roll, Pitch, Yaw
        roll, pitch, yaw = self.current_rotation.as_euler('xyz', degrees=True)
        
        # 4. Add Text Overlay (Fixed on screen)
        # Positioned at top-left (0.05, 0.95)
        stats_text = f"ROLL:  {roll:>6.1f}°\nPITCH: {pitch:>6.1f}°\nYAW:   {yaw:>6.1f}°"
        self.ax.text2D(0.05, 0.95, stats_text, 
                       transform=self.ax.transAxes, 
                       color=self.text_color,
                       fontsize=12, 
                       family='monospace',   # Monospaced for alignment
                       fontweight='bold',
                       verticalalignment='top')

        # 5. Define rectangular box
        v = np.array([
            [-self.l, -self.w, -self.h], [self.l, -self.w, -self.h], [self.l, self.w, -self.h], [-self.l, self.w, -self.h],
            [-self.l, -self.w, self.h],  [self.l, -self.w, self.h],  [self.l, self.w, self.h],  [self.l, self.w, self.h]
        ])

        rv = self.current_rotation.apply(v)
        faces = [
            [rv[0], rv[1], rv[2], rv[3]], [rv[4], rv[5], rv[6], rv[7]],
            [rv[0], rv[1], rv[5], rv[4]], [rv[2], rv[3], rv[7], rv[6]],
            [rv[0], rv[3], rv[7], rv[4]], [rv[1], rv[2], rv[6], rv[5]]
        ]

        poly = Poly3DCollection(faces, facecolors=self.face_colors, linewidths=1, edgecolors=self.edge_color, alpha=1.0)
        self.ax.add_collection3d(poly)

        # 6. Local Axes
        origin = np.zeros(3)
        for i, color in enumerate(['#FF3333', '#33FF33', '#3333FF']):
            vec = np.zeros(3)
            vec[i] = 1.0 
            rotated_vec = self.current_rotation.apply(vec)
            self.ax.quiver(*origin, *rotated_vec, color=color, length=0.8, normalize=True)

        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = OdomOrientationPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()