import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
import os

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (rotation around z-axis)."""
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw

def parse_pose_covariance(cov_str):
    """Parse pose covariance string and extract useful uncertainty metrics."""
    try:
        values = list(map(float, str(cov_str).split()))
        if len(values) >= 36:
            # 6x6 covariance matrix: [x, y, z, roll, pitch, yaw]
            # Diagonal elements: [0]=x², [7]=y², [14]=z², [21]=roll², [28]=pitch², [35]=yaw²
            return {
                'pos_x_var': values[0],
                'pos_y_var': values[7],
                'pos_z_var': values[14],
                'roll_var': values[21],
                'pitch_var': values[28],
                'yaw_var': values[35]
            }
    except:
        pass
    return None

def parse_twist_covariance(cov_str):
    """Parse twist covariance string and extract velocity uncertainty metrics."""
    try:
        values = list(map(float, str(cov_str).split()))
        if len(values) >= 36:
            # 6x6 covariance matrix: [vx, vy, vz, wx, wy, wz]
            return {
                'vel_x_var': values[0],
                'vel_y_var': values[7],
                'vel_z_var': values[14],
                'ang_x_var': values[21],
                'ang_y_var': values[28],
                'ang_z_var': values[35]
            }
    except:
        pass
    return None

def analyze_odometry(csv_path, output_dir=None):
    """Analyze and plot odometry data from CSV file."""
    
    # Read CSV
    print(f"Reading {csv_path}...")
    df = pd.read_csv(csv_path)
    
    # Convert timestamp to seconds (relative to start)
    df['time_sec'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9
    
    # Calculate yaw from quaternion
    df['yaw'] = quaternion_to_yaw(
        df['pose.pose.orientation.x'],
        df['pose.pose.orientation.y'],
        df['pose.pose.orientation.z'],
        df['pose.pose.orientation.w']
    )
    df['yaw_deg'] = np.degrees(df['yaw'])
    
    # Calculate velocities magnitude
    df['linear_velocity'] = np.sqrt(
        df['twist.twist.linear.x']**2 + 
        df['twist.twist.linear.y']**2 + 
        df['twist.twist.linear.z']**2
    )
    df['angular_velocity'] = np.sqrt(
        df['twist.twist.angular.x']**2 + 
        df['twist.twist.angular.y']**2 + 
        df['twist.twist.angular.z']**2
    )
    
    # Parse covariance data
    pose_cov_data = df['pose.covariance'].apply(parse_pose_covariance)
    twist_cov_data = df['twist.covariance'].apply(parse_twist_covariance)
    
    has_pose_cov = pose_cov_data.iloc[0] is not None
    has_twist_cov = twist_cov_data.iloc[0] is not None
    
    if has_pose_cov:
        df['pos_x_std'] = pose_cov_data.apply(lambda x: np.sqrt(x['pos_x_var']) if x else np.nan)
        df['pos_y_std'] = pose_cov_data.apply(lambda x: np.sqrt(x['pos_y_var']) if x else np.nan)
        df['pos_z_std'] = pose_cov_data.apply(lambda x: np.sqrt(x['pos_z_var']) if x else np.nan)
        df['yaw_std'] = pose_cov_data.apply(lambda x: np.sqrt(x['yaw_var']) if x else np.nan)
        df['horizontal_pos_uncertainty'] = np.sqrt(df['pos_x_std']**2 + df['pos_y_std']**2)
    
    if has_twist_cov:
        df['vel_x_std'] = twist_cov_data.apply(lambda x: np.sqrt(x['vel_x_var']) if x else np.nan)
        df['vel_y_std'] = twist_cov_data.apply(lambda x: np.sqrt(x['vel_y_var']) if x else np.nan)
        df['vel_z_std'] = twist_cov_data.apply(lambda x: np.sqrt(x['vel_z_var']) if x else np.nan)
        df['ang_z_std'] = twist_cov_data.apply(lambda x: np.sqrt(x['ang_z_var']) if x else np.nan)
        df['horizontal_vel_uncertainty'] = np.sqrt(df['vel_x_std']**2 + df['vel_y_std']**2)
    
    # Calculate distance traveled
    dx = np.diff(df['pose.pose.position.x'])
    dy = np.diff(df['pose.pose.position.y'])
    dz = np.diff(df['pose.pose.position.z'])
    distances = np.sqrt(dx**2 + dy**2 + dz**2)
    total_distance = np.sum(distances)
    
    # Calculate position drift (deviation from mean)
    if len(df) > 10:
        df['pos_drift'] = np.sqrt(
            (df['pose.pose.position.x'] - df['pose.pose.position.x'].mean())**2 +
            (df['pose.pose.position.y'] - df['pose.pose.position.y'].mean())**2
        )
    
    # Print statistics
    print("\n" + "="*60)
    print("ODOMETRY ANALYSIS")
    print("="*60)
    print(f"Total messages: {len(df)}")
    print(f"Duration: {df['time_sec'].iloc[-1]:.2f} seconds")
    print(f"Average rate: {len(df) / df['time_sec'].iloc[-1]:.2f} Hz")
    print(f"\nPosition range:")
    print(f"  X: [{df['pose.pose.position.x'].min():.2f}, {df['pose.pose.position.x'].max():.2f}] m")
    print(f"  Y: [{df['pose.pose.position.y'].min():.2f}, {df['pose.pose.position.y'].max():.2f}] m")
    print(f"  Z: [{df['pose.pose.position.z'].min():.2f}, {df['pose.pose.position.z'].max():.2f}] m")
    print(f"\nTotal distance traveled: {total_distance:.2f} m")
    print(f"\nLinear velocity:")
    print(f"  Mean: {df['linear_velocity'].mean():.3f} m/s")
    print(f"  Max: {df['linear_velocity'].max():.3f} m/s")
    print(f"\nAngular velocity:")
    print(f"  Mean: {df['angular_velocity'].mean():.3f} rad/s")
    print(f"  Max: {df['angular_velocity'].max():.3f} rad/s")
    print(f"\nYaw angle range: [{df['yaw_deg'].min():.1f}°, {df['yaw_deg'].max():.1f}°]")
    
    if has_pose_cov:
        print(f"\nPosition Uncertainty (1σ):")
        print(f"  Horizontal: {df['horizontal_pos_uncertainty'].mean():.3f} m (mean), {df['horizontal_pos_uncertainty'].max():.3f} m (max)")
        print(f"  Vertical: {df['pos_z_std'].mean():.3f} m (mean), {df['pos_z_std'].max():.3f} m (max)")
        print(f"  Yaw: {np.degrees(df['yaw_std'].mean()):.2f}° (mean), {np.degrees(df['yaw_std'].max()):.2f}° (max)")
    
    if has_twist_cov:
        print(f"\nVelocity Uncertainty (1σ):")
        print(f"  Horizontal: {df['horizontal_vel_uncertainty'].mean():.4f} m/s (mean)")
        print(f"  Angular (yaw rate): {df['ang_z_std'].mean():.4f} rad/s (mean)")
    
    print("="*60 + "\n")
    
    # Create main plots
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # 1. 2D Trajectory (X-Y plane) with uncertainty ellipses
    ax1 = fig.add_subplot(gs[0, :2])
    scatter = ax1.scatter(df['pose.pose.position.x'], 
                         df['pose.pose.position.y'],
                         c=df['time_sec'], 
                         cmap='viridis', 
                         s=10, 
                         alpha=0.6)
    
    # Add uncertainty ellipses (subsample for clarity)
    if has_pose_cov:
        N = max(1, len(df) // 20)
        df_sub = df.iloc[::N]
        for _, row in df_sub.iterrows():
            ellipse = plt.Circle((row['pose.pose.position.x'], row['pose.pose.position.y']),
                                radius=row['horizontal_pos_uncertainty'],
                                color='red', fill=False, alpha=0.3, linewidth=1)
            ax1.add_patch(ellipse)
    
    ax1.plot(df['pose.pose.position.x'].iloc[0], 
             df['pose.pose.position.y'].iloc[0], 
             'go', markersize=10, label='Start')
    ax1.plot(df['pose.pose.position.x'].iloc[-1], 
             df['pose.pose.position.y'].iloc[-1], 
             'ro', markersize=10, label='End')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('2D Trajectory with Uncertainty')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Time (s)')
    
    # 2. Position over time with uncertainty bands
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(df['time_sec'], df['pose.pose.position.x'], label='X', alpha=0.7)
    ax2.plot(df['time_sec'], df['pose.pose.position.y'], label='Y', alpha=0.7)
    ax2.plot(df['time_sec'], df['pose.pose.position.z'], label='Z', alpha=0.7)
    
    if has_pose_cov:
        ax2.fill_between(df['time_sec'],
                        df['pose.pose.position.x'] - df['pos_x_std'],
                        df['pose.pose.position.x'] + df['pos_x_std'],
                        alpha=0.2)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Linear velocity components with uncertainty
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(df['time_sec'], df['twist.twist.linear.x'], label='Vx', alpha=0.7)
    ax3.plot(df['time_sec'], df['twist.twist.linear.y'], label='Vy', alpha=0.7)
    ax3.plot(df['time_sec'], df['twist.twist.linear.z'], label='Vz', alpha=0.7)
    
    if has_twist_cov:
        ax3.fill_between(df['time_sec'],
                        df['twist.twist.linear.x'] - df['vel_x_std'],
                        df['twist.twist.linear.x'] + df['vel_x_std'],
                        alpha=0.2)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Linear Velocity (m/s)')
    ax3.set_title('Linear Velocity Components')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. Angular velocity components
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(df['time_sec'], df['twist.twist.angular.x'], label='ωx', alpha=0.7)
    ax4.plot(df['time_sec'], df['twist.twist.angular.y'], label='ωy', alpha=0.7)
    ax4.plot(df['time_sec'], df['twist.twist.angular.z'], label='ωz', alpha=0.7)
    
    if has_twist_cov:
        ax4.fill_between(df['time_sec'],
                        df['twist.twist.angular.z'] - df['ang_z_std'],
                        df['twist.twist.angular.z'] + df['ang_z_std'],
                        alpha=0.2)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angular Velocity (rad/s)')
    ax4.set_title('Angular Velocity Components')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. Horizontal position uncertainty
    ax5 = fig.add_subplot(gs[1, 2])
    if has_pose_cov:
        ax5.plot(df['time_sec'], df['horizontal_pos_uncertainty'], alpha=0.7, color='red')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Uncertainty (m)')
        ax5.set_title('Horizontal Position Uncertainty')
        ax5.grid(True, alpha=0.3)
    else:
        ax5.text(0.5, 0.5, 'No pose covariance data', 
                ha='center', va='center', transform=ax5.transAxes)
        ax5.set_title('Horizontal Position Uncertainty')
    
    # 6. Yaw angle with uncertainty
    ax6 = fig.add_subplot(gs[2, 0])
    ax6.plot(df['time_sec'], df['yaw_deg'], alpha=0.7, label='Yaw')
    
    if has_pose_cov:
        ax6.fill_between(df['time_sec'],
                        df['yaw_deg'] - np.degrees(df['yaw_std']),
                        df['yaw_deg'] + np.degrees(df['yaw_std']),
                        alpha=0.3, label='±1σ')
    
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Yaw Angle (degrees)')
    ax6.set_title('Yaw Angle vs Time')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # 7. Altitude with uncertainty
    ax7 = fig.add_subplot(gs[2, 1])
    ax7.plot(df['time_sec'], df['pose.pose.position.z'], alpha=0.7, color='purple', label='Altitude')
    
    if has_pose_cov:
        ax7.fill_between(df['time_sec'],
                        df['pose.pose.position.z'] - df['pos_z_std'],
                        df['pose.pose.position.z'] + df['pos_z_std'],
                        alpha=0.3, color='purple', label='±1σ')
    
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Altitude (m)')
    ax7.set_title('Altitude vs Time')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # 8. Velocity uncertainty
    ax8 = fig.add_subplot(gs[2, 2])
    if has_twist_cov:
        ax8.plot(df['time_sec'], df['horizontal_vel_uncertainty'], alpha=0.7, color='green')
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('Uncertainty (m/s)')
        ax8.set_title('Horizontal Velocity Uncertainty')
        ax8.grid(True, alpha=0.3)
    else:
        ax8.text(0.5, 0.5, 'No twist covariance data', 
                ha='center', va='center', transform=ax8.transAxes)
        ax8.set_title('Horizontal Velocity Uncertainty')
    
    plt.suptitle('Odometry Analysis', fontsize=16, fontweight='bold')
    
    # Save plot
    if output_dir is None:
        output_dir = os.path.dirname(csv_path)
    
    output_file = os.path.join(output_dir, 'odometry_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Create detailed uncertainty analysis plot
    if has_pose_cov or has_twist_cov:
        fig2, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Position uncertainty components
        if has_pose_cov:
            axes[0, 0].plot(df['time_sec'], df['pos_x_std'], label='X', alpha=0.7)
            axes[0, 0].plot(df['time_sec'], df['pos_y_std'], label='Y', alpha=0.7)
            axes[0, 0].plot(df['time_sec'], df['pos_z_std'], label='Z', alpha=0.7)
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Position Uncertainty (m)')
            axes[0, 0].set_title('Position Uncertainty Components (1σ)')
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)
        else:
            axes[0, 0].text(0.5, 0.5, 'No pose covariance data', 
                           ha='center', va='center', transform=axes[0, 0].transAxes)
        
        # Velocity uncertainty components
        if has_twist_cov:
            axes[0, 1].plot(df['time_sec'], df['vel_x_std'], label='Vx', alpha=0.7)
            axes[0, 1].plot(df['time_sec'], df['vel_y_std'], label='Vy', alpha=0.7)
            axes[0, 1].plot(df['time_sec'], df['vel_z_std'], label='Vz', alpha=0.7)
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Velocity Uncertainty (m/s)')
            axes[0, 1].set_title('Velocity Uncertainty Components (1σ)')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)
        else:
            axes[0, 1].text(0.5, 0.5, 'No twist covariance data', 
                           ha='center', va='center', transform=axes[0, 1].transAxes)
        
        # Horizontal uncertainty distribution
        if has_pose_cov:
            axes[1, 0].hist(df['horizontal_pos_uncertainty'], bins=50, alpha=0.7, 
                           color='blue', edgecolor='black')
            axes[1, 0].axvline(df['horizontal_pos_uncertainty'].mean(), color='red', 
                              linestyle='--', label=f'Mean: {df["horizontal_pos_uncertainty"].mean():.3f} m')
            axes[1, 0].set_xlabel('Horizontal Position Uncertainty (m)')
            axes[1, 0].set_ylabel('Count')
            axes[1, 0].set_title('Horizontal Position Uncertainty Distribution')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
        else:
            axes[1, 0].text(0.5, 0.5, 'No pose covariance data', 
                           ha='center', va='center', transform=axes[1, 0].transAxes)
        
        # Yaw uncertainty over time
        if has_pose_cov:
            axes[1, 1].plot(df['time_sec'], np.degrees(df['yaw_std']), alpha=0.7, color='orange')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Yaw Uncertainty (degrees)')
            axes[1, 1].set_title('Yaw Angle Uncertainty (1σ)')
            axes[1, 1].grid(True, alpha=0.3)
        else:
            axes[1, 1].text(0.5, 0.5, 'No pose covariance data', 
                           ha='center', va='center', transform=axes[1, 1].transAxes)
        
        plt.tight_layout()
        uncertainty_file = os.path.join(output_dir, 'odometry_uncertainty_analysis.png')
        plt.savefig(uncertainty_file, dpi=150, bbox_inches='tight')
        print(f"Uncertainty analysis saved to: {uncertainty_file}")
    
    # Save statistics to text file
    stats_file = os.path.join(output_dir, 'odometry_statistics.txt')
    with open(stats_file, 'w') as f:
        f.write("ODOMETRY STATISTICS\n")
        f.write("="*60 + "\n")
        f.write(f"Total messages: {len(df)}\n")
        f.write(f"Duration: {df['time_sec'].iloc[-1]:.2f} seconds\n")
        f.write(f"Average rate: {len(df) / df['time_sec'].iloc[-1]:.2f} Hz\n")
        f.write(f"\nPosition range:\n")
        f.write(f"  X: [{df['pose.pose.position.x'].min():.2f}, {df['pose.pose.position.x'].max():.2f}] m\n")
        f.write(f"  Y: [{df['pose.pose.position.y'].min():.2f}, {df['pose.pose.position.y'].max():.2f}] m\n")
        f.write(f"  Z: [{df['pose.pose.position.z'].min():.2f}, {df['pose.pose.position.z'].max():.2f}] m\n")
        f.write(f"\nTotal distance traveled: {total_distance:.2f} m\n")
        f.write(f"\nLinear velocity:\n")
        f.write(f"  Mean: {df['linear_velocity'].mean():.3f} m/s\n")
        f.write(f"  Max: {df['linear_velocity'].max():.3f} m/s\n")
        f.write(f"\nAngular velocity:\n")
        f.write(f"  Mean: {df['angular_velocity'].mean():.3f} rad/s\n")
        f.write(f"  Max: {df['angular_velocity'].max():.3f} rad/s\n")
        f.write(f"\nYaw angle range: [{df['yaw_deg'].min():.1f}°, {df['yaw_deg'].max():.1f}°]\n")
        
        if has_pose_cov:
            f.write(f"\nPosition Uncertainty (1σ):\n")
            f.write(f"  Horizontal: {df['horizontal_pos_uncertainty'].mean():.3f} m (mean), {df['horizontal_pos_uncertainty'].max():.3f} m (max)\n")
            f.write(f"  Vertical: {df['pos_z_std'].mean():.3f} m (mean), {df['pos_z_std'].max():.3f} m (max)\n")
            f.write(f"  Yaw: {np.degrees(df['yaw_std'].mean()):.2f}° (mean), {np.degrees(df['yaw_std'].max()):.2f}° (max)\n")
        
        if has_twist_cov:
            f.write(f"\nVelocity Uncertainty (1σ):\n")
            f.write(f"  Horizontal: {df['horizontal_vel_uncertainty'].mean():.4f} m/s (mean)\n")
            f.write(f"  Angular (yaw rate): {df['ang_z_std'].mean():.4f} rad/s (mean)\n")
    
    print(f"Statistics saved to: {stats_file}")
    
    plt.show()

def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description='Plot and analyze odometry data from CSV file.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'csv_path',
        help='Path to the odometry CSV file'
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='Output directory for plots (default: same as CSV file)'
    )
    
    args = parser.parse_args()
    
    # Validate CSV path exists
    if not os.path.exists(args.csv_path):
        print(f"Error: CSV file '{args.csv_path}' does not exist")
        return 1
    
    try:
        analyze_odometry(args.csv_path, args.output)
        return 0
    except Exception as e:
        print(f"Error analyzing odometry: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())