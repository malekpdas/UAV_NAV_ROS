import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
import os

def parse_covariance(cov_str):
    """Parse covariance string and extract diagonal elements."""
    try:
        values = list(map(float, str(cov_str).split()))
        # Extract diagonal elements (0, 7, 14, 21, 28, 35 for 6x6 matrix)
        if len(values) >= 36:
            return {
                'vx_var': values[0],
                'vy_var': values[7],
                'vz_var': values[14],
                'wx_var': values[21],
                'wy_var': values[28],
                'wz_var': values[35]
            }
    except:
        pass
    return None

def analyze_gps_velocity(csv_path, output_dir=None):
    """Analyze and plot GPS velocity data from CSV file."""
    
    # Read CSV
    print(f"Reading {csv_path}...")
    df = pd.read_csv(csv_path)
    
    # Convert timestamp to seconds (relative to start)
    df['time_sec'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9
    
    # Calculate velocity magnitudes
    df['linear_velocity'] = np.sqrt(
        df['twist.twist.linear.x']**2 + 
        df['twist.twist.linear.y']**2 + 
        df['twist.twist.linear.z']**2
    )
    df['horizontal_velocity'] = np.sqrt(
        df['twist.twist.linear.x']**2 + 
        df['twist.twist.linear.y']**2
    )
    df['angular_velocity'] = np.sqrt(
        df['twist.twist.angular.x']**2 + 
        df['twist.twist.angular.y']**2 + 
        df['twist.twist.angular.z']**2
    )
    
    # Convert to km/h for readability
    df['linear_velocity_kmh'] = df['linear_velocity'] * 3.6
    df['horizontal_velocity_kmh'] = df['horizontal_velocity'] * 3.6
    
    # Calculate heading from velocity vector
    df['heading_deg'] = np.degrees(
        np.arctan2(df['twist.twist.linear.y'], df['twist.twist.linear.x'])
    )
    df['heading_deg'] = (df['heading_deg'] + 360) % 360  # Normalize to 0-360
    
    # Calculate acceleration (change in velocity)
    df['accel_x'] = df['twist.twist.linear.x'].diff() / df['time_sec'].diff()
    df['accel_y'] = df['twist.twist.linear.y'].diff() / df['time_sec'].diff()
    df['accel_z'] = df['twist.twist.linear.z'].diff() / df['time_sec'].diff()
    df['accel_magnitude'] = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
    
    # Parse covariance for uncertainty estimates
    cov_data = df['twist.covariance'].apply(parse_covariance)
    if cov_data.iloc[0] is not None:
        df['vx_std'] = cov_data.apply(lambda x: np.sqrt(x['vx_var']) if x else np.nan)
        df['vy_std'] = cov_data.apply(lambda x: np.sqrt(x['vy_var']) if x else np.nan)
        df['vz_std'] = cov_data.apply(lambda x: np.sqrt(x['vz_var']) if x else np.nan)
    
    # Calculate distance traveled (integrate velocity)
    dt = df['time_sec'].diff().fillna(0)
    df['distance_increment'] = df['horizontal_velocity'] * dt
    df['cumulative_distance'] = df['distance_increment'].cumsum()
    
    # Print statistics
    print("\n" + "="*60)
    print("GPS VELOCITY ANALYSIS")
    print("="*60)
    print(f"Total messages: {len(df)}")
    print(f"Duration: {df['time_sec'].iloc[-1]:.2f} seconds")
    print(f"Average rate: {len(df) / df['time_sec'].iloc[-1]:.2f} Hz")
    
    print(f"\nLinear Velocity:")
    print(f"  Mean: {df['linear_velocity'].mean():.3f} m/s ({df['linear_velocity_kmh'].mean():.2f} km/h)")
    print(f"  Max: {df['linear_velocity'].max():.3f} m/s ({df['linear_velocity_kmh'].max():.2f} km/h)")
    print(f"  Min: {df['linear_velocity'].min():.3f} m/s ({df['linear_velocity_kmh'].min():.2f} km/h)")
    
    print(f"\nHorizontal Velocity:")
    print(f"  Mean: {df['horizontal_velocity'].mean():.3f} m/s ({df['horizontal_velocity_kmh'].mean():.2f} km/h)")
    print(f"  Max: {df['horizontal_velocity'].max():.3f} m/s ({df['horizontal_velocity_kmh'].max():.2f} km/h)")
    
    print(f"\nVertical Velocity (Vz):")
    print(f"  Mean: {df['twist.twist.linear.z'].mean():.3f} m/s")
    print(f"  Max: {df['twist.twist.linear.z'].max():.3f} m/s")
    print(f"  Min: {df['twist.twist.linear.z'].min():.3f} m/s")
    
    print(f"\nAngular Velocity:")
    print(f"  Mean: {df['angular_velocity'].mean():.4f} rad/s")
    print(f"  Max: {df['angular_velocity'].max():.4f} rad/s")
    
    if 'accel_magnitude' in df.columns:
        print(f"\nAcceleration:")
        print(f"  Mean: {df['accel_magnitude'].mean():.3f} m/s²")
        print(f"  Max: {df['accel_magnitude'].max():.3f} m/s²")
    
    total_distance = df['cumulative_distance'].iloc[-1]
    print(f"\nEstimated distance traveled: {total_distance:.2f} m ({total_distance/1000:.3f} km)")
    
    if 'vx_std' in df.columns and not df['vx_std'].isna().all():
        print(f"\nVelocity Uncertainty (std dev):")
        print(f"  Vx: {df['vx_std'].mean():.4f} m/s")
        print(f"  Vy: {df['vy_std'].mean():.4f} m/s")
        print(f"  Vz: {df['vz_std'].mean():.4f} m/s")
    
    print("="*60 + "\n")
    
    # Create plots
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # 1. Velocity components over time
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(df['time_sec'], df['twist.twist.linear.x'], label='Vx (East)', alpha=0.7)
    ax1.plot(df['time_sec'], df['twist.twist.linear.y'], label='Vy (North)', alpha=0.7)
    ax1.plot(df['time_sec'], df['twist.twist.linear.z'], label='Vz (Up)', alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('Linear Velocity Components')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Speed (magnitude) over time
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(df['time_sec'], df['linear_velocity_kmh'], alpha=0.7, color='red')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed (km/h)')
    ax2.set_title('Total Speed')
    ax2.grid(True, alpha=0.3)
    
    # 3. Horizontal velocity over time
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(df['time_sec'], df['horizontal_velocity_kmh'], alpha=0.7, color='purple')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (km/h)')
    ax3.set_title('Horizontal Speed')
    ax3.grid(True, alpha=0.3)
    
    # 4. Vertical velocity over time
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(df['time_sec'], df['twist.twist.linear.z'], alpha=0.7, color='green')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Vertical Velocity (m/s)')
    ax4.set_title('Vertical Velocity (Vz)')
    ax4.grid(True, alpha=0.3)
    
    # 5. Velocity vector plot (2D)
    ax5 = fig.add_subplot(gs[1, 2])
    # Subsample for clarity
    N = max(1, len(df) // 50)
    df_sub = df.iloc[::N]
    ax5.quiver(df_sub['twist.twist.linear.x'], df_sub['twist.twist.linear.y'],
              df_sub['twist.twist.linear.x'], df_sub['twist.twist.linear.y'],
              df_sub['time_sec'], cmap='viridis', alpha=0.6)
    ax5.set_xlabel('Vx (m/s)')
    ax5.set_ylabel('Vy (m/s)')
    ax5.set_title('Velocity Vectors (2D)')
    ax5.grid(True, alpha=0.3)
    ax5.axis('equal')
    
    # 6. Angular velocity components
    ax6 = fig.add_subplot(gs[2, 0])
    ax6.plot(df['time_sec'], df['twist.twist.angular.x'], label='ωx (Roll)', alpha=0.7)
    ax6.plot(df['time_sec'], df['twist.twist.angular.y'], label='ωy (Pitch)', alpha=0.7)
    ax6.plot(df['time_sec'], df['twist.twist.angular.z'], label='ωz (Yaw)', alpha=0.7)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Angular Velocity (rad/s)')
    ax6.set_title('Angular Velocity Components')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # 7. Heading from velocity
    ax7 = fig.add_subplot(gs[2, 1])
    ax7.plot(df['time_sec'], df['heading_deg'], alpha=0.7, color='orange')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Heading (degrees)')
    ax7.set_title('Heading from Velocity Vector')
    ax7.set_ylim(0, 360)
    ax7.grid(True, alpha=0.3)
    
    # 8. Acceleration over time
    ax8 = fig.add_subplot(gs[2, 2])
    ax8.plot(df['time_sec'], df['accel_magnitude'], alpha=0.7, color='brown')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Acceleration (m/s²)')
    ax8.set_title('Acceleration Magnitude')
    ax8.grid(True, alpha=0.3)
    
    plt.suptitle('GPS Velocity Analysis', fontsize=16, fontweight='bold')
    
    # Save plot
    if output_dir is None:
        output_dir = os.path.dirname(csv_path)
    
    output_file = os.path.join(output_dir, 'gps_velocity_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Create additional plot for velocity uncertainty (if available)
    if 'vx_std' in df.columns and not df['vx_std'].isna().all():
        fig2, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Vx with uncertainty
        axes[0].plot(df['time_sec'], df['twist.twist.linear.x'], 'b-', label='Vx', alpha=0.7)
        axes[0].fill_between(df['time_sec'],
                            df['twist.twist.linear.x'] - df['vx_std'],
                            df['twist.twist.linear.x'] + df['vx_std'],
                            alpha=0.3, label='±1σ')
        axes[0].set_ylabel('Vx (m/s)')
        axes[0].set_title('Velocity with Uncertainty Bounds')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        # Vy with uncertainty
        axes[1].plot(df['time_sec'], df['twist.twist.linear.y'], 'g-', label='Vy', alpha=0.7)
        axes[1].fill_between(df['time_sec'],
                            df['twist.twist.linear.y'] - df['vy_std'],
                            df['twist.twist.linear.y'] + df['vy_std'],
                            alpha=0.3, label='±1σ')
        axes[1].set_ylabel('Vy (m/s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        # Vz with uncertainty
        axes[2].plot(df['time_sec'], df['twist.twist.linear.z'], 'r-', label='Vz', alpha=0.7)
        axes[2].fill_between(df['time_sec'],
                            df['twist.twist.linear.z'] - df['vz_std'],
                            df['twist.twist.linear.z'] + df['vz_std'],
                            alpha=0.3, label='±1σ')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Vz (m/s)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        uncertainty_file = os.path.join(output_dir, 'gps_velocity_uncertainty.png')
        plt.savefig(uncertainty_file, dpi=150, bbox_inches='tight')
        print(f"Uncertainty plot saved to: {uncertainty_file}")
    
    # Save statistics to text file
    stats_file = os.path.join(output_dir, 'gps_velocity_statistics.txt')
    with open(stats_file, 'w') as f:
        f.write("GPS VELOCITY STATISTICS\n")
        f.write("="*60 + "\n")
        f.write(f"Total messages: {len(df)}\n")
        f.write(f"Duration: {df['time_sec'].iloc[-1]:.2f} seconds\n")
        f.write(f"Average rate: {len(df) / df['time_sec'].iloc[-1]:.2f} Hz\n")
        
        f.write(f"\nLinear Velocity:\n")
        f.write(f"  Mean: {df['linear_velocity'].mean():.3f} m/s ({df['linear_velocity_kmh'].mean():.2f} km/h)\n")
        f.write(f"  Max: {df['linear_velocity'].max():.3f} m/s ({df['linear_velocity_kmh'].max():.2f} km/h)\n")
        f.write(f"  Min: {df['linear_velocity'].min():.3f} m/s ({df['linear_velocity_kmh'].min():.2f} km/h)\n")
        
        f.write(f"\nHorizontal Velocity:\n")
        f.write(f"  Mean: {df['horizontal_velocity'].mean():.3f} m/s ({df['horizontal_velocity_kmh'].mean():.2f} km/h)\n")
        f.write(f"  Max: {df['horizontal_velocity'].max():.3f} m/s ({df['horizontal_velocity_kmh'].max():.2f} km/h)\n")
        
        f.write(f"\nVertical Velocity (Vz):\n")
        f.write(f"  Mean: {df['twist.twist.linear.z'].mean():.3f} m/s\n")
        f.write(f"  Max: {df['twist.twist.linear.z'].max():.3f} m/s\n")
        f.write(f"  Min: {df['twist.twist.linear.z'].min():.3f} m/s\n")
        
        f.write(f"\nAngular Velocity:\n")
        f.write(f"  Mean: {df['angular_velocity'].mean():.4f} rad/s\n")
        f.write(f"  Max: {df['angular_velocity'].max():.4f} rad/s\n")
        
        if 'accel_magnitude' in df.columns:
            f.write(f"\nAcceleration:\n")
            f.write(f"  Mean: {df['accel_magnitude'].mean():.3f} m/s²\n")
            f.write(f"  Max: {df['accel_magnitude'].max():.3f} m/s²\n")
        
        f.write(f"\nEstimated distance traveled: {total_distance:.2f} m ({total_distance/1000:.3f} km)\n")
        
        if 'vx_std' in df.columns and not df['vx_std'].isna().all():
            f.write(f"\nVelocity Uncertainty (std dev):\n")
            f.write(f"  Vx: {df['vx_std'].mean():.4f} m/s\n")
            f.write(f"  Vy: {df['vy_std'].mean():.4f} m/s\n")
            f.write(f"  Vz: {df['vz_std'].mean():.4f} m/s\n")
    
    print(f"Statistics saved to: {stats_file}")
    
    plt.show()

def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description='Plot and analyze GPS velocity data from CSV file.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'csv_path',
        help='Path to the GPS velocity CSV file'
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
        analyze_gps_velocity(args.csv_path, args.output)
        return 0
    except Exception as e:
        print(f"Error analyzing GPS velocity: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())