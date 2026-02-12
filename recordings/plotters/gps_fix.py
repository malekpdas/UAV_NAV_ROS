import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
import os

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters."""
    R = 6371000  # Earth radius in meters
    
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    distance = R * c
    
    return distance

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing between two GPS coordinates in degrees."""
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    bearing = np.arctan2(x, y)
    bearing = np.degrees(bearing)
    bearing = (bearing + 360) % 360
    
    return bearing

def parse_covariance(cov_str):
    """Parse covariance string and extract useful uncertainty metrics."""
    try:
        values = list(map(float, str(cov_str).split()))
        if len(values) >= 9:
            # 3x3 position covariance matrix
            # Diagonal elements: [0]=x², [4]=y², [8]=z²
            return {
                'pos_x_var': values[0],
                'pos_y_var': values[4],
                'pos_z_var': values[8],
                'pos_xy_cov': values[1] if len(values) > 1 else 0,
            }
    except:
        pass
    return None

def status_to_string(status):
    """Convert GPS status code to string."""
    status_map = {
        -2: 'UNKNOWN',
        -1: 'NO_FIX',
        0: 'FIX',
        1: 'SBAS_FIX',
        2: 'GBAS_FIX'
    }
    return status_map.get(status, f'UNKNOWN({status})')

def service_to_string(service):
    """Convert GPS service code to string."""
    services = []
    if service & 1:
        services.append('GPS')
    if service & 2:
        services.append('GLONASS')
    if service & 4:
        services.append('COMPASS')
    if service & 8:
        services.append('GALILEO')
    return '+'.join(services) if services else 'UNKNOWN'

def analyze_gps_fix(csv_path, output_dir=None):
    """Analyze and plot GPS fix data from CSV file."""
    
    # Read CSV
    print(f"Reading {csv_path}...")
    df = pd.read_csv(csv_path)
    
    # Convert timestamp to seconds (relative to start)
    df['time_sec'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9
    
    # Parse covariance data
    cov_data = df['position_covariance'].apply(parse_covariance)
    has_covariance = cov_data.iloc[0] is not None
    
    if has_covariance:
        df['pos_x_std'] = cov_data.apply(lambda x: np.sqrt(x['pos_x_var']) if x else np.nan)
        df['pos_y_std'] = cov_data.apply(lambda x: np.sqrt(x['pos_y_var']) if x else np.nan)
        df['pos_z_std'] = cov_data.apply(lambda x: np.sqrt(x['pos_z_var']) if x else np.nan)
        df['horizontal_uncertainty'] = np.sqrt(df['pos_x_std']**2 + df['pos_y_std']**2)
        
        # Calculate HDOP-like metric (normalized uncertainty)
        df['hdop_estimate'] = df['horizontal_uncertainty'] / df['horizontal_uncertainty'].median()
    
    # Calculate distances between consecutive points
    distances = []
    bearings = []
    speeds = []
    
    for i in range(1, len(df)):
        dist = haversine_distance(
            df['latitude'].iloc[i-1], df['longitude'].iloc[i-1],
            df['latitude'].iloc[i], df['longitude'].iloc[i]
        )
        distances.append(dist)
        
        bearing = calculate_bearing(
            df['latitude'].iloc[i-1], df['longitude'].iloc[i-1],
            df['latitude'].iloc[i], df['longitude'].iloc[i]
        )
        bearings.append(bearing)
        
        dt = df['time_sec'].iloc[i] - df['time_sec'].iloc[i-1]
        speed = dist / dt if dt > 0 else 0
        speeds.append(speed)
    
    # Add first point values (zero)
    distances.insert(0, 0)
    bearings.insert(0, 0)
    speeds.insert(0, 0)
    
    df['distance'] = distances
    df['bearing'] = bearings
    df['speed_mps'] = speeds
    df['speed_kmh'] = np.array(speeds) * 3.6
    
    # Calculate cumulative distance
    df['cumulative_distance'] = np.cumsum(distances)
    
    # Calculate position jitter (measure of noise)
    if len(df) > 1:
        df['lat_diff'] = df['latitude'].diff().abs()
        df['lon_diff'] = df['longitude'].diff().abs()
        df['alt_diff'] = df['altitude'].diff().abs()
    
    # Print statistics
    print("\n" + "="*60)
    print("GPS FIX ANALYSIS")
    print("="*60)
    print(f"Total messages: {len(df)}")
    print(f"Duration: {df['time_sec'].iloc[-1]:.2f} seconds")
    print(f"Average rate: {len(df) / df['time_sec'].iloc[-1]:.2f} Hz")
    
    print(f"\nGPS Status:")
    status_counts = df['status.status'].value_counts()
    for status, count in status_counts.items():
        print(f"  {status_to_string(status)}: {count} ({100*count/len(df):.1f}%)")
    
    print(f"\nGPS Service:")
    service_counts = df['status.service'].value_counts()
    for service, count in service_counts.items():
        print(f"  {service_to_string(service)}: {count} ({100*count/len(df):.1f}%)")
    
    print(f"\nPosition:")
    print(f"  Latitude: [{df['latitude'].min():.6f}, {df['latitude'].max():.6f}]°")
    print(f"  Longitude: [{df['longitude'].min():.6f}, {df['longitude'].max():.6f}]°")
    print(f"  Altitude: [{df['altitude'].min():.2f}, {df['altitude'].max():.2f}] m")
    
    total_distance = df['cumulative_distance'].iloc[-1]
    print(f"\nTotal distance traveled: {total_distance:.2f} m ({total_distance/1000:.3f} km)")
    
    print(f"\nSpeed:")
    print(f"  Mean: {df['speed_kmh'].mean():.2f} km/h ({df['speed_mps'].mean():.2f} m/s)")
    print(f"  Max: {df['speed_kmh'].max():.2f} km/h ({df['speed_mps'].max():.2f} m/s)")
    
    if has_covariance:
        print(f"\nPosition Uncertainty (1σ):")
        print(f"  Horizontal: {df['horizontal_uncertainty'].mean():.3f} m (mean), {df['horizontal_uncertainty'].max():.3f} m (max)")
        print(f"  Vertical: {df['pos_z_std'].mean():.3f} m (mean), {df['pos_z_std'].max():.3f} m (max)")
        print(f"  CEP (50%): ~{df['horizontal_uncertainty'].mean() * 0.59:.3f} m")
        print(f"  95% confidence: ~{df['horizontal_uncertainty'].mean() * 2.45:.3f} m")
    
    if 'lat_diff' in df.columns:
        print(f"\nPosition Jitter (between samples):")
        print(f"  Latitude: {df['lat_diff'].mean()*111000:.3f} m (mean)")
        print(f"  Longitude: {df['lon_diff'].mean()*111000*np.cos(np.radians(df['latitude'].mean())):.3f} m (mean)")
        print(f"  Altitude: {df['alt_diff'].mean():.3f} m (mean)")
    
    print("="*60 + "\n")
    
    # Create main plots
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # 1. GPS Track (Lat/Lon) with uncertainty ellipses
    ax1 = fig.add_subplot(gs[0, :2])
    scatter = ax1.scatter(df['longitude'], 
                         df['latitude'],
                         c=df['time_sec'], 
                         cmap='viridis', 
                         s=20, 
                         alpha=0.6)
    
    # Add uncertainty ellipses (subsample for clarity)
    if has_covariance:
        N = max(1, len(df) // 20)
        df_sub = df.iloc[::N]
        for _, row in df_sub.iterrows():
            # Convert std dev to degrees (approximate)
            lat_std_deg = row['pos_y_std'] / 111000
            lon_std_deg = row['pos_x_std'] / (111000 * np.cos(np.radians(row['latitude'])))
            
            ellipse = plt.Circle((row['longitude'], row['latitude']),
                                radius=max(lat_std_deg, lon_std_deg),
                                color='red', fill=False, alpha=0.3, linewidth=1)
            ax1.add_patch(ellipse)
    
    ax1.plot(df['longitude'].iloc[0], 
             df['latitude'].iloc[0], 
             'go', markersize=12, label='Start', zorder=5)
    ax1.plot(df['longitude'].iloc[-1], 
             df['latitude'].iloc[-1], 
             'ro', markersize=12, label='End', zorder=5)
    ax1.set_xlabel('Longitude (°)')
    ax1.set_ylabel('Latitude (°)')
    ax1.set_title('GPS Track with Uncertainty')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Time (s)')
    
    # 2. Altitude over time with uncertainty
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(df['time_sec'], df['altitude'], alpha=0.7, color='purple', label='Altitude')
    if has_covariance:
        ax2.fill_between(df['time_sec'],
                        df['altitude'] - df['pos_z_std'],
                        df['altitude'] + df['pos_z_std'],
                        alpha=0.3, color='purple', label='±1σ')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Horizontal uncertainty over time
    ax3 = fig.add_subplot(gs[1, 0])
    if has_covariance:
        ax3.plot(df['time_sec'], df['horizontal_uncertainty'], alpha=0.7, color='red')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Horizontal Uncertainty (m)')
        ax3.set_title('Horizontal Position Uncertainty')
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, 'No covariance data available', 
                ha='center', va='center', transform=ax3.transAxes)
        ax3.set_title('Horizontal Position Uncertainty')
    
    # 4. Vertical uncertainty over time
    ax4 = fig.add_subplot(gs[1, 1])
    if has_covariance:
        ax4.plot(df['time_sec'], df['pos_z_std'], alpha=0.7, color='orange')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Vertical Uncertainty (m)')
        ax4.set_title('Vertical Position Uncertainty')
        ax4.grid(True, alpha=0.3)
    else:
        ax4.text(0.5, 0.5, 'No covariance data available', 
                ha='center', va='center', transform=ax4.transAxes)
        ax4.set_title('Vertical Position Uncertainty')
    
    # 5. Speed over time
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.plot(df['time_sec'], df['speed_kmh'], alpha=0.7, color='green')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Speed (km/h)')
    ax5.set_title('Speed vs Time')
    ax5.grid(True, alpha=0.3)
    
    # 6. Position jitter (lat/lon)
    ax6 = fig.add_subplot(gs[2, 0])
    if 'lat_diff' in df.columns:
        ax6.semilogy(df['time_sec'], df['lat_diff']*111000, alpha=0.5, label='Latitude', linewidth=0.5)
        ax6.semilogy(df['time_sec'], df['lon_diff']*111000*np.cos(np.radians(df['latitude'].mean())), 
                    alpha=0.5, label='Longitude', linewidth=0.5)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Position Change (m, log scale)')
        ax6.set_title('Position Jitter Between Samples')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
    
    # 7. Cumulative distance
    ax7 = fig.add_subplot(gs[2, 1])
    ax7.plot(df['time_sec'], df['cumulative_distance'], alpha=0.7, color='brown')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Distance (m)')
    ax7.set_title('Cumulative Distance')
    ax7.grid(True, alpha=0.3)
    
    # 8. GPS Status over time
    ax8 = fig.add_subplot(gs[2, 2])
    status_values = df['status.status'].values
    ax8.plot(df['time_sec'], status_values, 'o', alpha=0.5, markersize=4)
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Status Code')
    ax8.set_title('GPS Fix Status')
    ax8.set_yticks([-2, -1, 0, 1, 2])
    ax8.set_yticklabels(['UNKNOWN', 'NO_FIX', 'FIX', 'SBAS', 'GBAS'])
    ax8.grid(True, alpha=0.3)
    
    plt.suptitle('GPS Fix Analysis', fontsize=16, fontweight='bold')
    
    # Save plot
    if output_dir is None:
        output_dir = os.path.dirname(csv_path)
    
    output_file = os.path.join(output_dir, 'gps_fix_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Create detailed uncertainty analysis plot
    if has_covariance:
        fig2, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Uncertainty components
        axes[0, 0].plot(df['time_sec'], df['pos_x_std'], label='East (X)', alpha=0.7)
        axes[0, 0].plot(df['time_sec'], df['pos_y_std'], label='North (Y)', alpha=0.7)
        axes[0, 0].plot(df['time_sec'], df['pos_z_std'], label='Up (Z)', alpha=0.7)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position Uncertainty (m)')
        axes[0, 0].set_title('Position Uncertainty Components (1σ)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Uncertainty distribution
        axes[0, 1].hist(df['horizontal_uncertainty'], bins=50, alpha=0.7, color='blue', edgecolor='black')
        axes[0, 1].axvline(df['horizontal_uncertainty'].mean(), color='red', 
                          linestyle='--', label=f'Mean: {df["horizontal_uncertainty"].mean():.3f} m')
        axes[0, 1].axvline(df['horizontal_uncertainty'].median(), color='green', 
                          linestyle='--', label=f'Median: {df["horizontal_uncertainty"].median():.3f} m')
        axes[0, 1].set_xlabel('Horizontal Uncertainty (m)')
        axes[0, 1].set_ylabel('Count')
        axes[0, 1].set_title('Horizontal Uncertainty Distribution')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # Uncertainty vs GPS status
        axes[1, 0].scatter(df['status.status'], df['horizontal_uncertainty'], alpha=0.3)
        axes[1, 0].set_xlabel('GPS Status')
        axes[1, 0].set_ylabel('Horizontal Uncertainty (m)')
        axes[1, 0].set_title('Uncertainty vs GPS Status')
        axes[1, 0].set_xticks([-2, -1, 0, 1, 2])
        axes[1, 0].set_xticklabels(['UNK', 'NOFIX', 'FIX', 'SBAS', 'GBAS'], rotation=45)
        axes[1, 0].grid(True, alpha=0.3)
        
        # HDOP estimate
        if 'hdop_estimate' in df.columns:
            axes[1, 1].plot(df['time_sec'], df['hdop_estimate'], alpha=0.7, color='purple')
            axes[1, 1].axhline(1.0, color='green', linestyle='--', alpha=0.5, label='Reference')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Normalized Uncertainty')
            axes[1, 1].set_title('HDOP-like Estimate (normalized to median)')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        uncertainty_file = os.path.join(output_dir, 'gps_uncertainty_analysis.png')
        plt.savefig(uncertainty_file, dpi=150, bbox_inches='tight')
        print(f"Uncertainty analysis saved to: {uncertainty_file}")
    
    # Save statistics to text file
    stats_file = os.path.join(output_dir, 'gps_fix_statistics.txt')
    with open(stats_file, 'w') as f:
        f.write("GPS FIX STATISTICS\n")
        f.write("="*60 + "\n")
        f.write(f"Total messages: {len(df)}\n")
        f.write(f"Duration: {df['time_sec'].iloc[-1]:.2f} seconds\n")
        f.write(f"Average rate: {len(df) / df['time_sec'].iloc[-1]:.2f} Hz\n")
        
        f.write(f"\nGPS Status:\n")
        for status, count in status_counts.items():
            f.write(f"  {status_to_string(status)}: {count} ({100*count/len(df):.1f}%)\n")
        
        f.write(f"\nGPS Service:\n")
        for service, count in service_counts.items():
            f.write(f"  {service_to_string(service)}: {count} ({100*count/len(df):.1f}%)\n")
        
        f.write(f"\nPosition:\n")
        f.write(f"  Latitude: [{df['latitude'].min():.6f}, {df['latitude'].max():.6f}]°\n")
        f.write(f"  Longitude: [{df['longitude'].min():.6f}, {df['longitude'].max():.6f}]°\n")
        f.write(f"  Altitude: [{df['altitude'].min():.2f}, {df['altitude'].max():.2f}] m\n")
        
        f.write(f"\nTotal distance traveled: {total_distance:.2f} m ({total_distance/1000:.3f} km)\n")
        
        f.write(f"\nSpeed:\n")
        f.write(f"  Mean: {df['speed_kmh'].mean():.2f} km/h ({df['speed_mps'].mean():.2f} m/s)\n")
        f.write(f"  Max: {df['speed_kmh'].max():.2f} km/h ({df['speed_mps'].max():.2f} m/s)\n")
        
        if has_covariance:
            f.write(f"\nPosition Uncertainty (1σ):\n")
            f.write(f"  Horizontal: {df['horizontal_uncertainty'].mean():.3f} m (mean), {df['horizontal_uncertainty'].max():.3f} m (max)\n")
            f.write(f"  Vertical: {df['pos_z_std'].mean():.3f} m (mean), {df['pos_z_std'].max():.3f} m (max)\n")
            f.write(f"  CEP (50%): ~{df['horizontal_uncertainty'].mean() * 0.59:.3f} m\n")
            f.write(f"  95% confidence: ~{df['horizontal_uncertainty'].mean() * 2.45:.3f} m\n")
        
        if 'lat_diff' in df.columns:
            f.write(f"\nPosition Jitter (between samples):\n")
            f.write(f"  Latitude: {df['lat_diff'].mean()*111000:.3f} m (mean)\n")
            f.write(f"  Longitude: {df['lon_diff'].mean()*111000*np.cos(np.radians(df['latitude'].mean())):.3f} m (mean)\n")
            f.write(f"  Altitude: {df['alt_diff'].mean():.3f} m (mean)\n")
    
    print(f"Statistics saved to: {stats_file}")
    
    plt.show()

def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description='Plot and analyze GPS fix data from CSV file.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'csv_path',
        help='Path to the GPS fix CSV file'
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
        analyze_gps_fix(args.csv_path, args.output)
        return 0
    except Exception as e:
        print(f"Error analyzing GPS fix: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())