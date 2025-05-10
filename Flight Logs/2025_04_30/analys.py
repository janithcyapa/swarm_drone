import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
from pymavlink import mavutil

def load_flight_log(log_file):
    """Load flight log using pymavlink"""
    mlog = mavutil.mavlink_connection(log_file)
    data = mlog.recv_match(type=['ATT', 'GPS', 'CTUN', 'MODE'], blocking=False)
    
    # Convert to pandas DataFrame
    msgs = []
    while data is not None:
        msgs.append(data.to_dict())
        data = mlog.recv_match(type=['ATT', 'GPS', 'CTUN', 'MODE'], blocking=False)
    
    df = pd.DataFrame(msgs)
    return df

def plot_attitude_with_modes(df):
    """Plot yaw, pitch, roll with flight modes highlighted"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    # Convert timestamps to seconds from start
    df['time_rel'] = (df['TimeUS'] - df['TimeUS'].min()) / 1e6
    
    # Plot attitude data
    ax1.plot(df['time_rel'], df['Roll'], label='Roll', color='blue')
    ax1.set_ylabel('Roll (deg)')
    ax1.grid(True)
    
    ax2.plot(df['time_rel'], df['Pitch'], label='Pitch', color='green')
    ax2.set_ylabel('Pitch (deg)')
    ax2.grid(True)
    
    ax3.plot(df['time_rel'], df['Yaw'], label='Yaw', color='red')
    ax3.set_ylabel('Yaw (deg)')
    ax3.set_xlabel('Time (seconds)')
    ax3.grid(True)
    
    # Highlight flight modes if available
    if 'Mode' in df.columns:
        mode_changes = df[df['Mode'].notna() & (df['Mode'].shift() != df['Mode'])]
        
        for idx, row in mode_changes.iterrows():
            for ax in [ax1, ax2, ax3]:
                ax.axvline(x=row['time_rel'], color='gray', linestyle='--', alpha=0.7)
    
    plt.suptitle('Attitude Data with Flight Mode Changes')
    plt.tight_layout()
    return fig

def plot_altitude(df):
    """Plot altitude over time"""
    fig, ax = plt.subplots(figsize=(12, 4))
    
    if 'Alt' in df.columns:  # GPS altitude
        ax.plot(df['time_rel'], df['Alt'], label='GPS Altitude', color='purple')
    elif 'BarAlt' in df.columns:  # Barometric altitude
        ax.plot(df['time_rel'], df['BarAlt'], label='Barometric Altitude', color='purple')
    
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude Over Time')
    ax.grid(True)
    plt.tight_layout()
    return fig

def plot_position_2d(df):
    """Plot 2D position (latitude/longitude)"""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    if 'Lat' in df.columns and 'Lng' in df.columns:
        # Convert to relative coordinates
        lat_ref = df['Lat'].mean()
        lng_ref = df['Lng'].mean()
        
        # Simple scaling - for better results use proper geodesic calculations
        x = (df['Lng'] - lng_ref) * 111320 * np.cos(np.radians(lat_ref))
        y = (df['Lat'] - lat_ref) * 111320
        
        ax.plot(x, y, color='blue')
        
        # Add start and end markers
        ax.plot(x.iloc[0], y.iloc[0], 'go', markersize=10, label='Start')
        ax.plot(x.iloc[-1], y.iloc[-1], 'ro', markersize=10, label='End')
        
        ax.set_xlabel('East-West Position (m)')
        ax.set_ylabel('North-South Position (m)')
        ax.set_title('2D Flight Path')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
    
    plt.tight_layout()
    return fig

def analyze_flight_log(log_file):
    """Main function to analyze and plot flight log"""
    df = load_flight_log(log_file)
    
    # Create plots
    fig_att = plot_attitude_with_modes(df)
    fig_alt = plot_altitude(df)
    fig_pos = plot_position_2d(df)
    
    plt.show()
    
    return fig_att, fig_alt, fig_pos

if __name__ == "__main__":
    import sys
    import numpy as np
    
    if len(sys.argv) < 2:
        print("Usage: python flight_log_analyzer.py <path_to_log_file.bin>")
        sys.exit(1)
    
    log_file = sys.argv[1]
    analyze_flight_log(log_file)