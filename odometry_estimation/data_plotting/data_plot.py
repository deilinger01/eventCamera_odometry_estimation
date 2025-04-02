import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv(...) # TODO: add path to the csv file

df['timestamp'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1e9


plt.figure(figsize=(10, 6))
if '_OF_odom_twist_twist_linear_x' in df.columns and not df['_OF_odom_twist_twist_linear_x'].isna().all() or '_ekf_odometry_filtered_twist_twist_linear_x' in df.columns and not df['_ekf_odometry_filtered_twist_twist_linear_x'].isna().all():
    if '_ekf_odometry_filtered_twist_twist_linear_x' in df.columns and not df['_ekf_odometry_filtered_twist_twist_linear_x'].isna().all():
        plt.plot(df['timestamp'], df['_ekf_odometry_filtered_twist_twist_linear_x'], 
                color='green', label='Wheel Encoder')

    if '_OF_odom_twist_twist_linear_x' in df.columns and not df['_OF_odom_twist_twist_linear_x'].isna().all():
        plt.plot(df['timestamp'], df['_OF_odom_twist_twist_linear_x'], 
                color='blue', label='eOF')
    
    plt.xlabel('Timestamp in seconds')
    plt.ylabel(r'velocity in $\frac{m}{s}$')
    plt.title('Longitudinal Velocity')
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    print("No valid Longitudinal velocity data found in _OF_odom_twist_twist_linear_x and _ekf_odometry_filtered_twist_twist_linear_x column")

if '_OF_odom_twist_twist_linear_y' in df.columns and not df['_OF_odom_twist_twist_linear_y'].isna().all():
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp'], df['_OF_odom_twist_twist_linear_y'], 
             color='red', label='eOF Lateral Velocity')
    
    plt.xlabel('Timestamp in seconds')
    plt.ylabel(r'velocity in $\frac{m}{s}$')
    plt.title('Lateral Velocity')
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    print("No valid lateral velocity data found in _OF_odom_twist_twist_linear_y column")