import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Configuration
SAVE_FIGURES = False  # Set to True to save figures instead of displaying them

# Load the data path from curr_datafile_name.txt
with open('curr_datafile_name.txt', 'r') as f:
    data_path = f.read().strip()

print("="*60)
print("Data Visualization Script")
print("="*60)
print(f"Loading data from: {data_path}")
print(f"Save mode: {'ON (figures will be saved)' if SAVE_FIGURES else 'OFF (figures will be displayed)'}")
print("="*60)
df = pd.read_csv(data_path)

# Extract filename for saving figures
data_filename = os.path.splitext(os.path.basename(data_path))[0]

# Create time axis (relative time in seconds)
time = df['loop_time']

# Define loop time window [start, end] in seconds
loop_time_window = [time.iloc[0], time.iloc[-1]]
# loop_time_window = [30, 52]
# loop_time_window = [49, 52]

# Filter data based on time window
mask = (time >= loop_time_window[0]) & (time <= loop_time_window[1])
df_filtered = df[mask].copy()
time_filtered = time[mask]

# # COMMENTED OUT - Original comprehensive plots
# # Create a comprehensive figure with multiple subplots
# fig, axes = plt.subplots(4, 2, figsize=(15, 12))
# fig.suptitle(f'Actuator System Data (Time Window: {loop_time_window[0]}-{loop_time_window[1]}s)', fontsize=16, fontweight='bold')

# # 1. Actuator Angle
# axes[0, 0].plot(time_filtered, df_filtered['actuator_angle'], 'b-', linewidth=1)
# axes[0, 0].set_ylabel('Angle (deg)', fontweight='bold')
# axes[0, 0].set_title('Actuator Angle')
# axes[0, 0].grid(True, alpha=0.3)

# # 2. Actuator Velocity
# axes[0, 1].plot(time_filtered, df_filtered['actuator_velocity'], 'g-', linewidth=1)
# axes[0, 1].set_ylabel('Velocity (deg/s)', fontweight='bold')
# axes[0, 1].set_title('Actuator Velocity')
# axes[0, 1].grid(True, alpha=0.3)

# # 3. Actuator Torque
# axes[1, 0].plot(time_filtered, df_filtered['actuator_torque'], 'r-', linewidth=1)
# axes[1, 0].set_ylabel('Torque (Nm)', fontweight='bold')
# axes[1, 0].set_title('Actuator Torque')
# axes[1, 0].grid(True, alpha=0.3)

# # 4. Motor Current
# axes[1, 1].plot(time_filtered, df_filtered['mc_current'], 'm-', linewidth=1)
# axes[1, 1].set_ylabel('Current (A)', fontweight='bold')
# axes[1, 1].set_title('Motor Controller Current')
# axes[1, 1].grid(True, alpha=0.3)

# # 5. Cam Angle
# axes[2, 0].plot(time_filtered, df_filtered['cam_angle'], 'c-', linewidth=1)
# axes[2, 0].set_ylabel('Angle (deg)', fontweight='bold')
# axes[2, 0].set_title('Cam Angle')
# axes[2, 0].grid(True, alpha=0.3)

# # 6. Cam Velocity
# axes[2, 1].plot(time_filtered, df_filtered['cam_velocity'], 'orange', linewidth=1)
# axes[2, 1].set_ylabel('Velocity (deg/s)', fontweight='bold')
# axes[2, 1].set_title('Cam Velocity')
# axes[2, 1].grid(True, alpha=0.3)

# # 7. Disturbance Displacement
# axes[3, 0].plot(time_filtered, df_filtered['disturbance_displacement'], 'darkblue', linewidth=1)
# axes[3, 0].set_ylabel('Displacement', fontweight='bold')
# axes[3, 0].set_xlabel('Time (s)', fontweight='bold')
# axes[3, 0].set_title('Disturbance Displacement')
# axes[3, 0].grid(True, alpha=0.3)

# # 8. Disturbance Velocity
# axes[3, 1].plot(time_filtered, df_filtered['disturbance_velocity'], 'darkgreen', linewidth=1)
# axes[3, 1].set_ylabel('Velocity', fontweight='bold')
# axes[3, 1].set_xlabel('Time (s)', fontweight='bold')
# axes[3, 1].set_title('Disturbance Velocity')
# axes[3, 1].grid(True, alpha=0.3)

# plt.tight_layout()
# plt.show()

# # Plot control signals
# fig, axes = plt.subplots(3, 1, figsize=(15, 10))
# fig.suptitle(f'Control Signals (Time Window: {loop_time_window[0]}-{loop_time_window[1]}s)', fontsize=16, fontweight='bold')

# # 1. Control Torque Components
# axes[0].plot(time_filtered, df_filtered['mc_control_torque'], 'b-', label='Control Torque', linewidth=1)
# axes[0].plot(time_filtered, df_filtered['mc_command_feedforward_torque'], 'r--', label='Feedforward Torque', linewidth=1, alpha=0.7)
# axes[0].set_ylabel('Torque (Nm)', fontweight='bold')
# axes[0].set_title('Motor Control Torque')
# axes[0].legend()
# axes[0].grid(True, alpha=0.3)

# # 2. Command Current
# axes[1].plot(time_filtered, df_filtered['mc_command_current'], 'g-', linewidth=1)
# axes[1].set_ylabel('Current (A)', fontweight='bold')
# axes[1].set_title('Motor Command Current')
# axes[1].grid(True, alpha=0.3)

# # 3. Cam Angle Error
# axes[2].plot(time_filtered, df_filtered['cam_angle_error'], 'r-', linewidth=1)
# axes[2].set_ylabel('Error (deg)', fontweight='bold')
# axes[2].set_xlabel('Time (s)', fontweight='bold')
# axes[2].set_title('Cam Angle Error')
# axes[2].grid(True, alpha=0.3)

# plt.tight_layout()
# plt.show()


# ===== FOCUSED PLOTS - Selected Data =====
fig, axes = plt.subplots(4, 1, figsize=(15, 12))
# fig.suptitle(f'Focused Control Analysis (Time Window: {loop_time_window[0]}-{loop_time_window[1]}s)', fontsize=16, fontweight='bold')

# 1. Actuator: Commanded vs Actual
axes[0].plot(time_filtered, df_filtered['commanded_actuator_torque'], 'r--', label='Commanded Torque', linewidth=1, alpha=0.7)
# axes[0].set_ylabel('Angle (deg)', fontweight='bold')
# axes[0].set_title('Actuator Angle')
axes[0].legend(loc='best')
axes[0].grid(True, alpha=0.3)

axes_twin0 = axes[0].twinx()
axes_twin0.plot(time_filtered, df_filtered['commanded_actuator_velocity'], 'g--', label='Commanded Actuator Velocity', linewidth=1, alpha=0.7)
axes_twin0.plot(time_filtered, df_filtered['actuator_velocity'], 'b-', label='Actuator Velocity', linewidth=1, alpha=0.7)
axes_twin0.plot(time_filtered, df_filtered['disturbance_velocity'], 'g-', label='Disturbance Velocity', linewidth=1, alpha=0.7)
# axes_twin0.set_ylabel('Velocity/Torque', fontweight='bold')
axes_twin0.legend(loc='upper right')

# 2. Cam: Commanded, Actual, and Error
# axes[1].plot(time_filtered, df_filtered['cam_velocity'], 'orange', linewidth=1)
axes[1].plot(time_filtered, df_filtered['commanded_cam_angle'], 'b--', label='Commanded Cam Angle', linewidth=1.5, alpha=0.7)
axes[1].plot(time_filtered, df_filtered['cam_angle'], 'b-', label='Actual Cam Angle', linewidth=1.5)
axes[1].plot(time_filtered, df_filtered['cam_angle_error'], 'r-', label='Cam Angle Error', linewidth=1)
# axes[1].set_ylabel('Angle (deg)', fontweight='bold')
# axes[1].set_title('Cam Angle Tracking')
axes[1].legend(loc='lower left')
axes[1].grid(True, alpha=0.3)

axes_twin1 = axes[1].twinx()
axes_twin1.plot(time_filtered, df_filtered['commanded_actuator_torque'], 'r--', label='Commanded Torque', linewidth=1, alpha=0.7)
# axes_twin1.set_ylabel('Error (deg)', fontweight='bold', color='r')
axes_twin1.tick_params(axis='y', labelcolor='r')
axes_twin1.legend(loc='upper right')

# 3. Motor Control Torque Components
axes[2].plot(time_filtered, df_filtered['commanded_actuator_torque'], 'k--', label='Commanded Torque', linewidth=1, alpha=0.7)
axes[2].plot(time_filtered, df_filtered['mc_control_torque'], 'b-', label='Control Torque', linewidth=1.5)
axes[2].plot(time_filtered, df_filtered['mc_command_feedforward_torque'], 'g--', label='Feedforward Torque', linewidth=1.5, alpha=0.7)
axes[2].plot(time_filtered, df_filtered['mc_torque_error'], 'r-', label='Torque Error', linewidth=1, alpha=0.7)
axes[2].set_ylabel('Torque (Nm)', fontweight='bold')
axes[2].set_title('Motor Control Torque Analysis')
axes[2].legend(loc='best')
axes[2].grid(True, alpha=0.3)

axes_twin2 = axes[2].twinx()
axes_twin2.plot(time_filtered, df_filtered['disturbance_velocity'], 'darkgreen', linewidth=1, label='Disturbance Velocity')
axes_twin2.tick_params(axis='y', labelcolor='darkgreen')
axes_twin2.legend(loc='upper right')

# 4. TTL Signals
axes[3].plot(time_filtered, df_filtered['commanded_actuator_torque'], 'r--', label='Commanded Torque', linewidth=1, alpha=0.7)
axes[3].plot(time_filtered, df_filtered['ttl_signal'], 'b-', label='TTL Signal', linewidth=2, drawstyle='steps-post')
axes[3].plot(time_filtered, df_filtered['ttl_triggered'], 'r-', label='TTL Triggered', linewidth=2, drawstyle='steps-post', alpha=0.7)
axes[3].set_ylabel('Signal State', fontweight='bold')
axes[3].set_xlabel('Time (s)', fontweight='bold')
axes[3].set_title('TTL Signals')
axes[3].legend(loc='best')
axes[3].grid(True, alpha=0.3)
axes[3].set_ylim([-0.1, 1.1])

plt.tight_layout()

# Save or show the figure
try:
    if SAVE_FIGURES:
        output_filename = f'visualization_{data_filename}.png'
        plt.savefig(output_filename, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {output_filename}")
        plt.close()
    else:
        print("Displaying figure... (Close window or press Ctrl+C to exit)")
        plt.show()
except KeyboardInterrupt:
    print("\nVisualization interrupted by user. Closing gracefully...")
    plt.close('all')
finally:
    print("Visualization complete.")