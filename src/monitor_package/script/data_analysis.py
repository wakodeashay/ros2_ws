import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Read CSV files
csv_def = pd.read_csv('nav_log_def.csv')
csv_mod = pd.read_csv('nav_log_mod.csv')

# Create figure and subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

# Calculate variances for linear velocity
def_lin_x_var = np.var(csv_def['lin_x'])
mod_lin_x_var = np.var(csv_mod['lin_x'])
lin_text = f'Def lin vel variance: {def_lin_x_var:.4f}\nMod lin vel variance: {mod_lin_x_var:.4f}'

# Plot Linear Velocity
ax1.plot(csv_def['lin_x'], label='Linear Velocity (Default Planner)')
ax1.plot(csv_mod['lin_x'], label='Linear Velocity (Modified Planner)')
ax1.set_title('Comparison of Linear Velocity')
ax1.set_xlabel('Index')
ax1.set_ylabel('Linear Velocity (m/s)')
ax1.legend()
ax1.grid()

# Position the text within the linear velocity subplot (top-left corner)
ax1.text(0.5, 0.15, lin_text, transform=ax1.transAxes, fontsize=12,
         verticalalignment='top', bbox=dict(facecolor='red', alpha=0.3))

# Calculate variances for angular velocity
def_ang_z_var = np.var(csv_def['ang_z'])
mod_ang_z_var = np.var(csv_mod['ang_z'])
ang_text = f'Def ang vel variance: {def_ang_z_var:.4f}\nMod ang vel variance: {mod_ang_z_var:.4f}'

# Plot Angular Velocity
ax2.plot(csv_def['ang_z'], label='Angular Velocity (Default Planner)')
ax2.plot(csv_mod['ang_z'], label='Angular Velocity (Modified Planner)')
ax2.set_title('Comparison of Angular Velocity')
ax2.set_xlabel('Index')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.legend()
ax2.grid()

# Position the text within the angular velocity subplot (top-left corner)
ax2.text(0.02, 0.95, ang_text, transform=ax2.transAxes, fontsize=12,
         verticalalignment='top', bbox=dict(facecolor='blue', alpha=0.3))

# Adjust layout
plt.tight_layout()

# Show plots
plt.show()
