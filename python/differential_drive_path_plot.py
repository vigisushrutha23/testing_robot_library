#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 16 14:37:03 2025

@author: woolfrey
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import numpy as np
from functools import reduce
from matplotlib.animation import FuncAnimation

plt.close('all')
phi = 1.618

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load desired configuration data
desired_csv_path = os.path.join(script_dir, '..', 'build', 'desired_configuration_data.csv')
desired_data = np.loadtxt(desired_csv_path, delimiter=',')
time = desired_data[:, 0]
x_desired = desired_data[:, 1]
y_desired = desired_data[:, 2]
heading_desired = desired_data[:, 3]

# Load actual configuration data
actual_csv_path = os.path.join(script_dir, '..', 'build', 'actual_configuration_data.csv')
actual_data = np.loadtxt(actual_csv_path, delimiter=',')
x_actual = actual_data[:, 1]
y_actual = actual_data[:, 2]
heading_actual = actual_data[:, 3]

# Load the first row of obstacle_path.csv (time, center_x, center_y, radius_x, radius_y)
obstacle_csv_path = os.path.join(script_dir, '..', 'build', 'obstacle_data.csv')
obstacle_exists = os.path.exists(obstacle_csv_path)

if obstacle_exists == True:
    obstacle_rows = np.loadtxt(obstacle_csv_path, delimiter=',')
   # time_obs = obstacle_first_row[0]
    #center_x = obstacle_first_row[1]
    #center_y = obstacle_first_row[2]
    #radius_x = obstacle_first_row[3]
    #radius_y = obstacle_first_row[4]

# Plot desired and actual paths
fig1, ax1 = plt.subplots()

ax1.plot(x_desired, y_desired, label='Desired', color='black')
ax1.plot(x_actual,  y_actual,  label='Actual',  color='red')

# Plot the obstacle ellipsoid as an ellipse patch
#if obstacle_exists:
#    ellipse = patches.Ellipse((center_x, center_y), width=2*radius_x, height=2*radius_y,
#                            edgecolor='blue', facecolor='none', linewidth=2, label='Obstacle Ellipsoid')
#    ax1.add_patch(ellipse)
#    ax1.text(center_x, center_y, f't={time_obs:.2f}', color='blue', fontsize=8)

# Arrows for desired path start and end
arrow_length = 0.05
ax1.arrow(x_actual[0], y_actual[0],
          arrow_length * np.cos(heading_actual[0]),
          arrow_length * np.sin(heading_actual[0]),
          head_width=0.03, head_length=0.05, fc='red', ec='red')

ax1.arrow(x_actual[-1], y_actual[-1],
          arrow_length * np.cos(heading_actual[-1]),
          arrow_length * np.sin(heading_actual[-1]),
          head_width=0.03, head_length=0.05, fc='red', ec='red')
          
# Load ellipsoid data
ellipsoid_csv_path = os.path.join(script_dir, '..', 'build', 'ellipsoid_data.csv')
ellipsoid_data = np.loadtxt(ellipsoid_csv_path, delimiter=',')
p_x = ellipsoid_data[0]
p_y = ellipsoid_data[1]
# A from CSV
A = np.array([[ellipsoid_data[2], ellipsoid_data[3]],
              [ellipsoid_data[4], ellipsoid_data[5]]])

# Eigen-decomposition
eigvals, eigvecs = np.linalg.eigh(A)
order = np.argsort(eigvals)[::-1]  # largest first
eigvals = eigvals[order]
eigvecs = eigvecs[:, order]

# Width and height are 2*sqrt(eigenvalues)
width  = 2 * np.sqrt(eigvals[0])
height = 2 * np.sqrt(eigvals[1])

# Rotation angle
angle = np.degrees(np.arctan2(eigvecs[1,0], eigvecs[0,0]))

# Add ellipse patch
ellipse = patches.Ellipse((p_x, p_y), width=width, height=height,
                          angle=angle, edgecolor='blue', facecolor='none', linewidth=2)
ax1.add_patch(ellipse)


# Style adjustments
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
ax1.grid(False)
ax1.set_xlabel('X Position')
ax1.set_ylabel('Y Position')
ax1.set_title('Cartesian Path')
ax1.axis('equal')
robot_ellipse = None
obstacle_ellipse = None
def animate(i):
    global robot_ellipse
    global obstacle_ellipse
    global obstacle_rows
    if robot_ellipse is not None:
        robot_ellipse.remove()
    if obstacle_exists:
        if obstacle_ellipse is not None:
            obstacle_ellipse.remove()
        time_obs = obstacle_rows[i,0]
        center_x = obstacle_rows[i,1]
        center_y = obstacle_rows[i,2]
        radius_x = obstacle_rows[i,3]
        radius_y = obstacle_rows[i,4]
        ellipse = patches.Ellipse((center_x, center_y), width=2*radius_x, height=2*radius_y,
                                edgecolor='blue', facecolor='none', linewidth=2, label='Obstacle Ellipsoid')
        obstacle_ellipse = ax1.add_patch(ellipse)
    r_ellipse = patches.Ellipse((x_actual[i], y_actual[i]), width=2*0.1, height=2*0.05, angle = np.degrees(heading_actual[i])-90,
                                edgecolor='red', facecolor='none', linewidth=2, label='Robot Ellipsoid')
    robot_ellipse = ax1.add_patch(r_ellipse)
    return [robot_ellipse, obstacle_ellipse]
anim = FuncAnimation(fig1, animate,frames = len(x_actual), interval = 10, repeat = False, cache_frame_data = False )
anim
plt.legend(loc="upper left")
plt.show()


### Load control input data
control_csv_path = os.path.join(script_dir, '..', 'build', 'control_input_data.csv')
control_data = np.loadtxt(control_csv_path, delimiter=',')
control_time = control_data[:, 0]
linear_velocity = control_data[:, 1]
angular_velocity = control_data[:, 2]

# Plot control inputs in subfigures
fig2, (ax2_1, ax2_2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

# Linear velocity
ax2_1.plot(control_time, linear_velocity, color='black')
ax2_1.set_ylabel('Linear Velocity (m/s)')
ax2_1.spines['top'].set_visible(False)
ax2_1.spines['right'].set_visible(False)
ax2_1.spines['bottom'].set_visible(False)
ax2_1.tick_params(axis='x', which='both', bottom=False, top=False)
ax2_1.grid(False)

# Angular velocity
ax2_2.plot(control_time, angular_velocity * (30 / np.pi), color='black')
ax2_2.set_ylabel('Angular Velocity (rpm)')
ax2_2.set_xlabel('Time (s)')
ax2_2.spines['top'].set_visible(False)
ax2_2.spines['right'].set_visible(False)
ax2_2.grid(False)
fig2.suptitle('Control Inputs')

# Load tracking error data
error_csv_path = os.path.join(script_dir, '..', 'build', 'tracking_error_data.csv')
error_data = np.loadtxt(error_csv_path, delimiter=',')
error_time = error_data[:, 0]
position_error_mm = error_data[:, 1] * 1000  # Convert meters to mm
orientation_error_deg = np.degrees(error_data[:, 2])  # Convert radians to degrees

# Plot tracking errors in subfigures
fig3, (ax3_1, ax3_2) = plt.subplots(2, 1, figsize=(4*phi, 4), sharex=True)

# Position error
ax3_1.plot(error_time, position_error_mm, color='black')
ax3_1.set_ylabel('Position Error (mm)')
ax3_1.spines['top'].set_visible(False)
ax3_1.spines['right'].set_visible(False)
ax3_1.spines['bottom'].set_visible(False)
ax3_1.tick_params(axis='x', which='both', bottom=False, top=False)
ax3_1.grid(False)

# Orientation error
ax3_2.plot(error_time, orientation_error_deg, color='black')
ax3_2.set_ylabel('Orientation Error (°)')
ax3_2.set_xlabel('Time (s)')
ax3_2.spines['top'].set_visible(False)
ax3_2.spines['right'].set_visible(False)
ax3_2.grid(False)
fig3.suptitle('Tracking Errors')

plt.tight_layout()
plt.show()
