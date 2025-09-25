#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 17 07:58:27 2024

@author: woolfrey
"""

import os
import pandas as pd
import matplotlib

# Set matplotlib backend to avoid Qt threading warnings
matplotlib.use('TkAgg')  # Use 'Agg' if you want no GUI output (e.g. for servers)

import matplotlib.pyplot as plt

########## Load the data from file ##############
path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../build/trajectory_test_data.csv"))

data = pd.read_csv(path, header=None)

data.columns = ["Time", "Position", "Velocity", "Acceleration"]  # Name columns

################### Plot the data ###############
fig, ax = plt.subplots(3, 1, dpi=100)  # Create figure with 3 subplots stacked vertically

ax[0].plot(data["Time"], data["Position"], color='k')
ax[1].plot(data["Time"], data["Velocity"], color='k')
ax[2].plot(data["Time"], data["Acceleration"], color='k')
ax[2].set_xlabel("Time")

##### Clean up the figure and remove spines #####
labels = ["Position", "Velocity", "Acceleration"]
for i in range(3):
    ax[i].spines['top'].set_visible(False)
    ax[i].spines['right'].set_visible(False)
    ax[i].set_ylabel(labels[i])
    ax[i].set_yticks([data[labels[i]].min(), data[labels[i]].max()])
    if i < 2:
        ax[i].spines['bottom'].set_visible(False)
        ax[i].set_xticks([])
    if i > 0:
        ax[i].axhline(y=0, color=[0.2, 0.2, 0.2], linewidth=0.5)

plt.show()