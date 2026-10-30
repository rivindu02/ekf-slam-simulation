"""Plot the completed Gazebo run with host Matplotlib."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = Path(__file__).resolve().parents[1]
a = np.genfromtxt(ROOT/'results/gazebo/trajectory.csv', delimiter=',', names=True)
fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
for prefix, label, style in [('true','Gazebo truth','k-'), ('odom','Noisy encoder odometry','--'), ('ekf','EKF-SLAM','b-')]:
    axes[0].plot(a[prefix+'_x'], a[prefix+'_y'], style, label=label)
axes[0].set(xlabel='x (m)', ylabel='y (m)', title='Physics-simulation trajectory')
axes[0].axis('equal'); axes[0].legend(fontsize=8)
for prefix, label in [('odom','Odometry'), ('ekf','EKF-SLAM')]:
    axes[1].plot(a['time'], np.hypot(a[prefix+'_x']-a['true_x'], a[prefix+'_y']-a['true_y']),label=label)
axes[1].set(xlabel='Time (s)', ylabel='Position error (m)',title='One Gazebo run, synthetic landmark sensor')
axes[1].legend()
for ax in axes: ax.grid(alpha=.25)
fig.tight_layout(); fig.savefig(ROOT/'results/gazebo/gazebo_results.png', dpi=180)
fig.savefig(ROOT/'results/gazebo/gazebo_results.pdf')
