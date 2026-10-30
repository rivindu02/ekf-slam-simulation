"""Gazebo demonstration navigation, independent of the SLAM estimator.

Uses simulator pose for route control only, not EKF correction. A radius-4 m
circle centred at (0, 5) leaves clearance around the landmark posts.
"""
import numpy as np
from ekf_slam import wrap


def route_command(pose, obstacles=()):
    position = np.asarray(pose[:2])
    distances = [float(np.linalg.norm(np.asarray(p)-position)) for p in obstacles]
    nearest = min(distances, default=float('inf'))
    # Conservative stop: chassis circumscribed radius + post radius < 0.45 m.
    if nearest < .8:
        return 0., 0., nearest
    phase = np.arctan2(pose[0], 5.-pose[1])
    target = np.array([4*np.sin(phase+.25), 5.-4*np.cos(phase+.25)])
    delta = target-position
    distance = np.linalg.norm(delta)
    error = wrap(np.arctan2(delta[1], delta[0])-pose[2])
    speed = .6*max(.15, float(np.cos(error)))
    if abs(error) > 1.2:
        speed = 0.
        turn = np.clip(1.5*error, -1., 1.)
    else:
        turn = np.clip(2*speed*np.sin(error)/max(distance, .1), -1., 1.)
    return float(speed), float(turn), nearest
