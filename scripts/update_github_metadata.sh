#!/usr/bin/env bash
# Apply the repository description and discovery topics with GitHub CLI.
set -euo pipefail
gh repo edit "${1:-rivindu02/ekf-slam-simulation}" \
    --description "2D landmark-based EKF-SLAM in Python with loop-closure replay, Monte Carlo experiments, and Gazebo/ROS 2 visualization." \
    --add-topic slam \
    --add-topic ekf-slam \
    --add-topic extended-kalman-filter \
    --add-topic robotics \
    --add-topic localization \
    --add-topic mapping \
    --add-topic loop-closure \
    --add-topic differential-drive \
    --add-topic python \
    --add-topic numpy \
    --add-topic matplotlib \
    --add-topic gazebo \
    --add-topic ros2 \
    --add-topic simulation
