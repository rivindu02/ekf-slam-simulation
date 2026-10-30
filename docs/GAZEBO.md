# Gazebo Classic 11 + ROS 2 Humble demonstration

## Requirements

Install Gazebo Classic 11, ROS 2 Humble, `gazebo_ros`, `gazebo_plugins`, and NumPy. The demo was tested with Gazebo Classic 11.10.2. Use `gzserver`/`gzclient`; no ROS workspace build or downloaded robot model is required. A Distrobox container can be used when these dependencies are not installed on the host.

Run the ROS adapter with the Python environment supplied by ROS. Use the project `.venv` for plotting and document generation to keep those dependencies separate.

## Display compatibility

The launcher forces `QT_QPA_PLATFORM=xcb` for `gzclient`. Gazebo Classic uses OGRE/GLX embedded windows; the X11 backend avoids the `Invalid parentWindowHandle` error that can occur with native Wayland. The setting applies only to the Gazebo client and leaves your desktop configuration unchanged.

## Run

From the project root, in a terminal with the ROS/Gazebo dependencies installed:

```bash
bash gazebo/run_demo.sh gui 105
```

If using Distrobox, enter your ROS container first, then change to the cloned project directory and run the same command.

For headless validation:

```bash
bash gazebo/run_demo.sh headless 105
```

The duration is in simulated seconds. The robot starts moving once model states are received. Blue chassis with a red front marker indicates heading; orange posts are the landmarks. The robot follows a radius-4 m circular route centred at (0, 5), using simulator pose for demonstration navigation only. This controller is separate from EKF estimation. A conservative stop is applied within 0.8 m of a post centre. This is a fixed-route demonstration with a safety stop, not a general obstacle-avoidance planner. Physics still differs from the independent kinematic experiment.

The GUI launcher also opens RViz with `gazebo/slam.rviz`: green rays show current noisy observations, blue spheres and ID labels show estimated landmarks, an arrow shows estimated robot heading, and a blue path shows the EKF trajectory. A visible counter and terminal messages report how many landmarks are mapped and how many corrections were performed. The Gazebo window itself shows the physical scene; the SLAM overlay is in RViz.

The launcher sources ROS, uses ROS_DOMAIN_ID=74, and uses Gazebo master port 11355. It temporarily disables shell nounset while sourcing ROS because the installed setup script references optional environment variables. It runs a local world with no model-database access. Override ROS_DOMAIN_ID/GAZEBO_MASTER_URI if needed. If another server already occupies this port, choose a different one; do not start two launchers on the same port.

## Signal flow

```text
Gazebo wheel joints -> encoder-mode odometry -> added wheel-travel noise
                                               |
                                               v
                                         EKF prediction

Gazebo model poses -> synthetic range/bearing/ID + noise -> EKF correction
          |
          +-> ground-truth trajectory/map error evaluation

EKF -> ROS estimated path and landmark markers -> CSV/JSON results
```

The filter is never given true landmark coordinates. The sensor adapter knows the simulated environment to synthesize measurements, as any ideal sensor simulator must. Landmark identities are provided by the simulator. The model does not include camera images, ray-cast occlusion, LiDAR extraction, or association inference.

The differential-drive plugin explicitly sets `odometry_source` to 0 (encoder) rather than 1 (world truth). Encoder twist and the message timestamp interval are converted to wheel travel. The configured noise of 0.006 m per wheel update is then added. State messages used for sensor synthesis are asynchronous, so results include some timing mismatch.

## ROS inspection / optional RViz

In another terminal with the same ROS environment:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=74
ros2 topic list
ros2 topic echo /slam_demo/wheel_odom --once
rviz2
```

In RViz set Fixed Frame to `map`, add a Path display using `/slam_demo/estimated_path`, and add a MarkerArray display using `/slam_demo/estimated_landmarks`. The estimated landmarks are blue spheres. RViz is opened automatically in GUI mode; it is not needed for headless runs.

| Topic | Type | Use |
|---|---|---|
| `/slam_demo/cmd_vel` | geometry_msgs/Twist | Robot command |
| `/slam_demo/wheel_odom` | nav_msgs/Odometry | Encoder-mode odometry |
| `/slam_demo/model_states` | gazebo_msgs/ModelStates | Synthetic sensor and evaluation |
| `/slam_demo/estimated_path` | nav_msgs/Path | Estimated trajectory |
| `/slam_demo/estimated_landmarks` | visualization_msgs/MarkerArray | Estimated map |

## Outputs

`results/gazebo/trajectory.csv` stores true, odometry-only and EKF poses over time. `metrics.json` stores RMSE, mapped landmark count and estimated coordinates. `server.log` and `ros_logs/` provide diagnostics. Each run replaces these output files. Interrupted runs save partial trajectories, so check `duration_s` and `landmarks` in `metrics.json` before comparing results.

The Gazebo launcher exits and stops its own server after the node reaches the requested duration. Ctrl+C saves partial results if observations have been received. If Gazebo fails to initialize, inspect `server.log`; run the launcher under `timeout 180` for a bounded diagnostic run. A small built-in Python dependency (NumPy) and ROS message packages must be importable in the ROS environment.

To rebuild the SDF after changing `LANDMARKS` in the kinematic simulator, run on the host:

```bash
.venv/bin/python scripts/build_world.py
```

## References

- [ROS 2 Gazebo Classic integration](https://classic.gazebosim.org/tutorials?cat=connect_ros&tut=ros2_installing)
- [ROS 2 differential-drive plugin source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)
- [World-state plugin source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/src/gazebo_ros_state.cpp)

If a post is reported nearby, the safety stop prevents further motion. Restart the world with the launcher rather than reusing a manually displaced robot. Restart any running demo after updating the project scripts.

An earlier 105 s validation mapped all 10 landmarks from 549 observations and performed 539 EKF corrections, with a minimum post-centre distance of 0.966 m. The latest files in `results/gazebo/` may come from a shorter run. Results from the earlier open-loop controller are retained in `results/gazebo_previous_open_loop/` for comparison.

## Duration and clean interruption

The last argument is the duration in simulated seconds, not the loop count. `bash gazebo/run_demo.sh gui 105` stops automatically at 105 s and saves results. Use `bash gazebo/run_demo.sh gui 300` for a longer run, or `bash gazebo/run_demo.sh gui 0` to continue until Ctrl+C. SIGINT/SIGTERM are handled before ROS context shutdown so the robot stop command and partial result save happen while publishers remain valid.
