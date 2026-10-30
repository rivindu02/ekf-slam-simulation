# Demonstration and viva notes

## A short demonstration

1. Open the report and explain that the map starts empty.
2. Show src/ekf_slam.py: state, prediction, new-landmark initialization, correction.
3. Run `.venv/bin/python -m unittest discover -s tests -v`.
4. Show results/trajectory_demo.gif and results/trajectory_map.png.
5. Show results/monte_carlo.png: 20 runs per scenario, not one lucky run.
6. Explain the dropout failure and underestimated-noise diagnostic.
7. If time allows, launch Gazebo following GAZEBO.md. State explicitly that landmark observations are synthetic with known IDs.
8. Show results/gazebo/metrics.json and distinguish that single physics run from the kinematic Monte Carlo experiment.

## Likely questions

**Why EKF rather than ordinary KF?**
The heading-dependent motion and range/bearing observations are nonlinear. EKF linearizes them locally with Jacobians while maintaining a joint Gaussian approximation.

**What is estimated?**
Robot x, y and heading, plus x/y coordinates for every discovered landmark. Ten landmarks produce 23 states.

**What distinguishes this from localization?**
Localization assumes a known map. This estimator begins with no landmark coordinates and jointly estimates them with robot pose.

**Does knowing IDs make the map known?**
No. An ID says which object is observed; it does not give its coordinates. It simplifies association. Incorrect associations and feature detection remain outside this project.

**Does the EKF use ground truth?**
Only outside the filter: truth drives sensor synthesis and error scoring. The EKF receives wheel travel and noisy ID/range/bearing observations. In Gazebo the same separation applies.

**Why preserve cross-covariances?**
Landmarks observed from an uncertain pose share that pose uncertainty. Their correlations let later landmark observations constrain the joint robot/map state.

**How is a landmark initialized?**
Convert its range/bearing into world x/y using the estimated robot pose. Propagate robot and sensor uncertainty through the inverse-model Jacobians, including cross-covariance with every existing state.

**Why not immediately update using the first observation?**
Initialization already uses that measurement. Correcting with it again counts the same information twice.

**Why wrap angles?**
Bearings near +pi and -pi describe nearly the same direction. Unwrapped subtraction can turn a tiny error into an approximately 2pi error.

**What do Q and R mean?**
Qw models wheel-travel errors in m² per sample. R contains range variance in m² and bearing variance in rad². V maps wheel uncertainty into robot pose uncertainty.

**Why Joseph covariance update?**
It is numerically more stable than a subtractive covariance update. Linear solves avoid explicitly calculating a matrix inverse.

**Does a final good robot pose prove the map is good?**
No. The dropout result demonstrates that final robot position may recover while landmark estimates remain distorted.

**Why does uniform noise underestimation barely change RMSE?**
Scaling Q and R together approximately preserves their ratio and the Kalman gains while shrinking covariance. NIS rises dramatically, exposing overconfidence even when mean estimates look similar.

**What does mean NIS near two show?**
It is a useful check for the two-component innovation under ideal assumptions. It does not establish full covariance consistency, particularly in nonlinear correlated SLAM.

**Is the no-reuse experiment specifically a loop-closure ablation?**
No. It removes all corrections to previously seen landmarks. This includes observations made shortly after initialization, not only after a full loop.

**Why is the Gazebo route different?**
The independent simulator uses ideal kinematics; Gazebo includes physical contacts, dynamics and slip. The Gazebo demo uses a separate simulator-pose route controller for a safer radius-4 m circle; the standalone experiment uses prescribed kinematics. The controller is not part of the EKF. Green rays in RViz show observations; blue spheres and labels show the estimated map.

**What are the main limitations?**
Known IDs, idealized sensor visibility, small map, Gaussian independent noise, known starting pose, linearization sensitivity, and dense covariance cost. No real sensor extraction or occupancy map is implemented.

**What would improve this next?**
Test bias and outliers, add robust association and gating with evaluation, extract actual sensor landmarks, assess consistency across more trajectories, or compare a graph-based SLAM implementation.
