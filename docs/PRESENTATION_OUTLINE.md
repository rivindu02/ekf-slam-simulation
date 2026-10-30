# Presentation outline

## Slide 1: EKF-Based 2D SLAM
- Differential-drive robot with range–bearing landmarks
- Jointly estimate robot pose and an initially unknown map
- Student: ____________________

## Slide 2: Problem and scope
- Odometry accumulates drift; the map is initially unknown.
- 10 stationary landmarks; known IDs; 360° sensing to 4.5 m.
- Known starting pose; Gaussian simulated noise.
- Python implementation plus Gazebo Classic / ROS 2 demo.

## Slide 3: Joint state and uncertainty
- X = [x, y, heading, m1x, m1y, ..., mNx, mNy]
- 23 states after all 10 landmarks are discovered.
- Full covariance preserves robot–landmark correlations.
- The filter receives no true landmark coordinates.

## Slide 4: Prediction and correction
- Predict pose from left/right wheel travel; propagate covariance.
- First sighting: inverse sensor model + correlated augmentation.
- Repeat sighting: range/bearing innovation and Kalman gain.
- Wrap angles; linear solves; Joseph covariance update.

## Slide 5: Trajectory and map
- Black: truth; blue: EKF-SLAM; dashed orange: odometry.
- Illustration uses seed 7, not the trial average.

## Slide 6: Paired Monte Carlo results
- 20 runs per scenario: EKF RMSE 0.238 m; odometry 3.233 m.
- Error bars show sample standard deviation across runs.

## Slide 7: What happens during an outage?
- No landmark corrections between 35 and 55 s.
- Robot position can recover while the map stays distorted.
- No-reuse ablation removes every repeated observation.

## Slide 8: Accuracy is not confidence
- Scale all assumed noise standard deviations by 0.35.
- Trajectory RMSE stays about 0.238 m.
- Mean NIS rises from 2.02 to 16.50: overconfident uncertainty.
- Mean NIS alone does not establish full SLAM consistency.

## Slide 9: Gazebo demonstration
- Gazebo Classic 11 + ROS 2 Humble in devbox.
- Encoder-mode odometry with added wheel noise.
- Synthetic range/bearing from world poses; known IDs.
- Completed headless physics run; GUI launcher supplied.
- Physics-run metrics are separate from Monte Carlo results.

## Slide 10: Conclusions and limitations
- Landmark reuse substantially reduces odometry drift.
- Outages and nonlinear errors can distort the map.
- Eleven automated tests and reproducible experiment outputs.
- Limitations: known IDs, ideal visibility, small map.
- Next: sensor extraction, robust association, more trajectories.
