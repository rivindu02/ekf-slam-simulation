# EKF-Based 2D SLAM mini project

For a top-to-bottom interactive explanation, open **PROJECT_GUIDE.html** in Brave. It works offline and includes recorded experiment replay, animated teaching diagrams, equations, source code, results, quizzes, and run instructions. Rebuild it with `.venv/bin/python scripts/build_guide.py` after changing source/results.

The replay shows the evolving estimated map and recorded noisy sensor readings. Click **Next loop closure** to inspect returning landmark IDs, pose corrections and update counts. Orange rings mark new landmarks, green rings mark matches and purple rings mark corrections after at least 10 seconds without an observation. IDs come from the synthetic sensor. The trajectory preserves past estimates; loop closure updates the current pose and map without rewriting the historical path. After changing simulation recording fields, regenerate the NPZ files with `scripts/run_experiments.py --trials 20` before rebuilding the guide.

For the assignment document, start with **report/EKF_SLAM_Report.pdf**. The editable version is **report/EKF_SLAM_Report.docx**. Fill in the four cover-page fields before submitting. The source report is `report/report.md`; numerical results are measured simulation results, not invented or real-world data.

## Run the independent simulation

From this project directory (Python 3.10 or newer):

```bash
python -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v
node scripts/validate_guide_replay.cjs # optional offline replay logic check
.venv/bin/python scripts/run_experiments.py --trials 20 --animate
.venv/bin/python scripts/build_report.py
.venv/bin/python scripts/build_presentation.py
```

The existing `.venv` is already installed and validated on this host. Gazebo is optional for these commands. On a machine without a writable Matplotlib cache, prefix plotting commands with `MPLCONFIGDIR=/tmp/ekf-slam-mpl`.

`requirements-lock.txt` records the validated host packages (Python 3.14). Use `requirements.txt` to resolve compatible wheels on another Python version. No MATLAB license is needed.

## What is included

| Path | Purpose |
|---|---|
| `src/ekf_slam.py` | Explicit EKF prediction, correction, Jacobians and correlated landmark initialization |
| `src/simulation.py` | Truth simulator, noisy sensor, scenarios and metrics |
| `scripts/run_experiments.py` | 20 paired trials × 6 scenarios; plots and GIF |
| `tests/test_slam.py`, `tests/test_demo_control.py` | Eleven mathematical, controller, loop-closure and replay-recording checks |
| `results/metrics.csv` | All 120 per-run metric rows |
| `results/summary.json` | Aggregated results and exact configurations |
| `results/*_seed7.npz` | Time histories, final map state/covariance and landmark indices |
| `results/*.png`, `*.pdf` | Figures for the report |
| `results/trajectory_demo.gif` | Animated trajectory comparison |
| `results/gazebo/` | Separate completed physics-simulation results |
| `report/` | PDF, editable DOCX, Markdown and presentation PDF |
| `docs/GAZEBO.md` | Gazebo Classic 11 / ROS 2 Humble instructions |
| `docs/VIVA_NOTES.md` | Explanation, demonstration sequence and likely questions |
| `docs/ASSIGNMENT_MAPPING.md` | Where each assignment requirement is addressed |

## Results and scope

Across seeds 0–19, baseline trajectory RMSE is 0.238 ± 0.146 m for EKF-SLAM versus 3.233 ± 1.172 m for odometry alone (mean ± sample SD). Final map RMSE is 0.207 ± 0.182 m. The figures use seed 7 and are not averages.

The map starts empty. The filter receives noisy wheel travel and landmark ID/range/bearing only. Ground-truth coordinates are used outside the filter for generating simulated observations and scoring. There are ten static point landmarks, known IDs, a 360-degree range-limited sensor, no occlusion, a known starting pose, and approximately two circular loops. This is landmark SLAM, not occupancy-grid or LiDAR SLAM.

The high-noise, dropout, no-reuse, and wrong-noise experiments show limitations as well as improvements. The dropout case can retain large map distortion; a low final robot error does not guarantee a good map. No landmark-reuse means all repeated corrections are removed, not just one loop closure.

## Gazebo in devbox

```bash
distrobox enter devbox -- bash -lc 'cd "/home/rivindu02/Documents/Robotics Mini Project" && bash gazebo/run_demo.sh gui 105'
```

GUI mode opens Gazebo and a separate RViz window showing green detection rays, blue estimated landmarks with IDs, and the EKF trajectory. The terminal prints visible landmark IDs and map/update counts. The robot follows a safer circle using simulator pose for demo navigation, with a near-post safety stop. The estimator does not receive the simulator pose as a correction. Use `headless` in place of `gui` for the non-GUI run. The final argument is simulated seconds: `105` stops automatically, `300` runs longer, and `0` runs until Ctrl+C while still saving partial results. The script stops its own server when complete; it does not stop unrelated Gazebo instances. Gazebo statistics are separate from the kinematic Monte Carlo statistics. The adapter uses a synthetic range–bearing sensor from simulator poses; camera/LiDAR landmark detection is not implemented.

## Rebuild a portable submission

```bash
.venv/bin/python scripts/package_submission.py
```

This creates `EKF_SLAM_Submission.zip`, excluding virtual environments, Git files, caches and ROS log directories. Review the report and fill in personal details in the DOCX, or update the template in `scripts/build_report.py` and rebuild both formats. Rebuilding overwrites generated report files.
