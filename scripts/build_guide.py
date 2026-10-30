#!/usr/bin/env python3
"""Build one offline HTML learning guide from the project's recorded artifacts."""
import base64, json, csv
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
summary = json.loads((ROOT/'results/summary.json').read_text())
data = {'summary':summary, 'replays':{}, 'sources':{}, 'figures':{}}
for item in summary['summary']:
    name = item['scenario']
    with np.load(ROOT/'results'/f'{name}_seed7.npz') as a:
        indices = sorted(set(range(0,len(a['time']),5)) | {len(a['time'])-1})
        data['replays'][name] = {key: np.round(a[key][indices],6).tolist()
                                for key in ['time','truth','odom','estimate','sigma','seen','landmark_rmse',
                                            'prior_estimate','updates','loop_closures','association']}
        for key in ['landmark_estimates', 'measurements']:
            values = np.round(a[key][indices], 6)
            # Missing map points/readings stay absent in JSON, never at (0, 0).
            data['replays'][name][key] = [
                [point.tolist() if np.isfinite(point).all() else None for point in frame]
                for frame in values]
        data['replays'][name]['final_state'] = a['state'].tolist()
        data['replays'][name]['final_covariance'] = a['covariance'].tolist()
        data['replays'][name]['ids'] = a['landmark_ids'].tolist()
        data['replays'][name]['map_indices'] = a['landmark_indices'].tolist()
for file in ['src/ekf_slam.py','src/simulation.py','src/demo_control.py','gazebo/slam_node.py',
             'gazebo/run_demo.sh','gazebo/slam.rviz','scripts/run_experiments.py',
             'tests/test_slam.py','tests/test_demo_control.py','scripts/validate_gazebo_shutdown.py']:
    data['sources'][file] = (ROOT/file).read_text()
for name in ['trajectory_map','pose_errors','monte_carlo','revisit_dropout']:
    data['figures'][name] = 'data:image/png;base64,'+base64.b64encode((ROOT/'results'/f'{name}.png').read_bytes()).decode()
data['figures']['gazebo_results'] = 'data:image/png;base64,'+base64.b64encode((ROOT/'results/gazebo/gazebo_results.png').read_bytes()).decode()
data['gazebo'] = json.loads((ROOT/'results/gazebo/metrics.json').read_text())
grows = list(csv.DictReader((ROOT/'results/gazebo/trajectory.csv').open()))
data['gazebo_path'] = [[float(r[k]) for k in ['time','true_x','true_y','true_yaw','ekf_x','ekf_y','ekf_yaw']]
                        for r in grows[::5]]
# Keep embedded source strings from closing the HTML script element.
encoded = json.dumps(data, separators=(',',':'), allow_nan=False).replace('<','\\u003c')
html = (ROOT/'docs/guide.template.html').read_text().replace('__PROJECT_DATA__',encoded)
(ROOT/'PROJECT_GUIDE.html').write_text(html)
print(f'Built PROJECT_GUIDE.html ({len(html.encode())/1024/1024:.2f} MiB), fully offline.')
