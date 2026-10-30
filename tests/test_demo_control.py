"""Navigation checks for the separate Gazebo demonstration controller."""
import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from demo_control import route_command
from ekf_slam import motion
from simulation import LANDMARKS


class DemoControlTests(unittest.TestCase):
    def test_full_route_clearance_and_landmark_coverage(self):
        pose = np.zeros(3)
        minimum = float('inf')
        seen = set()
        for _ in range(1050):
            v, w, nearest = route_command(pose, LANDMARKS)
            minimum = min(minimum, nearest)
            pose = motion(pose, .1*np.array([v-.2*w, v+.2*w]), .4)[0]
            seen.update(np.flatnonzero(np.linalg.norm(LANDMARKS-pose[:2], axis=1) <= 4.5))
        # Robot + post footprint is under 0.45 m; require substantially more clearance.
        self.assertGreater(minimum, 1.)
        self.assertEqual(len(seen), len(LANDMARKS))

    def test_close_obstacle_stops_motion(self):
        v, w, nearest = route_command([1., 2., .7], [[1.3, 2.]])
        self.assertEqual((v, w), (0., 0.))
        self.assertAlmostEqual(nearest, .3)

if __name__ == '__main__':
    unittest.main()
