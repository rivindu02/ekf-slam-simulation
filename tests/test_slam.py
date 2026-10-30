"""Numerical checks of the estimator's most failure-prone mathematics."""
import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from ekf_slam import EKFSLAM, motion, observation, wrap
from simulation import run, Config, LANDMARKS, NEW, UPDATED, REVISITED, IGNORED


def numerical_jacobian(fun, x):
    eps = 1e-6
    columns = []
    for i in range(len(x)):
        delta = np.zeros_like(x); delta[i] = eps
        columns.append((fun(x+delta)-fun(x-delta))/(2*eps))
    return np.column_stack(columns)


class SlamTests(unittest.TestCase):
    def test_motion_jacobians(self):
        pose, wheels = np.array([1., 2., .7]), np.array([.05, .09])
        _, G, V = motion(pose, wheels, .4)
        np.testing.assert_allclose(G, numerical_jacobian(lambda p: motion(p, wheels, .4)[0], pose), atol=1e-8)
        np.testing.assert_allclose(V, numerical_jacobian(lambda u: motion(pose, u, .4)[0], wheels), atol=1e-8)

    def test_measurement_jacobian(self):
        state = np.array([1., 2., .7, 3., 5., -1., 6.])
        _, H = observation(state, 3)
        np.testing.assert_allclose(H, numerical_jacobian(lambda x: observation(x, 3)[0], state), atol=1e-8)

    def test_augmentation_cross_covariance(self):
        f = EKFSLAM(np.eye(2)*.001, np.diag([.01, .0004]))
        f.P = np.diag([.1, .2, .05])
        old = f.P.copy()
        z = np.array([3., .4])
        A = np.array([[1, 0, -3*np.sin(.4)], [0, 1, 3*np.cos(.4)]])
        f.correct(42, z)
        np.testing.assert_allclose(f.P[3:, :3], A@old)
        self.assertEqual(len(f.nis), 0, 'Initialization must not double-count the observation')
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), -1e-10)
        self.assertEqual(f.ids, {42: 3})

    def test_wrapped_bearing_innovation(self):
        f = EKFSLAM(np.eye(2)*.001, np.diag([.01, .0004]))
        f.correct(0, [2., np.pi-.001])
        nis = f.correct(0, [2., -np.pi+.001])
        self.assertLess(nis, .1)
        self.assertTrue(-np.pi <= f.x[2] < np.pi)

    def test_zero_noise_and_covariance(self):
        h, f, metrics, _ = run(Config(steps=300, wheel_std=0., range_std=0., bearing_std=0.), 3)
        np.testing.assert_allclose(h['estimate'], h['truth'], atol=1e-7)
        np.testing.assert_allclose(f.P, f.P.T, atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), -1e-9)
        self.assertGreater(metrics['landmarks'], 0)

    def test_reproducible_and_baseline_improves(self):
        a, f, metrics, _ = run(Config(), 7)
        b, _, metrics2, _ = run(Config(), 7)
        np.testing.assert_array_equal(a['estimate'], b['estimate'])
        self.assertEqual(metrics, metrics2)
        self.assertLess(metrics['position_rmse_m'], metrics['odometry_rmse_m'])
        self.assertEqual(metrics['landmarks'], 10)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), -1e-9)

    def test_replay_records_map_and_real_associations(self):
        h, f, metrics, _ = run(Config(), 7)
        # Future landmarks must not appear in the historical map.
        initial_ids = np.flatnonzero(h['association'][0] == NEW)
        self.assertEqual(len(initial_ids), h['seen'][0])
        self.assertTrue(np.isnan(h['landmark_estimates'][0, 9]).all())
        self.assertEqual(np.count_nonzero(h['association'] == NEW), len(f.ids))
        for identity, index in f.ids.items():
            np.testing.assert_array_equal(h['landmark_estimates'][-1, identity], f.x[index:index+2])
        corrections = np.sum(np.isin(h['association'], [UPDATED, REVISITED]), axis=1)
        np.testing.assert_array_equal(h['updates'], np.cumsum(corrections))
        self.assertEqual(h['updates'][-1], metrics['updates'])
        # These are the noisy sensor measurements, not rays to known true points.
        z = h['measurements'][0, initial_ids[0]]
        true_range = np.linalg.norm(LANDMARKS[initial_ids[0]]-h['truth'][0, :2])
        self.assertNotEqual(z[0], true_range)
        self.assertTrue(np.isnan(h['measurements'][1]).all())

    def test_loop_closure_reuses_map_and_corrects_drift(self):
        h, _, _, _ = run(Config(), 7)
        returning = np.flatnonzero(np.any(h['association'] == REVISITED, axis=1))
        self.assertGreater(len(returning), 0)
        k = returning[0]
        for identity in np.flatnonzero(h['association'][k] == REVISITED):
            previous = np.flatnonzero(h['association'][:k, identity])[-1]
            self.assertGreaterEqual(h['time'][k]-h['time'][previous], 10.)
        self.assertEqual(h['seen'][k], h['seen'][k-1])
        before = np.linalg.norm(h['prior_estimate'][k, :2]-h['truth'][k, :2])
        after = np.linalg.norm(h['estimate'][k, :2]-h['truth'][k, :2])
        self.assertLess(after, before)
        self.assertGreater(h['loop_closures'][-1], 0)
        self.assertLess(np.linalg.norm(h['estimate'][-1, :2]-h['truth'][-1, :2]), .05)

    def test_no_reuse_replay_has_no_false_loop_closures(self):
        h, _, _, _ = run(Config(reuse_landmarks=False), 7)
        self.assertTrue(np.any(h['association'] == IGNORED))
        self.assertFalse(np.any(h['association'] == REVISITED))
        np.testing.assert_array_equal(h['updates'], 0)
        np.testing.assert_array_equal(h['loop_closures'], 0)
        np.testing.assert_array_equal(h['estimate'], h['odom'])

if __name__ == '__main__':
    unittest.main(verbosity=2)
