"""Small, explicit EKF-SLAM implementation. Units: metres and radians."""
import numpy as np


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def motion(pose, wheels, track):
    """Midpoint differential-drive model and Jacobians wrt pose and wheel travel.

    wheels = [left travel, right travel], metres over one sample.
    This is a midpoint approximation, not exact circular-arc integration.
    """
    left, right = wheels
    distance = (left + right) / 2
    rotation = (right - left) / track
    a = pose[2] + rotation / 2
    c, s = np.cos(a), np.sin(a)
    out = np.asarray(pose, dtype=float).copy()
    out += [distance*c, distance*s, rotation]
    out[2] = wrap(out[2])
    G = np.array([[1., 0., -distance*s], [0., 1., distance*c], [0., 0., 1.]])
    V = np.array([[c/2+distance*s/(2*track), c/2-distance*s/(2*track)],
                  [s/2-distance*c/(2*track), s/2+distance*c/(2*track)],
                  [-1/track, 1/track]])
    return out, G, V


def observation(state, index):
    """Range/bearing prediction and full-state Jacobian for a known map index."""
    dx, dy = state[index:index+2] - state[:2]
    q = dx*dx + dy*dy
    if q < 1e-12:
        raise ValueError('Landmark is too close to define a bearing')
    r = np.sqrt(q)
    z = np.array([r, wrap(np.arctan2(dy, dx)-state[2])])
    H = np.zeros((2, len(state)))
    H[:, :3] = [[-dx/r, -dy/r, 0], [dy/q, -dx/q, -1]]
    H[:, index:index+2] = [[dx/r, dy/r], [-dy/q, dx/q]]
    return z, H


class EKFSLAM:
    def __init__(self, wheel_cov, measurement_cov, track=0.4):
        # Initial pose defines the map coordinate frame. No landmark coordinates supplied.
        self.x = np.zeros(3)
        self.P = np.eye(3)*1e-10
        self.wheel_cov = np.asarray(wheel_cov)
        self.R = np.asarray(measurement_cov)
        self.track = track
        self.ids = {}
        self.nis = []

    def predict(self, wheels):
        pose, G, V = motion(self.x[:3], wheels, self.track)
        self.x[:3] = pose
        F = np.eye(len(self.x))
        F[:3, :3] = G
        self.P = F @ self.P @ F.T
        self.P[:3, :3] += V @ self.wheel_cov @ V.T
        self.P = (self.P+self.P.T)/2

    def initialize(self, identity, z):
        r, bearing = z
        a = self.x[2]+bearing
        c, s = np.cos(a), np.sin(a)
        landmark = self.x[:2] + r*np.array([c, s])
        A = np.zeros((2, len(self.x)))
        A[:, :3] = [[1, 0, -r*s], [0, 1, r*c]]
        B = np.array([[c, -r*s], [s, r*c]])
        cross = A @ self.P
        lower = A @ self.P @ A.T + B @ self.R @ B.T
        self.ids[identity] = len(self.x)
        self.x = np.r_[self.x, landmark]
        self.P = np.block([[self.P, cross.T], [cross, lower]])
        # The initializing observation is not used again in a correction.

    def correct(self, identity, z):
        if identity not in self.ids:
            self.initialize(identity, z)
            return None
        predicted, H = observation(self.x, self.ids[identity])
        innovation = np.asarray(z)-predicted
        innovation[1] = wrap(innovation[1])
        S = H @ self.P @ H.T + self.R
        K = np.linalg.solve(S, H @ self.P).T
        self.x += K @ innovation
        self.x[2] = wrap(self.x[2])
        # Joseph form preserves covariance symmetry/PSD better than subtraction.
        A = np.eye(len(self.x))-K@H
        self.P = A @ self.P @ A.T + K @ self.R @ K.T
        self.P = (self.P+self.P.T)/2
        nis = float(innovation @ np.linalg.solve(S, innovation))
        self.nis.append(nis)
        return nis
