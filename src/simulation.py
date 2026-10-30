"""Reproducible simulation. Ground truth is used only by sensor generation/evaluation."""
from dataclasses import dataclass, asdict
import numpy as np
from ekf_slam import EKFSLAM, motion, wrap

LANDMARKS = np.array([[-2., 0.], [2., 0.], [5., 1.], [7., 5.], [5., 9.],
                      [2., 10.], [-2., 10.], [-5., 9.], [-7., 5.], [-5., 1.]])

# Recorded association outcomes. A revisit is a successful known-ID correction
# after at least 10 simulated seconds without observing that landmark.
NO_OBSERVATION, NEW, UPDATED, REVISITED, IGNORED = range(5)
REVISIT_GAP_S = 10.

@dataclass
class Config:
    name: str = 'baseline'
    dt: float = 0.1
    steps: int = 1050
    track: float = 0.4
    speed: float = 0.6
    yaw_rate: float = 0.12
    wheel_std: float = 0.006
    range_std: float = 0.1
    bearing_std: float = 0.02
    max_range: float = 4.5
    sensor_stride: int = 5
    dropout_start: float = -1.
    dropout_end: float = -1.
    reuse_landmarks: bool = True
    assumed_noise_scale: float = 1.


def scenarios():
    return [Config(), Config(name='high_sensor_noise', range_std=0.3, bearing_std=0.06),
            Config(name='high_odometry_noise', wheel_std=0.012),
            Config(name='sensor_dropout', dropout_start=35., dropout_end=55.),
            Config(name='no_landmark_reuse', reuse_landmarks=False),
            Config(name='underestimated_noise', assumed_noise_scale=0.35)]


def run(config=None, seed=7):
    c = config or Config()
    # Separate streams keep wheel disturbances paired across sensor scenarios.
    wheel_rng = np.random.default_rng(seed)
    sensor_rng = np.random.default_rng(seed+100000)
    wheel_noise = wheel_rng.standard_normal((c.steps, 2))*c.wheel_std
    sensor_noise = sensor_rng.standard_normal((c.steps, len(LANDMARKS), 2))
    Q = np.eye(2)*(c.wheel_std*c.assumed_noise_scale)**2
    R = np.diag(np.array([c.range_std, c.bearing_std])**2)*c.assumed_noise_scale**2
    ekf = EKFSLAM(Q, R, c.track)
    truth, odom = np.zeros(3), np.zeros(3)
    hist = {key: [] for key in ['truth', 'odom', 'estimate', 'sigma', 'landmark_rmse', 'seen', 'nis',
                               'landmark_estimates', 'measurements', 'association',
                               'prior_estimate', 'updates', 'loop_closures']}
    last_seen = {}
    loop_closures = 0
    wheels = c.dt*np.array([c.speed-c.yaw_rate*c.track/2, c.speed+c.yaw_rate*c.track/2])
    for k in range(c.steps):
        truth = motion(truth, wheels, c.track)[0]
        noisy_wheels = wheels+wheel_noise[k]
        odom = motion(odom, noisy_wheels, c.track)[0]
        ekf.predict(noisy_wheels)
        prior_estimate = ekf.x[:3].copy()
        t = (k+1)*c.dt
        measurements = np.full((len(LANDMARKS), 2), np.nan)
        association = np.zeros(len(LANDMARKS), dtype=np.int8)
        if k % c.sensor_stride == 0 and not c.dropout_start <= t <= c.dropout_end:
            for identity, landmark in enumerate(LANDMARKS):
                delta = landmark-truth[:2]
                r = np.linalg.norm(delta)
                if r <= c.max_range:
                    z = np.array([r, wrap(np.arctan2(delta[1], delta[0])-truth[2])])
                    z += sensor_noise[k, identity]*[c.range_std, c.bearing_std]
                    z[1] = wrap(z[1])
                    if z[0] <= 0:  # impossible negative-range sample is discarded
                        continue
                    measurements[identity] = z
                    is_new = identity not in ekf.ids
                    if is_new:
                        association[identity] = NEW
                    elif not c.reuse_landmarks:
                        association[identity] = IGNORED
                    elif t-last_seen[identity] >= REVISIT_GAP_S:
                        association[identity] = REVISITED
                    else:
                        association[identity] = UPDATED
                    if c.reuse_landmarks or is_new:
                        ekf.correct(identity, z)
                    last_seen[identity] = t
        # Several returning IDs in one sensor scan form one revisit event.
        loop_closures += int(np.any(association == REVISITED))
        mapped = np.full((len(LANDMARKS), 2), np.nan)
        for identity, index in ekf.ids.items():
            mapped[identity] = ekf.x[index:index+2]
        errors = [np.sum((ekf.x[index:index+2]-LANDMARKS[identity])**2)
                  for identity, index in ekf.ids.items()]
        hist['truth'].append(truth.copy())
        hist['odom'].append(odom.copy())
        hist['estimate'].append(ekf.x[:3].copy())
        hist['sigma'].append(np.sqrt(np.maximum(np.diag(ekf.P)[:3], 0)))
        hist['landmark_rmse'].append(np.sqrt(np.mean(errors)) if errors else np.nan)
        hist['seen'].append(len(ekf.ids))
        hist['nis'].append(ekf.nis[-1] if ekf.nis else np.nan)
        hist['landmark_estimates'].append(mapped)
        hist['measurements'].append(measurements)
        hist['association'].append(association)
        hist['prior_estimate'].append(prior_estimate)
        hist['updates'].append(len(ekf.nis))
        hist['loop_closures'].append(loop_closures)
    hist = {key: np.array(value) for key, value in hist.items()}
    hist['time'] = np.arange(1, c.steps+1)*c.dt
    position_error = np.linalg.norm(hist['estimate'][:, :2]-hist['truth'][:, :2], axis=1)
    odom_error = np.linalg.norm(hist['odom'][:, :2]-hist['truth'][:, :2], axis=1)
    heading_error = wrap(hist['estimate'][:, 2]-hist['truth'][:, 2])
    metrics = dict(scenario=c.name, seed=seed,
                   position_rmse_m=float(np.sqrt(np.mean(position_error**2))),
                   odometry_rmse_m=float(np.sqrt(np.mean(odom_error**2))),
                   heading_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(heading_error**2)))),
                   final_position_error_m=float(position_error[-1]),
                   final_map_rmse_m=float(hist['landmark_rmse'][-1]),
                   landmarks=len(ekf.ids), updates=len(ekf.nis),
                   mean_nis=float(np.mean(ekf.nis)) if ekf.nis else None,
                   minimum_covariance_eigenvalue=float(np.linalg.eigvalsh(ekf.P).min()))
    return hist, ekf, metrics, asdict(c)
