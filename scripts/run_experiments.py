#!/usr/bin/env python3
"""Run paired Monte Carlo trials, save all numerical results and plots."""
import sys, json, csv, argparse, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, PillowWriter
from simulation import run, scenarios, LANDMARKS
from ekf_slam import wrap


def save(fig, name):
    fig.tight_layout()
    fig.savefig(ROOT/'results'/f'{name}.png', dpi=180)
    fig.savefig(ROOT/'results'/f'{name}.pdf')
    plt.close(fig)


def ellipse(ax, mean, cov):
    values, vectors = np.linalg.eigh(cov)
    angle = np.rad2deg(np.arctan2(vectors[1, 1], vectors[0, 1]))
    widths = 2*np.sqrt(5.991*np.maximum(values, 0))
    ax.add_patch(Ellipse(mean, widths[1], widths[0], angle=angle,
                         fill=False, edgecolor='tab:blue', alpha=0.5))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--animate', action='store_true')
    args = parser.parse_args()
    if args.trials < 2:
        parser.error('Use at least two trials for sample standard deviations')
    out = ROOT/'results'
    out.mkdir(exist_ok=True)
    started = time.perf_counter()
    rows, summary, demos = [], [], {}
    configs = scenarios()
    for c in configs:
        for seed in range(args.trials):
            hist, ekf, metrics, config = run(c, seed)
            rows.append(metrics)
        hist, ekf, metrics, config = run(c, 7)
        demos[c.name] = (hist, ekf, metrics)
        np.savez_compressed(out/f'{c.name}_seed7.npz', **hist, state=ekf.x,
                            covariance=ekf.P, landmark_ids=np.array(list(ekf.ids)),
                            landmark_indices=np.array(list(ekf.ids.values())))
        group = [r for r in rows if r['scenario'] == c.name]
        entry = {'scenario': c.name, 'trials': args.trials, 'config': config}
        for key in ['position_rmse_m', 'odometry_rmse_m', 'heading_rmse_deg',
                    'final_position_error_m', 'final_map_rmse_m']:
            values = np.array([r[key] for r in group])
            entry[key+'_mean'] = float(values.mean())
            entry[key+'_std'] = float(values.std(ddof=1))
        entry['mean_nis'] = (float(np.mean([r['mean_nis'] for r in group]))
                             if group[0]['mean_nis'] is not None else None)
        summary.append(entry)
        print(f"{c.name}: pose RMSE {entry['position_rmse_m_mean']:.3f} m, "
              f"odometry {entry['odometry_rmse_m_mean']:.3f} m", flush=True)
    with (out/'metrics.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    payload = {'seed_policy': f'paired integer seeds 0 to {args.trials-1}; illustration seed 7',
               'runtime_seconds': time.perf_counter()-started, 'summary': summary,
               'illustration_metrics': [value[2] for value in demos.values()]}
    (out/'summary.json').write_text(json.dumps(payload, indent=2))
    h, ekf, _ = demos['baseline']
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(*h['truth'][:, :2].T, 'k-', label='Ground truth')
    ax.plot(*h['odom'][:, :2].T, '--', color='tab:orange', label='Odometry only')
    ax.plot(*h['estimate'][:, :2].T, color='tab:blue', label='EKF-SLAM')
    ax.scatter(*LANDMARKS.T, marker='*', s=110, c='k', label='True landmarks')
    mapped = np.array([ekf.x[ekf.ids[i]:ekf.ids[i]+2] for i in sorted(ekf.ids)])
    ax.scatter(*mapped.T, facecolors='none', edgecolors='tab:blue', label='Estimated landmarks')
    for identity, index in ekf.ids.items():
        ellipse(ax, ekf.x[index:index+2], ekf.P[index:index+2, index:index+2])
        ax.text(*LANDMARKS[identity]+0.15, str(identity), fontsize=8)
    ax.set(xlabel='x (m)', ylabel='y (m)', title='Two-loop trajectory and jointly estimated map (seed 7)')
    ax.axis('equal'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    save(fig, 'trajectory_map')
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i, ax in enumerate(axes):
        error = h['estimate'][:, i]-h['truth'][:, i]
        odom = h['odom'][:, i]-h['truth'][:, i]
        if i == 2:
            error, odom = wrap(error), wrap(odom)
        scale = 180/np.pi if i == 2 else 1
        ax.plot(h['time'], error*scale, label='EKF error')
        ax.plot(h['time'], odom*scale, alpha=.6, label='Odometry error')
        ax.fill_between(h['time'], -2*h['sigma'][:, i]*scale,
                        2*h['sigma'][:, i]*scale, alpha=.2, label='EKF ±2 marginal standard deviations')
        ax.set_ylabel(['x error (m)', 'y error (m)', 'Heading error (deg)'][i]); ax.grid(alpha=.25)
    axes[0].legend(fontsize=7); axes[-1].set_xlabel('Time (s)')
    save(fig, 'pose_errors')
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for name in ['baseline', 'sensor_dropout', 'no_landmark_reuse']:
        d = demos[name][0]
        axes[0].plot(d['time'], np.linalg.norm(d['estimate'][:, :2]-d['truth'][:, :2], axis=1), label=name)
        axes[1].plot(d['time'], d['landmark_rmse'], label=name)
    for ax in axes:
        ax.axvspan(35, 55, color='gray', alpha=.15, label='Dropout interval (dropout scenario only)')
        ax.grid(alpha=.25)
    axes[0].set_ylabel('Position error (m)'); axes[0].legend(fontsize=7)
    axes[1].set(xlabel='Time (s)', ylabel='Map RMSE, seen landmarks (m)')
    save(fig, 'revisit_dropout')
    fig, ax = plt.subplots(figsize=(9, 4.5))
    x = np.arange(len(summary))
    ax.bar(x-.18, [s['position_rmse_m_mean'] for s in summary], .36,
           yerr=[s['position_rmse_m_std'] for s in summary], capsize=3, label='EKF-SLAM')
    ax.bar(x+.18, [s['odometry_rmse_m_mean'] for s in summary], .36,
           yerr=[s['odometry_rmse_m_std'] for s in summary], capsize=3, label='Odometry')
    ax.set_xticks(x, [s['scenario'].replace('_', '\n') for s in summary], fontsize=8)
    ax.set(ylabel='Trajectory position RMSE (m)', title=f'{args.trials} paired trials per scenario; bars show mean ± sample SD')
    ax.legend(); ax.grid(axis='y', alpha=.25)
    save(fig, 'monte_carlo')
    if args.animate:
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(*LANDMARKS.T, marker='*', c='k', label='True landmarks')
        truth_line, = ax.plot([], [], 'k-', label='Truth')
        estimate_line, = ax.plot([], [], 'b-', label='EKF trajectory')
        odom_line, = ax.plot([], [], '--', color='tab:orange', label='Odometry')
        robot, = ax.plot([], [], 'bo')
        ax.set(xlim=(-8, 8), ylim=(-2, 12), xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal'); ax.legend(fontsize=8)
        def update(k):
            for line, key in [(truth_line, 'truth'), (estimate_line, 'estimate'), (odom_line, 'odom')]:
                line.set_data(*h[key][:k+1, :2].T)
            robot.set_data([h['estimate'][k, 0]], [h['estimate'][k, 1]])
            ax.set_title(f'EKF-SLAM trajectory, t = {h["time"][k]:.1f} s')
            return truth_line, estimate_line, odom_line, robot
        FuncAnimation(fig, update, frames=range(0, len(h['time']), 10), interval=80).save(
            out/'trajectory_demo.gif', writer=PillowWriter(fps=12))
        plt.close(fig)
    print(f'Artifacts saved in {out}')

if __name__ == '__main__':
    main()
