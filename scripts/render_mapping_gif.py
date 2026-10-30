"""Render landmark discovery and revisits from a recorded simulation."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def render(recording, output):
    import sys
    sys.path.insert(0, str(ROOT/'src'))
    from simulation import LANDMARKS, NEW, UPDATED, REVISITED, IGNORED

    with np.load(recording) as archive:
        history = {key: archive[key] for key in archive.files}
    frames = sorted(set(range(0, len(history['time']), 5)) | {len(history['time'])-1})
    # Briefly hold returning-landmark scans so their labels can be read.
    frames = [k for k in frames for _ in range(
        4 if np.any(history['association'][k] == REVISITED) else 1)]
    frames += [frames[-1]]*12
    colors = {NEW: '#ca7b18', UPDATED: '#178269', REVISITED: '#893bb2', IGNORED: '#78848c'}
    fig, ax = plt.subplots(figsize=(7.2, 6.4), dpi=100)
    fig.patch.set_facecolor('#f6f8fb')
    ax.set_facecolor('#ffffff')
    ax.set(xlim=(-8.5, 8.5), ylim=(-2, 12), xlabel='x (m)', ylabel='y (m)')
    ax.set_aspect('equal')
    ax.grid(alpha=.18)
    ax.scatter(*LANDMARKS.T, marker='*', s=90, color='#57636e', label='True landmarks')
    for identity, point in enumerate(LANDMARKS):
        ax.annotate(str(identity), point, xytext=(6, 5), textcoords='offset points', fontsize=8)
    truth_line, = ax.plot([], [], '--', color='#727d87', lw=1.3, label='True path')
    estimate_line, = ax.plot([], [], color='#2563b5', lw=2, label='EKF path')
    mapped = ax.scatter([], [], marker='x', s=75, color='#2563b5', linewidths=2,
                        label='Estimated landmarks', zorder=4)
    active = ax.scatter([], [], s=210, facecolors='none', linewidths=2, zorder=5)
    robot, = ax.plot([], [], 'o', color='#2563b5', ms=7, zorder=6)
    heading, = ax.plot([], [], color='#2563b5', lw=2, zorder=6)
    rays = LineCollection([], linewidths=1, alpha=.65, zorder=2)
    ax.add_collection(rays)
    title = ax.set_title('Landmark mapping and revisits', fontsize=14, pad=15)
    status = fig.text(.5, .055, '', ha='center', fontsize=10)
    fig.text(.5, .025, 'Orange: new ID    Green: matched ID    Purple: revisited ID',
             ha='center', fontsize=9, color='#526170')
    ax.legend(loc='upper right', fontsize=8, framealpha=.95)
    fig.subplots_adjust(left=.1, right=.97, bottom=.15, top=.9)

    def update(k):
        truth_line.set_data(*history['truth'][:k+1, :2].T)
        estimate_line.set_data(*history['estimate'][:k+1, :2].T)
        points = history['landmark_estimates'][k]
        present = np.isfinite(points).all(axis=1)
        mapped.set_offsets(points[present])
        outcomes = history['association'][k]
        visible = outcomes != 0
        active.set_offsets(points[visible])
        active.set_edgecolors([colors[int(s)] for s in outcomes[visible]])
        pose = history['estimate'][k]
        robot.set_data([pose[0]], [pose[1]])
        front = pose[:2]+.45*np.array([np.cos(pose[2]), np.sin(pose[2])])
        heading.set_data([pose[0], front[0]], [pose[1], front[1]])
        measurements = history['measurements'][k, visible]
        endpoints = pose[:2]+measurements[:, :1]*np.column_stack((
            np.cos(pose[2]+measurements[:, 1]), np.sin(pose[2]+measurements[:, 1])))
        rays.set_segments([[pose[:2], point] for point in endpoints])
        rays.set_colors([colors[int(s)] for s in outcomes[visible]])
        returning = np.flatnonzero(outcomes == REVISITED)
        title.set_text(f'Landmark mapping | t = {history["time"][k]:.1f} s')
        if len(returning):
            status.set_text('Loop-closure revisit: ID '+', '.join(map(str, returning)))
            status.set_color(colors[REVISITED])
        else:
            status.set_text(f'{history["seen"][k]} landmarks | '
                            f'{history["updates"][k]} corrections | '
                            f'{history["loop_closures"][k]} revisit scans')
            status.set_color('#526170')

    animation = FuncAnimation(fig, update, frames=frames, interval=80, repeat=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=12))
    plt.close(fig)
    print(f'Rendered {output} from {recording.name}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, default=ROOT/'results/baseline_seed7.npz')
    parser.add_argument('--output', type=Path, default=ROOT/'results/landmark_mapping.gif')
    args = parser.parse_args()
    render(args.recording, args.output)
