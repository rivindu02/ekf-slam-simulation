#!/usr/bin/env python3
"""ROS 2 Humble adapter: encoder odometry + explicitly synthetic landmark sensor.

Gazebo world poses are used only to synthesize ideal range/bearing measurements
and score estimates. Simulator robot pose also drives a separate demo route
controller. Ground-truth map coordinates are never passed to EKFSLAM.
No image detection, LiDAR extraction or unknown data association is claimed.
"""
import argparse, csv, json, sys, time, signal
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from ekf_slam import EKFSLAM, motion, wrap
from demo_control import route_command
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry, Path as RosPath
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


def yaw(q):
    return np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class SlamDemo(Node):
    def __init__(self, duration, output_dir):
        super().__init__('ekf_slam_demo')
        self.duration = duration
        self.saved = False
        self.output_dir = output_dir
        self.filter = EKFSLAM(np.eye(2)*.006**2, np.diag([.1**2, .02**2]))
        self.rng = np.random.default_rng(7)
        self.models = {}; self.truth = None
        self.previous = None; self.start = None; self.step = 0; self.done = False
        self.rows = []; self.odom = np.zeros(3)
        self.visible = []; self.observations = 0
        self.minimum_clearance = float("inf")
        self.last_status_time = -10.
        self.create_subscription(ModelStates, '/slam_demo/model_states', self.on_models, 10)
        self.create_subscription(Odometry, '/slam_demo/wheel_odom', self.on_odom, 10)
        self.command = self.create_publisher(Twist, '/slam_demo/cmd_vel', 10)
        self.path_pub = self.create_publisher(RosPath, '/slam_demo/estimated_path', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/slam_demo/estimated_landmarks', 10)
        self.path = RosPath(); self.path.header.frame_id = 'map'
        self.create_timer(.1, self.drive)
        self.get_logger().info('Synthetic range-bearing sensor with known IDs. RViz: green rays = observations; blue IDs = estimated map.')

    def drive(self):
        msg = Twist()
        if self.truth is not None and not self.done:
            obstacles = [(p.position.x, p.position.y) for n, p in self.models.items()
                         if n.startswith('landmark_')]
            v, w, nearest = route_command(self.truth, obstacles)
            self.minimum_clearance = min(self.minimum_clearance, nearest)
            msg.linear.x = v; msg.angular.z = w
        self.command.publish(msg)

    def on_models(self, msg):
        self.models = dict(zip(msg.name, msg.pose))
        pose = self.models.get('slam_robot')
        if pose:
            self.truth = np.array([pose.position.x, pose.position.y, yaw(pose.orientation)])

    def on_odom(self, msg):
        if self.truth is None or self.done:
            return
        t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.previous is None:
            self.previous = t; self.start = t
            # Initial pose anchors the coordinate frame before movement starts.
            self.filter.x[:3] = self.truth
            self.odom = self.truth.copy()
            return
        dt = t-self.previous
        self.previous = t
        if not 0 < dt < .5:
            return
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        wheels = dt*np.array([v-w*.2, v+w*.2])+self.rng.normal(0, .006, 2)
        self.filter.predict(wheels)
        self.odom = motion(self.odom, wheels, .4)[0]
        if self.step % 5 == 0:
            self.visible = []
            for name, pose in sorted(self.models.items()):
                if not name.startswith('landmark_'):
                    continue
                delta = np.array([pose.position.x, pose.position.y])-self.truth[:2]
                r = np.linalg.norm(delta)
                if .05 < r <= 4.5:
                    z = np.array([r, wrap(np.arctan2(delta[1], delta[0])-self.truth[2])])
                    z += self.rng.normal(0, [.1, .02])
                    z[1] = wrap(z[1])
                    if z[0] > 0:
                        identity = int(name.split('_')[1])
                        is_new = identity not in self.filter.ids
                        self.filter.correct(identity, z)
                        self.visible.append((identity, z.copy()))
                        self.observations += 1
                        if is_new:
                            self.get_logger().info(f'NEW landmark {identity}: range={z[0]:.2f} m, mapped={len(self.filter.ids)}/10')
        self.step += 1
        elapsed = t-self.start
        self.rows.append([elapsed, *self.truth, *self.odom, *self.filter.x[:3], len(self.filter.ids)])
        pose = PoseStamped(); pose.header = msg.header; pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = map(float, self.filter.x[:2])
        pose.pose.orientation.z = float(np.sin(self.filter.x[2]/2))
        pose.pose.orientation.w = float(np.cos(self.filter.x[2]/2))
        self.path.header = pose.header; self.path.poses.append(pose); self.path_pub.publish(self.path)
        if elapsed-self.last_status_time >= 2.:
            self.last_status_time = elapsed
            self.get_logger().info(f't={elapsed:.1f}s | visible IDs={[i for i, _ in self.visible]} | map={len(self.filter.ids)}/10 | observations={self.observations} | EKF updates={len(self.filter.nis)}')
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; markers.markers.append(clear)
        for identity, index in self.filter.ids.items():
            marker = Marker(); marker.header = pose.header; marker.id = identity
            marker.type = Marker.SPHERE; marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = map(float, self.filter.x[index:index+2])
            marker.pose.position.z = .75; marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .3
            marker.color.r = .1; marker.color.b = 1.; marker.color.a = 1.
            markers.markers.append(marker)
            label = Marker(); label.header = pose.header; label.ns = 'labels'; label.id = identity
            label.type = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD
            label.pose.position.x = marker.pose.position.x
            label.pose.position.y = marker.pose.position.y
            label.pose.position.z = 1.2; label.pose.orientation.w = 1.
            label.scale.z = .35; label.color.b = .8; label.color.a = 1.
            label.text = f'ID {identity}'
            markers.markers.append(label)
        rays = Marker(); rays.header = pose.header; rays.ns = 'detections'; rays.id = 0
        rays.type = Marker.LINE_LIST; rays.action = Marker.ADD
        rays.pose.orientation.w = 1.; rays.scale.x = .035
        rays.color.g = .8; rays.color.a = 1.
        for identity, z in self.visible:
            a = self.filter.x[2]+z[1]
            origin = Point(x=float(self.filter.x[0]), y=float(self.filter.x[1]), z=.35)
            endpoint = Point(x=float(origin.x+z[0]*np.cos(a)), y=float(origin.y+z[0]*np.sin(a)), z=.75)
            rays.points.extend([origin, endpoint])
        markers.markers.append(rays)
        robot = Marker(); robot.header = pose.header; robot.ns = 'robot'; robot.id = 0
        robot.type = Marker.ARROW; robot.action = Marker.ADD; robot.pose = pose.pose
        robot.pose.position.z = .25
        robot.scale.x = .7; robot.scale.y = .2; robot.scale.z = .2
        robot.color.b = 1.; robot.color.a = 1.; markers.markers.append(robot)
        status = Marker(); status.header = pose.header; status.ns = 'status'; status.id = 0
        status.type = Marker.TEXT_VIEW_FACING; status.action = Marker.ADD
        status.pose.position.y = -2.; status.pose.position.z = .5; status.pose.orientation.w = 1.
        status.scale.z = .4; status.color.r = .15; status.color.g = .15; status.color.b = .15; status.color.a = 1.
        status.text = f'Mapped {len(self.filter.ids)}/10 | Visible {len(self.visible)} | Updates {len(self.filter.nis)}'
        markers.markers.append(status)
        self.map_pub.publish(markers)
        if self.duration > 0 and elapsed >= self.duration:
            self.done = True; self.drive(); self.save()

    def save(self):
        if not self.rows or self.saved:
            return
        out = self.output_dir; out.mkdir(parents=True, exist_ok=True)
        with (out/'trajectory.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'true_x', 'true_y', 'true_yaw', 'odom_x', 'odom_y', 'odom_yaw',
                             'ekf_x', 'ekf_y', 'ekf_yaw', 'landmarks'])
            writer.writerows(self.rows)
        a = np.array(self.rows)
        map_errors = []
        mapped = {}
        for identity, index in self.filter.ids.items():
            xy = self.filter.x[index:index+2]
            pose = self.models['landmark_'+str(identity)]
            map_errors.append(np.sum((xy-[pose.position.x, pose.position.y])**2))
            mapped[str(identity)] = xy.tolist()
        metrics = {'duration_s': float(a[-1, 0]), 'samples': len(a),
                   'position_rmse_m': float(np.sqrt(np.mean(np.sum((a[:, 7:9]-a[:, 1:3])**2, axis=1)))),
                   'odometry_rmse_m': float(np.sqrt(np.mean(np.sum((a[:, 4:6]-a[:, 1:3])**2, axis=1)))),
                   'final_map_rmse_m': float(np.sqrt(np.mean(map_errors))) if map_errors else None,
                   'landmarks': len(mapped), 'landmark_estimates': mapped,
                   'observations': self.observations, 'ekf_updates': len(self.filter.nis),
                   'minimum_landmark_center_distance_m': self.minimum_clearance,
                   'controller': 'simulator-pose pure pursuit, radius 4 m, centre (0,5); safety stop at 0.8 m',
                   'sensor': 'synthetic range-bearing from Gazebo poses; known IDs; no occlusion',
                   'odometry': 'encoder-mode Gazebo plugin twist with added Gaussian wheel-travel noise'}
        (out/'metrics.json').write_text(json.dumps(metrics, indent=2))
        self.saved = True
        if rclpy.ok():
            self.get_logger().info(json.dumps(metrics))
        else:
            print(json.dumps(metrics), flush=True)


def main():
    p = argparse.ArgumentParser(); p.add_argument('--duration', type=float, default=105.)
    p.add_argument('--output-dir', type=Path, default=ROOT/'results'/'gazebo')
    args = p.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0:
        p.error('--duration must be nonnegative; 0 runs until Ctrl+C')
    # Keep the context alive until a stop command and data save are complete.
    # Humble's default SIGINT handler shuts it down before KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SlamDemo(args.duration, args.output_dir)
    stop_requested = False
    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True
    old_handlers = {s: signal.signal(s, request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
    print('Running until Ctrl+C.' if args.duration == 0 else
          f'Will stop automatically after {args.duration:g} simulated seconds.', flush=True)
    wall_start = time.monotonic()
    try:
        while rclpy.ok() and not node.done and not stop_requested:
            rclpy.spin_once(node, timeout_sec=.2)
            if node.previous is None and time.monotonic()-wall_start > 30:
                raise RuntimeError('No Gazebo odometry after 30 s; inspect results/gazebo/server.log')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.done = True
        try:
            if rclpy.ok():
                node.drive()
            node.save()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)

if __name__ == '__main__':
    main()
