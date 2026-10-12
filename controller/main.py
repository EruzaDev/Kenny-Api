"""Resident policy, continuous SLAM-map refinement, and cloud mission control.

Run with ROS sourced and the simulator's Python environment:
python -m controller.main --environment /path/environment.json --model /path/best.zip
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.error import URLError

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / 'Kenny---Automotive-self-driving-trash-can'))
from controller.mapping import AdaptiveMap, compose, inverse
from controller.missions import CloudAPI, Missions
from controller.planning import Planner
from controller.policy import ResidentPolicy, guard
from controller.sensors import lidar_features, depth_features
from controller.tracking import Tracker
from pi.receiver import save


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def make_node(args, policy, environment):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped, TransformStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as Route
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Bool, String, Float32MultiArray
    from visualization_msgs.msg import MarkerArray
    from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException

    class MainController(Node):
        def __init__(self):
            super().__init__('kenny_main_controller')
            self.mutex = threading.RLock()
            self.mapping = AdaptiveMap(environment)
            self.policy = policy
            self.alignment = None
            self.initial_pose = args.initial_pose or [*environment['start'], 0.]
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.broadcaster = StaticTransformBroadcaster(self)
            self.pose = None
            self.pose_time = self.odom_time = self.map_time = -math.inf
            self.pose_observed_at = None
            self.velocity = [0., 0.]
            self.uncertainty = math.inf
            self.marker_time = -math.inf
            self.sensors = {}
            self.goal_job = None
            self.goal = None
            self.route = []
            self.last_plan = -math.inf
            self.planned_revision = -1
            self.planned_goal = None
            self.status = 'Model loaded; waiting for live SLAM, pose, and sensors'
            self.guard_reason = ''
            self.odom_sequence = 0
            self.last_map = None
            self.plan_event = threading.Event()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pub = self.create_publisher(OccupancyGrid, '/kenny/map', qos)
            self.command_pub = self.create_publisher(TwistStamped, '/kenny/cmd_vel', 10)
            self.allow_pub = self.create_publisher(Bool, '/kenny/motion_allowed', 10)
            self.status_pub = self.create_publisher(String, '/kenny/controller/status', 10)
            self.route_pub = self.create_publisher(Route, '/kenny/route', 1)
            self.create_subscription(OccupancyGrid, args.slam_topic, self.on_map, qos)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, args.depth_topic, self.on_depth, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, args.slam_pose_topic, self.on_slam_pose, 10)
            self.create_subscription(Float32MultiArray, '/kenny/downward', self.on_downward, qos_profile_sensor_data)
            self.create_subscription(MarkerArray, '/kenny/marker_observations', self.on_markers, 10)
            self.create_timer(policy.config.dt, self.control)
            self.create_timer(.1, self.update_pose)
            self.create_timer(1., self.publish_status)
            self.create_timer(10., self.persist)
            self.create_timer(1., self.plan_event.set)

        def fresh_stamp(self, stamp, limit=.5):
            age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(stamp)
            return -.1 <= age <= limit

        def sensor_transform(self, frame, stamp):
            tf = self.buffer.lookup_transform(args.base_frame, frame, rclpy.time.Time.from_msg(stamp), timeout=rclpy.duration.Duration(seconds=.05))
            p, q = tf.transform.translation, tf.transform.rotation
            return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]

        def update_pose(self):
            try:
                tf = self.buffer.lookup_transform(args.slam_frame, args.base_frame, rclpy.time.Time())
                if not self.fresh_stamp(tf.header.stamp, .5):
                    return
                p = tf.transform.translation
                slam_pose = [p.x, p.y, yaw(tf.transform.rotation)]
                with self.mutex:
                    if self.alignment is None:
                        # Align the first real SLAM pose to the supplied approximate
                        # starting pose, without seeding SLAM with invented scans.
                        self.alignment = compose(self.initial_pose, inverse(slam_pose))
                        msg = TransformStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id, msg.child_frame_id = 'map', args.slam_frame
                        msg.transform.translation.x, msg.transform.translation.y = self.alignment[:2]
                        msg.transform.rotation.z = math.sin(self.alignment[2] / 2)
                        msg.transform.rotation.w = math.cos(self.alignment[2] / 2)
                        self.broadcaster.sendTransform(msg)
                    self.pose = compose(self.alignment, slam_pose)
                    self.pose_time = time.monotonic()
                    self.pose_observed_at = stamp_seconds(tf.header.stamp)
                    pending = self.last_map
                if pending is not None and self.map_time == -math.inf:
                    self.on_map(pending)
            except TransformException:
                return

        def on_map(self, msg):
            if msg.header.frame_id != args.slam_frame or not self.fresh_stamp(msg.header.stamp, 3.):
                return
            with self.mutex:
                self.last_map = msg
                if self.alignment is None:
                    return
                p = msg.info.origin.position
                grid = {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                        'origin': [p.x, p.y, yaw(msg.info.origin.orientation)], 'data': list(msg.data)}
                try:
                    self.mapping.update_slam(grid, self.alignment)
                except ValueError as error:
                    self.get_logger().error(str(error))
                    return
                self.map_time = time.monotonic()
                self.plan_event.set()
                grid = self.mapping.grid
                result = OccupancyGrid()
                result.header.frame_id = 'map'
                result.header.stamp = self.get_clock().now().to_msg()
                result.info.resolution = grid['resolution']
                result.info.width, result.info.height = grid['width'], grid['height']
                result.info.origin.position.x, result.info.origin.position.y = grid['origin'][:2]
                result.info.origin.orientation.w = 1.
                result.data = grid['data']
                self.map_pub.publish(result)

        def on_scan(self, msg):
            if not self.fresh_stamp(msg.header.stamp):
                return
            try:
                translation, quaternion = self.sensor_transform(msg.header.frame_id, msg.header.stamp)
                packet = lidar_features(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                        msg.range_max, translation, quaternion, self.policy.robot.lidar_range)
            except (TransformException, ValueError):
                return
            with self.mutex:
                self.sensors['lidar'] = (*packet, time.monotonic())

        def on_depth(self, msg):
            if not self.fresh_stamp(msg.header.stamp):
                return
            try:
                translation, quaternion = self.sensor_transform(msg.header.frame_id, msg.header.stamp)
                points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
                depth, floor = depth_features(points, translation, quaternion, self.policy.robot)
            except (TransformException, ValueError, AssertionError):
                return
            with self.mutex:
                now = time.monotonic()
                self.sensors['depth'], self.sensors['floor'] = (*depth, now), (*floor, now)

        def on_downward(self, msg):
            # [hazard_left, hazard_center, hazard_right, valid_left, valid_center, valid_right].
            data = np.asarray(msg.data)
            if data.shape != (6,) or not np.isfinite(data).all() or not np.isin(data, [0, 1]).all():
                return
            with self.mutex:
                self.sensors['downward'] = (data[:3].astype(bool), data[3:].astype(bool), time.monotonic())

        def on_odom(self, msg):
            if msg.header.frame_id != 'odom' or msg.child_frame_id != args.base_frame or not self.fresh_stamp(msg.header.stamp):
                return
            v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            if not np.isfinite([v, w]).all():
                return
            with self.mutex:
                self.velocity, self.odom_time = [v, w], time.monotonic()
                self.odom_sequence += 1

        def on_slam_pose(self, msg):
            if msg.header.frame_id != args.slam_frame or not self.fresh_stamp(msg.header.stamp):
                return
            covariance = np.asarray(msg.pose.covariance)
            if not np.isfinite(covariance).all() or min(covariance[0], covariance[7]) < 0:
                return
            with self.mutex:
                self.uncertainty = max(.02, math.sqrt(max(covariance[0], covariance[7])))

        def on_markers(self, msg):
            with self.mutex:
                if self.alignment is None:
                    return
                for marker in msg.markers:
                    if marker.header.frame_id != args.slam_frame or not self.fresh_stamp(marker.header.stamp):
                        continue
                    p = marker.pose.position
                    if self.mapping.observe_marker(marker.id, [p.x, p.y, yaw(marker.pose.orientation)], stamp_seconds(marker.header.stamp)):
                        self.marker_time = time.monotonic()

        def snapshot(self):
            with self.mutex:
                now = time.monotonic()
                reason = ''
                if self.pose is None or now - self.pose_time > .5:
                    reason = 'SLAM pose stale/unavailable'
                elif now - self.map_time > 3.:
                    reason = 'SLAM map stale/unavailable'
                elif now - self.odom_time > .5:
                    reason = 'measured odometry stale/unavailable'
                elif self.uncertainty > .35:
                    reason = 'SLAM pose covariance unavailable/high'
                else:
                    _, reason = guard([0., 0.], self.sensors, now, uncertainty=self.uncertainty)
                return {'ready': not reason, 'reason': reason, 'pose': self.pose,
                        'velocity': list(self.velocity), 'odom_sequence': self.odom_sequence}

        def destination(self, job):
            with self.mutex:
                return self.mapping.destination(job)

        def tracking_packet(self):
            with self.mutex:
                if self.pose is None or time.monotonic() - self.pose_time > .5:
                    return None
                route = self.route
                if len(route) > 200:
                    route = [route[i] for i in np.linspace(0, len(route) - 1, 200).astype(int)]
                return {'pose': {'frame_id': 'map', 'x': self.pose[0], 'y': self.pose[1], 'yaw': self.pose[2]},
                        'observed_at': self.pose_observed_at, 'source': 'physical',
                        'uncertainty_m': self.uncertainty if math.isfinite(self.uncertainty) else None,
                        'status': 'navigating' if self.goal_job and not self.guard_reason else 'waiting' if self.goal_job else 'idle',
                        'detail': (self.guard_reason or self.status)[:256],
                        'goal': {'x': self.goal[0], 'y': self.goal[1], 'marker_id': self.goal_job['marker_id']} if self.goal_job and self.goal else None,
                        'job_id': self.goal_job.get('id') if self.goal_job else None, 'route': route}

        def set_goal(self, job):
            with self.mutex:
                self.goal_job = job
                self.goal = self.mapping.destination(job)
                self.route = []
                self.last_plan = -math.inf
                self.policy.destination_changed()
                self.plan_event.set()

        def plan_once(self):
            # Route search can be slower than one control tick on a large map.
            # Keep it off the ROS sensor/inference callback and outside the lock.
            with self.mutex:
                if self.goal_job is None or self.pose is None:
                    return
                job_id = self.goal_job['id'] if 'id' in self.goal_job else self.goal_job['marker_id']
                goal = self.mapping.destination(self.goal_job)
                pose, grid, revision = list(self.pose), self.mapping.grid, self.mapping.revision
            route = Planner(grid, self.policy.robot.radius).path(pose, goal)
            with self.mutex:
                if self.goal_job is None:
                    return
                current_id = self.goal_job.get('id', self.goal_job['marker_id'])
                if current_id != job_id or self.mapping.destination(self.goal_job) != goal:
                    self.plan_event.set()
                    return
                self.route = route
                self.last_plan, self.planned_revision, self.planned_goal = time.monotonic(), revision, goal
                self.publish_route()

        def publish_command(self, command, allowed):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = args.base_frame
            msg.twist.linear.x, msg.twist.angular.z = float(command[0]), float(command[1])
            self.command_pub.publish(msg)
            self.allow_pub.publish(Bool(data=allowed))
            self.policy.record_executed(command)

        def control(self):
            with self.mutex:
                now = time.monotonic()
                snapshot = self.snapshot()
                command, allowed = [0., 0.], False
                if self.goal_job is not None and snapshot['ready']:
                    self.goal = self.mapping.destination(self.goal_job)
                    if now - self.last_plan > 1. or self.planned_revision != self.mapping.revision or self.planned_goal != self.goal:
                        self.plan_event.set()
                    if self.planned_goal != self.goal:
                        self.route = []
                    if self.route:
                        grid = self.mapping.grid
                        command = self.policy.predict(self.sensors, self.pose, self.goal, self.route,
                            [grid['width'] * grid['resolution'], grid['height'] * grid['resolution']],
                            self.velocity, self.uncertainty, now - self.marker_time, now, bool(self.guard_reason))
                        distance = math.dist(self.pose[:2], self.goal[:2])
                        if distance <= .25:
                            angle = math.atan2(math.sin(self.goal[2] - self.pose[2]), math.cos(self.goal[2] - self.pose[2]))
                            command = [0., max(-.3, min(.3, angle)) if abs(angle) > .25 else 0.]
                        command, self.guard_reason = guard(command, self.sensors, now, self.policy.robot.radius,
                                                          args.max_speed, uncertainty=self.uncertainty)
                        allowed = not self.guard_reason
                    else:
                        self.guard_reason = 'No route on the current refined map'
                else:
                    self.guard_reason = snapshot['reason'] if self.goal_job else ''
                self.publish_command(command, allowed)

        def publish_route(self):
            from geometry_msgs.msg import PoseStamped
            message = Route()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            for x, y in self.route:
                point = PoseStamped()
                point.header = message.header
                point.pose.position.x, point.pose.position.y = x, y
                point.pose.orientation.w = 1.
                message.poses.append(point)
            self.route_pub.publish(message)

        def arrived(self):
            with self.mutex:
                if self.goal is None or self.pose is None:
                    return False
                angle = abs(math.atan2(math.sin(self.goal[2] - self.pose[2]), math.cos(self.goal[2] - self.pose[2])))
                return math.dist(self.pose[:2], self.goal[:2]) <= .25 and angle <= .25

        def stop(self):
            with self.mutex:
                self.goal = self.goal_job = None
                self.route = []
                self.publish_command([0., 0.], False)
            deadline = time.monotonic() + 5.
            confirmed, last_sequence = 0, -1
            while time.monotonic() < deadline:
                sample = self.snapshot()
                if sample['odom_sequence'] != last_sequence and time.monotonic() - self.odom_time <= .5:
                    last_sequence = sample['odom_sequence']
                    confirmed = confirmed + 1 if abs(sample['velocity'][0]) < .01 and abs(sample['velocity'][1]) < .02 else 0
                    if confirmed >= 3:
                        return
                time.sleep(.05)
            raise RuntimeError('Physical stopping not confirmed by fresh odometry; API trip remains blocked')

        def persist(self):
            with self.mutex:
                if self.map_time == -math.inf:
                    return
                result = self.mapping.export(self.pose)
            try:
                args.refined.parent.mkdir(parents=True, exist_ok=True)
                save(args.refined, result)
            except OSError as error:
                self.get_logger().error(f'Cannot save refined map: {error}')

        def publish_status(self):
            with self.mutex:
                snapshot = self.snapshot()
                message = {**snapshot, 'status': self.status, 'guard': self.guard_reason,
                           'goal': self.goal, 'marker_id': self.goal_job['marker_id'] if self.goal_job else None,
                           'map_revision': self.mapping.revision, 'observed_cells': int(self.mapping.observed.sum()),
                           'model_resident': True}
                self.status_pub.publish(String(data=json.dumps(message, allow_nan=False)))

    return MainController()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--api', default=os.environ.get('KENNY_API_URL', 'https://kenny-api.onrender.com'))
    parser.add_argument('--worker', default='kenny-main')
    parser.add_argument('--state', type=Path, default=ROOT / 'data/main-controller.json')
    parser.add_argument('--refined', type=Path, default=ROOT / 'data/refined-environment.json')
    parser.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--slam-topic', default='/map')
    parser.add_argument('--slam-frame', default='slam_map')
    parser.add_argument('--slam-pose-topic', default='/pose')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--depth-topic', default='/camera/depth/points')
    parser.add_argument('--switch-margin', type=float, default=.25)
    parser.add_argument('--trip-timeout', type=float, default=1800.)
    parser.add_argument('--max-speed', type=float, default=.1)
    args, ros_args = parser.parse_known_args()
    if not 0 < args.max_speed <= .1 or args.switch_margin < 0 or args.trip_timeout <= 0 or not np.isfinite([args.max_speed, args.switch_margin, args.trip_timeout]).all():
        parser.error('Use finite positive speed ≤0.1 m/s and timeout, and a nonnegative switch margin')
    if args.slam_frame == 'map':
        parser.error('SLAM frame must differ from the estimate map frame (use slam_map)')
    if args.initial_pose and not np.isfinite(args.initial_pose).all():
        parser.error('Initial pose must be finite')
    environment = json.loads(args.environment.read_text())
    policy = ResidentPolicy(args.model)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=ros_args)
    node = make_node(args, policy, environment)
    api = CloudAPI(args.api, os.environ.get('KENNY_PI_TOKEN') or os.environ.get('KENNY_DISPATCH_TOKEN', ''))
    missions = Missions(api, node, args.state, args.worker, args.switch_margin, args.trip_timeout)
    stopped = threading.Event()

    def receive():
        while rclpy.ok() and not stopped.is_set():
            try:
                node.status = missions.step()
            except (URLError, TimeoutError, ConnectionError) as error:
                node.status = f'API unavailable; retrying: {error}'
            except Exception as error:
                node.status = f'Mission controller stopped: {error}'
                node.get_logger().error(node.status)
                with node.mutex:
                    node.goal = node.goal_job = None
                    node.publish_command([0., 0.], False)
                return
            stopped.wait(1.)

    thread = threading.Thread(target=receive, daemon=True)
    thread.start()

    def plan():
        while rclpy.ok() and not stopped.is_set():
            node.plan_event.wait(1.)
            node.plan_event.clear()
            try:
                node.plan_once()
            except Exception as error:
                node.get_logger().error(f'Route planning failed: {error}')
                with node.mutex:
                    node.route = []

    planner_thread = threading.Thread(target=plan, daemon=True)
    planner_thread.start()
    tracker = Tracker(api, missions.worker)

    def track():
        while rclpy.ok() and not stopped.is_set():
            try:
                tracker.publish(node.tracking_packet())
            except Exception as error:
                node.get_logger().warning(f'Live position upload unavailable: {error}')
            stopped.wait(1.)

    tracking_thread = threading.Thread(target=track, daemon=True)
    tracking_thread.start()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        node.plan_event.set()
        with node.mutex:
            node.goal = node.goal_job = None
            node.publish_command([0., 0.], False)
        node.persist()
        executor.shutdown()
        thread.join(timeout=16.)
        planner_thread.join(timeout=5.)
        tracking_thread.join(timeout=16.)
        missions.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
