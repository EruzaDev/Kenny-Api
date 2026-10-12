"""ROS adapter integration with synthetic inputs and no motor connection."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from controller.main import make_node
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'Source ROS Jazzy to test the ROS adapter')
class ROSControllerTests(unittest.TestCase):
    def test_live_pose_map_goal_and_command_share_the_persistent_controller(self):
        from test_controller import environment
        from kenny_rl.config import RobotConfig, EnvConfig
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from geometry_msgs.msg import TwistStamped

        class Policy:
            config = EnvConfig()
            robot = RobotConfig()
            model = object()
            changes = 0

            def destination_changed(self):
                self.changes += 1

            def predict(self, *args):
                return [.05, 0.]

            def record_executed(self, command):
                self.command = command

        with tempfile.TemporaryDirectory() as directory:
            args = argparse.Namespace(initial_pose=[1., 1., 0.], base_frame='base_link', slam_frame='slam_map',
                slam_topic='/test/slam_map', slam_pose_topic='/test/slam_pose', depth_topic='/test/depth',
                refined=Path(directory) / 'refined.json', max_speed=.1)
            # Isolate synthetic velocity/map topics from the robot's ROS domain.
            rclpy.init(domain_id=87)
            policy = Policy()
            node = make_node(args, policy, environment())
            receiver = rclpy.create_node('kenny_controller_test_listener')
            maps, commands = [], []
            receiver.create_subscription(OccupancyGrid, '/kenny/map', maps.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            receiver.create_subscription(TwistStamped, '/kenny/cmd_vel', commands.append, 10)
            try:
                stamp = node.get_clock().now().to_msg()
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'slam_map', 'base_link'
                tf.transform.rotation.w = 1.
                node.buffer.set_transform(tf, 'test')
                node.update_pose()
                np.testing.assert_allclose(node.pose, [1., 1., 0.])
                grid = OccupancyGrid()
                grid.header.stamp, grid.header.frame_id = stamp, 'slam_map'
                grid.info.width = grid.info.height = 12
                grid.info.resolution = .5
                grid.info.origin.orientation.w = 1.
                grid.data = [0] * 144
                node.on_map(grid)
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
                node.on_odom(odom)
                pose = PoseWithCovarianceStamped()
                pose.header.stamp, pose.header.frame_id = stamp, 'slam_map'
                pose.pose.covariance[0] = pose.pose.covariance[7] = .0025
                node.on_slam_pose(pose)
                now = time.monotonic()
                node.sensors = {name: (np.full(size, distance), np.ones(size, dtype=bool), now)
                    for name, size, distance in [('lidar', 72, 6.), ('depth', 36, 4.), ('floor', 12, 4.), ('downward', 3, 0.)]}
                self.assertTrue(node.snapshot()['ready'])
                model = policy.model
                node.set_goal({'marker_id': 1, 'dictionary': 'DICT_4X4_50'})
                node.plan_once()
                node.control()
                self.assertEqual(policy.command, [.05, 0.])
                node.set_goal({'marker_id': 2, 'dictionary': 'DICT_4X4_50'})
                self.assertIs(policy.model, model)
                self.assertEqual(policy.changes, 2)
                node.pose_time = -np.inf
                node.control()
                self.assertEqual(policy.command, [0., 0.])
                node.persist()
                result = json.loads(args.refined.read_text())
                self.assertGreater(result['mapping']['observed_cells'], 0)
                for _ in range(10):
                    rclpy.spin_once(receiver, timeout_sec=.05)
                self.assertTrue(maps)
                self.assertTrue(commands)
                self.assertEqual(commands[-1].twist.linear.x, 0.)
            finally:
                receiver.destroy_node()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
