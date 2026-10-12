"""Live evidence, pose transforms, checkpoint features, and preemption semantics."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / 'Kenny---Automotive-self-driving-trash-can'))
from app.queue import Queue, DispatchError
from controller.mapping import AdaptiveMap, compose, inverse
from controller.missions import Missions
from controller.motors import WheelOdometry
from controller.planning import Planner, nearest
from controller.policy import ResidentPolicy, guard
from controller.sensors import lidar_features


def environment():
    return {'version': 1, 'frame_id': 'map', 'units': 'meters', 'dictionary': 'DICT_4X4_50',
            'grid': {'width': 12, 'height': 12, 'resolution': .5, 'origin': [0., 0., 0.], 'data': [0] * 144},
            'start': [1., 1.], 'goal': [4., 4.], 'obstacles': [],
            'markers': [{'id': mid, 'x': x, 'y': y, 'z': 0., 'yaw': 0., 'size': .1, 'mount': 'floor'}
                        for mid, x, y in [(1, 4., 4.), (2, 2., 1.)]]}


class MapTests(unittest.TestCase):
    def test_real_observations_replace_estimated_walls_and_preserve_unseen_prior(self):
        env = environment()
        env['grid']['data'][2 * 12 + 2] = 100
        env['grid']['data'][3 * 12 + 3] = 100
        mapping = AdaptiveMap(env)
        live = copy.deepcopy(env['grid'])
        live['data'] = [-1] * 144
        live['data'][2 * 12 + 2] = 0
        live['data'][5 * 12 + 5] = 100
        mapping.update_slam(live, [0, 0, 0])
        self.assertEqual(mapping.grid['data'][2 * 12 + 2], 0)
        self.assertEqual(mapping.grid['data'][3 * 12 + 3], 100)
        self.assertEqual(mapping.grid['data'][5 * 12 + 5], 100)
        self.assertEqual(int(mapping.observed.sum()), 2)

    def test_loop_closure_replaces_old_evidence_and_no_go_cannot_be_erased(self):
        env = environment()
        env['obstacles'] = [{'kind': 'prohibited', 'x': 1., 'y': 1., 'width': .5, 'depth': .5, 'bottom': 0., 'height': 1.}]
        mapping = AdaptiveMap(env)
        live = copy.deepcopy(env['grid'])
        live['data'][6 * 12 + 6] = 100
        mapping.update_slam(live, [0, 0, 0])
        live['data'][6 * 12 + 6] = -1
        live['data'][7 * 12 + 7] = 100
        mapping.update_slam(live, [0, 0, 0])
        self.assertEqual(mapping.grid['data'][6 * 12 + 6], 0)
        self.assertEqual(mapping.grid['data'][7 * 12 + 7], 100)
        self.assertEqual(mapping.grid['data'][2 * 12 + 2], 100)

    def test_alignment_rotated_origin_and_map_growth(self):
        mapping = AdaptiveMap(environment())
        live = {'width': 2, 'height': 2, 'resolution': .5, 'origin': [0., 0., 0.], 'data': [0, 100, 0, 0]}
        mapping.update_slam(live, [7., 1., math.pi / 2])
        self.assertGreater(mapping.grid['width'], 12)
        self.assertTrue(mapping.observed.any())
        np.testing.assert_allclose(compose([3., 2., 1.], inverse([3., 2., 1.])), [0., 0., 0.], atol=1e-12)

    def test_marker_observations_refine_destinations_without_using_cloud_coordinates(self):
        mapping = AdaptiveMap(environment())
        for i, x in enumerate([2.1, 2.11, 2.09]):
            mapping.observe_marker(2, [x, 1.1, .1], i + 1)
        goal = mapping.destination({'marker_id': 2, 'dictionary': 'DICT_4X4_50', 'destination': {'x': 99, 'y': 99}})
        np.testing.assert_allclose(goal, [2.1, 1.1, .1], atol=.01)
        exported = mapping.export([1, 2, 0])
        self.assertEqual(exported['start'], [1, 2])
        json.dumps(exported, allow_nan=False)

    def test_routing_avoids_new_obstacles_and_does_not_cut_diagonal_corners(self):
        mapping = AdaptiveMap(environment())
        planner = Planner(mapping.grid, radius=.1, resolution=.5)
        route = planner.path([1.25, 1.25], [4.25, 1.25])
        self.assertTrue(route)
        live = copy.deepcopy(mapping.grid)
        for y in range(12):
            live['data'][y * 12 + 5] = 100
        mapping.update_slam(live, [0, 0, 0])
        self.assertFalse(Planner(mapping.grid, radius=.1, resolution=.5).path([1.25, 1.25], [4.25, 1.25]))


class QueueAPI:
    def __init__(self, queue):
        self.queue = queue
        self.fail_switch_once = False
        self.fail_claim_once = False

    def get(self, path):
        return {'jobs': self.queue.pending()}

    def post(self, path, payload):
        if path.endswith('/claim'):
            result = self.queue.claim(payload)
            if self.fail_claim_once:
                self.fail_claim_once = False
                raise ConnectionError('Claim response lost')
            return result
        if path.endswith('/switch'):
            result = self.queue.switch(payload)
            if self.fail_switch_once:
                self.fail_switch_once = False
                raise ConnectionError('Switch response lost')
            return result
        return self.queue.finish(path.split('/')[-2], payload)


class Driver:
    def __init__(self, mapping):
        self.mapping = mapping
        self.pose = [1., 1., 0.]
        self.job = None
        self.stops = 0
        self.goals = []
        self.reached = False

    def snapshot(self):
        return {'ready': True, 'pose': self.pose, 'reason': ''}

    def destination(self, job):
        return self.mapping.destination(job)

    def set_goal(self, job):
        self.job = job
        self.goals.append(job['id'])

    def arrived(self):
        return self.reached

    def stop(self):
        self.stops += 1
        self.job = None


class MissionTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)
        self.queue = Queue(self.directory / 'queue.sqlite', environment())
        self.api = QueueAPI(self.queue)
        self.driver = Driver(AdaptiveMap(environment()))
        self.missions = Missions(self.api, self.driver, self.directory / 'journal.json')
        self.addCleanup(self.missions.close)

    def submit(self, marker, request):
        return self.queue.submit({'marker_id': marker, 'dictionary': 'DICT_4X4_50', 'request_id': request})

    def test_nearer_call_preempts_and_old_call_remains_pending(self):
        far = self.submit(1, 'far')
        self.missions.step(0)
        near = self.submit(2, 'near')
        self.missions.step(1)
        self.assertEqual(self.driver.goals, [far['id'], near['id']])
        self.assertEqual(self.driver.stops, 1)
        self.assertEqual(self.queue.get(far['id'])['status'], 'queued')
        self.assertEqual(self.queue.get(near['id'])['status'], 'running')
        self.driver.reached = True
        self.missions.step(2)
        self.missions.step(3)
        self.driver.reached = False
        self.missions.step(4)
        self.assertEqual(self.driver.goals[-1], far['id'])

    def test_lost_switch_response_is_retried_without_double_claim_or_motion(self):
        far = self.submit(1, 'far')
        self.missions.step(0)
        near = self.submit(2, 'near')
        self.api.fail_switch_once = True
        with self.assertRaises(ConnectionError):
            self.missions.step(1)
        self.assertIsNone(self.driver.job)
        self.assertEqual(self.missions.state['phase'], 'switching')
        self.missions.step(2)
        self.assertEqual(self.driver.goals, [far['id'], near['id']])
        self.assertEqual(sum(j['status'] == 'running' for j in self.queue.pending()), 1)

    def test_further_or_nearly_equal_call_does_not_interrupt_current_trip(self):
        near = self.submit(2, 'near')
        self.missions.step(0)
        self.submit(1, 'far')
        self.submit(2, 'equal')
        self.missions.step(1)
        self.assertEqual(self.driver.goals, [near['id']])
        self.assertEqual(self.driver.stops, 0)

    def test_lost_selected_claim_response_recovers_before_motion(self):
        near = self.submit(2, 'near')
        self.api.fail_claim_once = True
        with self.assertRaises(ConnectionError):
            self.missions.step(0)
        self.assertEqual(self.missions.state['phase'], 'claiming')
        self.assertEqual(self.driver.goals, [])
        self.missions.step(1)
        self.assertEqual(self.driver.goals, [near['id']])
        self.assertEqual(sum(j['status'] == 'running' for j in self.queue.pending()), 1)

    def test_unconfirmed_stop_never_requeues_or_claims_another_trip(self):
        far = self.submit(1, 'far')
        self.missions.step(0)
        near = self.submit(2, 'near')
        def failed_stop():
            raise RuntimeError('Stop unconfirmed')
        self.driver.stop = failed_stop
        with self.assertRaisesRegex(RuntimeError, 'Stop unconfirmed'):
            self.missions.step(1)
        self.assertEqual(self.queue.get(far['id'])['status'], 'running')
        self.assertEqual(self.queue.get(near['id'])['status'], 'queued')

    def test_wrong_worker_cannot_switch_and_restart_does_not_replay(self):
        first = self.submit(1, 'far')
        self.missions.step(0)
        second = self.submit(2, 'near')
        with self.assertRaises(DispatchError):
            self.queue.switch({'worker': 'wrong', 'current_job_id': first['id'], 'next_job_id': second['id']})
        self.missions.close()
        with self.assertRaisesRegex(RuntimeError, 'Interrupted physical mission'):
            Missions(self.api, self.driver, self.directory / 'journal.json')


class SensorTests(unittest.TestCase):
    def test_missing_lidar_is_invalid_and_measurements_apply_real_sensor_tf(self):
        ranges = np.full(72, np.nan)
        ranges[36] = 1.
        values, valid = lidar_features(ranges, -math.pi, 2*math.pi/72, .1, 8., [.2, 0, 0], [0, 0, 0, 1], 6.)
        self.assertEqual(valid.sum(), 1)
        self.assertAlmostEqual(values[36], 1.2)
        command, reason = guard([.1, 0], {'lidar': (values, valid, 1.)}, 1.)
        self.assertEqual(command, [0., 0.])
        self.assertIn('stale', reason)

    def test_odometry_uses_real_counts_and_handles_counter_wrap(self):
        odom = WheelOdometry()
        initial = {'encodersEnabled': True, 'cpr': 1320, 'boot': 1, 'uptime': 10, 'counts': [2**31 - 10] * 4}
        self.assertIsNone(odom.update(initial))
        pose, velocity = odom.update({**initial, 'uptime': 110, 'counts': [-2**31 + 10] * 4})
        self.assertGreater(pose[0], 0.)
        self.assertAlmostEqual(pose[1], 0.)
        self.assertGreater(velocity[0], 0.)
        with self.assertRaisesRegex(ValueError, 'real, enabled encoders'):
            odom.update({**initial, 'encodersEnabled': False})


class CheckpointTests(unittest.TestCase):
    def test_live_feature_layout_matches_training_frame_and_model_stays_resident(self):
        from kenny_rl.config import load_config
        from kenny_rl.env import KennyEnv
        simulator = ROOT.parent / 'Kenny---Automotive-self-driving-trash-can'
        checkpoint = simulator / 'releases/dynamic-v3-seed2/model/best.zip'
        robot, config, _ = load_config(checkpoint.parent / 'config.json')
        fixture = json.loads((ROOT / 'examples/environment.json').read_text())
        env = KennyEnv(robot, config, environment=fixture)
        self.addCleanup(env.close)
        env.reset(seed=42)
        policy = ResidentPolicy(checkpoint)
        model = policy.model
        now = 1.
        sensors = {name: (getattr(env, name).ranges, getattr(env, name).valid, now) for name in ('lidar', 'depth', 'floor')}
        sensors['downward'] = (env.down_hazard, env.down_valid, now)
        command = policy.predict(sensors, env.estimate, env.world.goal, env.route, [env.world.width, env.world.height],
            env.measured_velocity, env.uncertainty, 0., now, False)
        np.testing.assert_allclose(policy.history[-1], env._frame(), atol=1e-6)
        self.assertTrue(np.isfinite(command).all())
        policy.destination_changed()
        policy.predict(sensors, env.estimate, env.world.start, env.route, [env.world.width, env.world.height],
            env.measured_velocity, env.uncertainty, 0., now, False)
        self.assertIs(policy.model, model)


if __name__ == '__main__':
    unittest.main()
