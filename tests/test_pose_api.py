import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import httpx

from app.main import create_app
from app.queue import Queue, DispatchError
from controller.tracking import Tracker
from test_api import run_async

ROOT = Path(__file__).resolve().parents[1]


class PoseTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.db = Path(temp.name) / 'queue.sqlite'
        self.environment = json.loads((ROOT / 'tests/fixtures/environment.json').read_text())
        self.queue = Queue(self.db, self.environment)
        self.packet = {'worker': 'robot', 'pose': {'frame_id': 'map', 'x': 1., 'y': 2., 'yaw': .3},
                       'source': 'physical', 'status': 'navigating', 'observed_at': 100.,
                       'goal': {'x': 2., 'y': 2., 'marker_id': 12}, 'route': [[1., 2.], [2., 2.]]}

    def test_pose_persistence_staleness_and_map_replacement(self):
        self.assertIsNone(self.queue.robot()['pose'])
        with patch('app.queue.time.time', return_value=100.):
            result = self.queue.publish_pose(self.packet)
            self.assertFalse(result['stale'])
            self.assertEqual(result['source'], 'physical')
        with patch('app.queue.time.time', return_value=106.):
            restarted = Queue(self.db, self.environment)
            self.assertTrue(restarted.robot()['stale'])
            self.assertEqual(restarted.robot()['pose'], self.packet['pose'])
            changed = {**self.environment, 'name': 'Another map'}
            restarted.upload_map({'environment': changed})
            self.assertIsNone(restarted.robot()['pose'])
            with self.assertRaisesRegex(DispatchError, 'Tracking map changed'):
                restarted.publish_pose({**self.packet, 'observed_at': 106., 'map_sha256': result['map_sha256']})

    def test_display_state_sync_does_not_refresh_stale_position(self):
        with patch('app.queue.time.time', return_value=100.):
            self.queue.publish_pose(self.packet)
        with patch('app.queue.time.time', return_value=110.):
            result = self.queue.set_display_state({'status': 'idle'})
            self.assertEqual(result['status'], 'idle')
            self.assertTrue(result['stale'])
            self.assertEqual(result['observed_at'], 100.)
            self.assertEqual(result['display_updated_at'], 110.)
            with self.assertRaises(DispatchError):
                self.queue.set_display_state({'status': 'stopped'})
        with patch('app.queue.time.time', return_value=111.):
            result = Queue(self.db, self.environment).robot()
            self.assertEqual(result['status'], 'idle')
            self.assertTrue(result['stale'])
            self.queue.publish_pose({**self.packet, 'observed_at': 111.})
            self.assertEqual(self.queue.robot()['status'], 'idle')

    def test_old_invalid_and_other_workers_cannot_replace_a_current_pose(self):
        with patch('app.queue.time.time', return_value=100.):
            self.queue.publish_pose(self.packet)
            with self.assertRaisesRegex(DispatchError, 'Older pose'):
                self.queue.publish_pose({**self.packet, 'observed_at': 99.})
            for pose in ({'frame_id': 'odom', 'x': 1, 'y': 2, 'yaw': 0}, {'frame_id': 'map', 'x': float('nan'), 'y': 2, 'yaw': 0}):
                with self.assertRaises(DispatchError):
                    self.queue.publish_pose({**self.packet, 'pose': pose})
            self.queue.submit({'request_id': 'call', 'marker_id': 12, 'dictionary': 'DICT_4X4_50'})
            self.queue.claim({'worker': 'robot'})
            with self.assertRaisesRegex(DispatchError, 'Another worker'):
                self.queue.publish_pose({**self.packet, 'worker': 'simulation'})

    def test_pose_http_roles_and_simulation_label(self):
        app = create_app(self.environment, self.db, scanner_token='scanner', pi_token='robot')
        async def check():
            async with app.router.lifespan_context(app):
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
                    scanner, robot = {'Authorization': 'Bearer scanner'}, {'Authorization': 'Bearer robot'}
                    packet = {**self.packet, 'source': 'simulation'}
                    packet.pop('observed_at')
                    self.assertEqual((await client.post('/api/pi/pose', json=packet, headers=scanner)).status_code, 401)
                    response = await client.post('/api/pi/pose', json=packet, headers=robot)
                    self.assertEqual(response.status_code, 200, response.text)
                    public = await client.get('/api/robot', headers=scanner)
                    self.assertEqual(public.json()['source'], 'simulation')
                    self.assertFalse(public.json()['stale'])
        run_async(check())

    def test_display_status_endpoint_accepts_pi_role_without_pose(self):
        app = create_app(self.environment, self.db, scanner_token='scanner', pi_token='robot')
        async def check():
            async with app.router.lifespan_context(app):
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
                    scanner, robot = {'Authorization': 'Bearer scanner'}, {'Authorization': 'Bearer robot'}
                    self.assertEqual((await client.post('/api/pi/display-state', json={'status':'navigating'}, headers=scanner)).status_code, 401)
                    response = await client.post('/api/pi/display-state', json={'status':'navigating'}, headers=robot)
                    self.assertEqual(response.status_code, 200, response.text)
                    self.assertEqual(response.json()['status'], 'navigating')
                    self.assertTrue(response.json()['stale'])
                    self.assertIsNone(response.json()['pose'])
                    self.assertEqual((await client.get('/api/robot', headers=scanner)).json()['status'], 'navigating')
        run_async(check())

    def test_tracker_pins_the_map_and_does_not_publish_missing_pose(self):
        api = unittest.mock.Mock()
        api.get.return_value = {'sha256': 'map-hash'}
        tracker = Tracker(api, 'robot')
        tracker.publish(None)
        api.post.assert_not_called()
        tracker.publish(self.packet)
        tracker.publish(self.packet)
        self.assertEqual(api.get.call_count, 1)
        self.assertEqual(api.post.call_args.args[1]['map_sha256'], 'map-hash')

    def test_native_mobile_origin_is_allowed_without_blueprint_env_sync(self):
        with patch.dict('os.environ', {}, clear=True):
            app = create_app(self.environment, self.db)
        async def check():
            async with app.router.lifespan_context(app):
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
                    response = await client.get('/api/robot', headers={'Origin': 'https://localhost'})
                    self.assertEqual(response.headers['access-control-allow-origin'], 'https://localhost')
        run_async(check())


if __name__ == '__main__':
    unittest.main()
