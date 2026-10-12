import copy
import json
from pathlib import Path
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor

import httpx

from app.main import create_app
from app.queue import Queue, DispatchError
from test_api import run_async

ROOT = Path(__file__).resolve().parents[1]


class MapTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.db = Path(temp.name) / 'queue.sqlite'
        self.environment = json.loads((ROOT / 'examples/environment.json').read_text())
        self.queue = Queue(self.db, self.environment)

    def changed(self, name='Uploaded room', displacement=.2):
        environment = copy.deepcopy(self.environment)
        environment['name'] = name
        next(m for m in environment['markers'] if m['id'] == 12)['x'] += displacement
        return environment

    def test_uploaded_map_survives_restart_and_changes_call_destinations(self):
        before = self.queue.map()
        result = self.queue.upload_map({'environment': self.changed(), 'expected_sha256': before['sha256']})
        self.assertEqual(result['revision'], 2)
        restarted = Queue(self.db, self.environment)
        self.assertEqual(restarted.map(), result)
        job = restarted.submit({'marker_id': 12, 'dictionary': 'DICT_4X4_50', 'request_id': 'new-map-call'})
        self.assertAlmostEqual(job['destination']['x'], 2.2)
        self.assertEqual(job['environment_sha256'], result['sha256'])

    def test_pending_trip_prevents_map_replacement_and_identical_upload_is_idempotent(self):
        self.queue.submit({'marker_id': 12, 'dictionary': 'DICT_4X4_50', 'request_id': 'pending'})
        with self.assertRaisesRegex(DispatchError, 'Finish pending'):
            self.queue.upload_map({'environment': self.changed()})
        current = self.queue.map()
        self.assertEqual(self.queue.upload_map({'environment': current['environment']}), current)

    def test_conflicting_uploads_are_atomic_and_revision_checked(self):
        expected = self.queue.map()['sha256']
        def upload(index):
            try:
                return self.queue.upload_map({'environment': self.changed(f'Map {index}', .1 * index), 'expected_sha256': expected})
            except DispatchError as error:
                return error.status
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(upload, [1, 2]))
        self.assertEqual(sum(isinstance(result, dict) for result in results), 1)
        self.assertIn(409, results)
        self.assertEqual(self.queue.map()['revision'], 2)

    def test_invalid_marker_arrival_and_nonfinite_metadata_do_not_change_the_map(self):
        current = self.queue.map()
        blocked = self.changed()
        blocked['grid']['data'] = [100] * len(blocked['grid']['data'])
        with self.assertRaisesRegex(DispatchError, 'arrival point'):
            self.queue.upload_map({'environment': blocked})
        with self.assertRaises(DispatchError):
            self.queue.upload_map({'environment': {**self.changed(), 'mapping': {'quality': float('nan')}}})
        self.assertEqual(self.queue.map(), current)

    def test_http_roles_map_body_size_and_fetch_upload_round_trip(self):
        app = create_app(self.environment, self.db, scanner_token='scanner', pi_token='pi')
        async def check():
            async with app.router.lifespan_context(app):
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
                    scanner, pi = {'Authorization': 'Bearer scanner'}, {'Authorization': 'Bearer pi'}
                    self.assertEqual((await client.get('/api/map')).status_code, 401)
                    initial = (await client.get('/api/map', headers=scanner)).json()
                    payload = {'environment': self.changed(), 'expected_sha256': initial['sha256']}
                    self.assertGreater(len(json.dumps(payload)), 16384)
                    self.assertEqual((await client.post('/api/map', json=payload, headers=scanner)).status_code, 401)
                    response = await client.post('/api/map', json=payload, headers=pi)
                    self.assertEqual(response.status_code, 200, response.text)
                    self.assertEqual((await client.get('/api/map', headers=pi)).json(), response.json())
                    self.assertEqual((await client.post('/api/calls', content=b' ' * 17000, headers=scanner)).status_code, 413)
        run_async(check())


if __name__ == '__main__':
    unittest.main()
