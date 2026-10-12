import json
from pathlib import Path
import tempfile
import unittest

import httpx

from app.main import create_app
from test_api import run_async


class PreemptionAPITests(unittest.TestCase):
    def test_pi_queue_selected_claim_switch_retry_and_role_tokens(self):
        environment = json.loads((Path(__file__).resolve().parents[1] / 'examples/environment.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            app = create_app(environment, Path(directory) / 'queue.sqlite', scanner_token='scanner', pi_token='robot')

            async def check():
                async with app.router.lifespan_context(app):
                    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
                        scanner, robot = {'Authorization': 'Bearer scanner'}, {'Authorization': 'Bearer robot'}
                        jobs = []
                        for request in ('first', 'second'):
                            response = await client.post('/api/calls', headers=scanner,
                                json={'request_id': request, 'marker_id': 12, 'dictionary': environment['dictionary']})
                            self.assertEqual(response.status_code, 200)
                            jobs.append(response.json())
                        self.assertEqual((await client.get('/api/pi/queue', headers=scanner)).status_code, 401)
                        self.assertEqual(len((await client.get('/api/pi/queue', headers=robot)).json()['jobs']), 2)
                        payload = {'worker': 'main', 'job_id': jobs[1]['id']}
                        claim = await client.post('/api/pi/claim', json=payload, headers=robot)
                        self.assertEqual(claim.json()['job']['id'], jobs[1]['id'])
                        self.assertEqual((await client.post('/api/pi/claim', json=payload, headers=robot)).json()['job']['id'], jobs[1]['id'])
                        switch = {'worker': 'main', 'current_job_id': jobs[1]['id'], 'next_job_id': jobs[0]['id']}
                        self.assertEqual((await client.post('/api/pi/switch', json=switch, headers=scanner)).status_code, 401)
                        response = await client.post('/api/pi/switch', json=switch, headers=robot)
                        self.assertEqual(response.json()['job']['id'], jobs[0]['id'])
                        self.assertEqual((await client.post('/api/pi/switch', json=switch, headers=robot)).json(), response.json())
                        pending = (await client.get('/api/pi/queue', headers=robot)).json()['jobs']
                        self.assertEqual(sum(j['status'] == 'running' for j in pending), 1)
                        self.assertEqual(next(j for j in pending if j['id'] == jobs[1]['id'])['status'], 'queued')

            run_async(check())


if __name__ == '__main__':
    unittest.main()
