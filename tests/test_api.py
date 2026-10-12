"""Delivery, persistence, ownership, and receiver failure semantics."""
from concurrent.futures import ThreadPoolExecutor
import io
import asyncio
import httpx
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import URLError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from app.queue import Queue, DispatchError
from app.main import create_app
from pi.receiver import API, consume, command_controller


def run_async(coro):
    # Some restricted environments suppress cross-thread asyncio wakeups.
    async def heartbeat():
        while True:
            await asyncio.sleep(.02)
    loop = asyncio.new_event_loop()
    ticker = loop.create_task(heartbeat())
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.run_until_complete(loop.shutdown_default_executor())
        ticker.cancel()
        loop.run_until_complete(asyncio.gather(ticker, return_exceptions=True))
        loop.close()


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.db = Path(self.temp.name)/'queue.sqlite'
        self.env = json.loads((ROOT/'examples/environment.json').read_text())
        self.queue = Queue(self.db, self.env)
        self.body = {'marker_id': 12, 'dictionary': self.env['dictionary'], 'request_id': 'scan-1'}

    def test_persistence_and_destination(self):
        job = self.queue.submit(self.body)
        self.assertEqual(job['destination']['frame_id'], 'map')
        self.assertEqual((job['destination']['x'], job['destination']['y']), (2., 2.))
        self.assertEqual(Queue(self.db, self.env).submit(self.body)['id'], job['id'])
        with self.assertRaises(DispatchError):
            self.queue.submit({**self.body, 'caller': 'someone else'})

    def test_concurrent_claim_and_fifo(self):
        first = self.queue.submit(self.body)
        second = self.queue.submit({**self.body, 'request_id': 'scan-2'})
        with ThreadPoolExecutor(max_workers=6) as pool:
            claims = list(pool.map(lambda i: self.queue.claim({'worker': f'pi-{i}'}), range(6)))
        jobs = [claim['job'] for claim in claims if claim['job']]
        self.assertEqual(len(jobs), 1)
        self.assertEqual(jobs[0]['id'], first['id'])
        with self.assertRaises(DispatchError):
            self.queue.finish(first['id'], {'worker': 'wrong', 'status': 'succeeded'})
        done = {'worker': jobs[0]['worker'], 'status': 'succeeded'}
        self.queue.finish(first['id'], done)
        self.queue.finish(first['id'], done)
        self.assertEqual(self.queue.claim({'worker': 'pi'})['job']['id'], second['id'])

    def test_validation(self):
        for change in [{'marker_id': True}, {'marker_id': 999}, {'dictionary': 'bad'}, {'request_id': ''}]:
            with self.subTest(change=change), self.assertRaises(DispatchError):
                self.queue.submit({**self.body, **change})
        blocked = json.loads(json.dumps(self.env))
        blocked['grid']['data'] = [100]*len(blocked['grid']['data'])
        with self.assertRaisesRegex(DispatchError, 'free map cell'):
            Queue(Path(self.temp.name)/'blocked.sqlite', blocked).submit(self.body)

    def test_map_validation_and_rotated_origin(self):
        from app.environment import arrival_is_free, validate_environment
        for change in [{'dictionary': 'bad'}, {'markers': []}, {'version': 2}]:
            with self.assertRaises(ValueError):
                validate_environment({**self.env, **change})
        env = json.loads(json.dumps(self.env))
        env['grid']['data'][0] = True
        with self.assertRaises(ValueError):
            validate_environment(env)
        grid = {'width': 3, 'height': 2, 'resolution': 1., 'origin': [10., 20., 1.5707963267948966], 'data': [0, -1, 0, 0, 0, 0]}
        self.assertTrue(arrival_is_free({'grid': grid}, [9.5, 20.5]))
        self.assertFalse(arrival_is_free({'grid': grid}, [9.5, 21.5]))
        self.assertFalse(arrival_is_free({'grid': grid, 'obstacles': [{'x': 9.4, 'y': 20.4, 'width': .2, 'depth': .2}]}, [9.5, 20.5]))

    def test_no_tokens_required(self):
        async def startup():
            with patch.dict(os.environ, {}, clear=True):
                app = create_app(self.env, self.db)
                async with app.router.lifespan_context(app):
                    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='https://test') as client:
                        job = (await client.post('/api/calls', json=self.body)).json()
                        claim = await client.post('/api/pi/claim', json={'worker': 'pi'})
                        self.assertEqual(claim.json()['job']['id'], job['id'])
                        done = await client.post(f"/api/calls/{job['id']}/result", json={'worker': 'pi', 'status': 'succeeded'})
                        self.assertEqual(done.json()['status'], 'succeeded')
                        self.assertEqual((await client.get('/api/calls')).status_code, 200)
        run_async(startup())

    def test_public_scanner_with_optional_pi_token(self):
        with patch.dict(os.environ, {}, clear=True):
            app = create_app(self.env, self.db, pi_token='pi-token')
            async def request():
                async with app.router.lifespan_context(app):
                    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='https://test') as client:
                        self.assertEqual((await client.post('/api/calls', json=self.body)).status_code, 200)
                        self.assertEqual((await client.post('/api/pi/claim', json={'worker': 'pi'})).status_code, 401)
                        claim = await client.post('/api/pi/claim', json={'worker': 'pi'}, headers={'Authorization': 'Bearer pi-token'})
                        self.assertIsNotNone(claim.json()['job'])
            run_async(request())

    def test_receiver_without_token(self):
        with patch('pi.receiver.urlopen') as send:
            send.return_value.__enter__.return_value = io.BytesIO(b'{"job": null}')
            self.assertEqual(API('https://test').post('/api/pi/claim', {'worker': 'pi'}), {'job': None})
            request = send.call_args.args[0]
            self.assertIsNone(request.get_header('Authorization'))

    def test_cors_and_shared_token_compatibility(self):
        with patch.dict(os.environ, {'KENNY_CORS_ORIGINS': 'https://scanner.example', 'KENNY_DISPATCH_TOKEN': 'shared'}, clear=True):
            app = create_app(self.env, self.db)
            async def request():
                async with app.router.lifespan_context(app):
                    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='https://test') as client:
                        response = await client.options('/api/calls', headers={'Origin': 'https://scanner.example', 'Access-Control-Request-Method': 'POST', 'Access-Control-Request-Headers': 'authorization,content-type'})
                        self.assertEqual(response.status_code, 200)
                        self.assertEqual(response.headers['access-control-allow-origin'], 'https://scanner.example')
                        self.assertEqual((await client.post('/api/pi/claim', json={'worker': 'pi'}, headers={'Authorization': 'Bearer shared'})).status_code, 200)
            run_async(request())

    def test_public_api(self):
        app = create_app(self.env, self.db, 'scanner-token', 'pi-token')
        async def routes():
            async with app.router.lifespan_context(app):
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='https://test') as client:
                    scanner = {'Authorization': 'Bearer scanner-token'}
                    pi = {'Authorization': 'Bearer pi-token'}
                    self.assertEqual((await client.get('/health')).status_code, 200)
                    self.assertEqual((await client.post('/api/calls', json=self.body)).status_code, 401)
                    response = await client.post('/api/calls', json=self.body, headers=scanner)
                    self.assertEqual(response.status_code, 200)
                    job = response.json()
                    duplicate = await client.post('/api/calls', json=self.body, headers=scanner)
                    self.assertEqual(duplicate.json()['id'], job['id'])
                    self.assertEqual((await client.post('/api/pi/claim', json={'worker': 'pi'}, headers=scanner)).status_code, 401)
                    claim = await client.post('/api/pi/claim', json={'worker': 'pi'}, headers=pi)
                    self.assertEqual(claim.json()['job']['id'], job['id'])
                    self.assertIsNone((await client.post('/api/pi/claim', json={'worker': 'other'}, headers=pi)).json()['job'])
                    done = await client.post(f"/api/calls/{job['id']}/result", json={'worker': 'pi', 'status': 'succeeded'}, headers=pi)
                    self.assertEqual(done.json()['status'], 'succeeded')
                    self.assertEqual((await client.get(f"/api/calls/{job['id']}", headers=scanner)).json()['status'], 'succeeded')
                    self.assertEqual(len((await client.get('/api/calls', headers=scanner)).json()['jobs']), 1)
                    self.assertEqual((await client.post('/api/pi/claim', json=[], headers=pi)).status_code, 422)
                    self.assertEqual((await client.get('/api/calls/missing', headers=pi)).status_code, 404)
                    self.assertEqual((await client.post('/api/calls', content=b'invalid', headers=scanner)).status_code, 422)
                    self.assertEqual((await client.post('/api/calls', content=b'x'*17000, headers=scanner)).status_code, 413)
        run_async(routes())

    def test_receiver_result_retry_does_not_repeat_motion(self):
        job = self.queue.submit(self.body)
        queue = self.queue
        class LocalAPI:
            uploads = 0
            def post(self, path, body):
                if path == '/api/pi/claim':
                    return queue.claim(body)
                self.uploads += 1
                if self.uploads == 1:
                    raise URLError('lost connection')
                return queue.finish(job['id'], body)
        trips = []
        def controller(job, timeout):
            trips.append(job['id'])
            return {'status': 'succeeded', 'detail': 'arrived'}
        state = Path(self.temp.name)/'state.json'
        with patch('pi.receiver.time.sleep'):
            consume(LocalAPI(), 'pi', state, controller, 60, once=True)
        self.assertEqual(trips, [job['id']])
        self.assertEqual(queue.get(job['id'])['status'], 'succeeded')
        self.assertFalse(state.exists())

    def test_interrupted_receiver_blocks_restart(self):
        self.queue.submit(self.body)
        queue = self.queue
        class LocalAPI:
            def post(self, path, body):
                return queue.claim(body)
        state = Path(self.temp.name)/'state.json'
        def broken(job, timeout):
            raise RuntimeError('sensor bridge lost')
        with self.assertRaisesRegex(RuntimeError, 'sensor bridge lost'):
            consume(LocalAPI(), 'pi', state, broken, 60, once=True)
        with self.assertRaisesRegex(RuntimeError, 'Interrupted job'):
            consume(LocalAPI(), 'pi', state, broken, 60, once=True)
        self.assertIsNone(queue.claim({'worker': 'other'})['job'])

    def test_model_command_receives_destination(self):
        script = "import json,sys; j=json.load(sys.stdin); print(json.dumps({'status':'succeeded','detail':str(j['destination']['x'])}))"
        job = self.queue.submit(self.body)
        result = command_controller([sys.executable, '-c', script], job, 10)
        self.assertEqual(result, {'status': 'succeeded', 'detail': '2.0'})


if __name__ == '__main__':
    unittest.main()
