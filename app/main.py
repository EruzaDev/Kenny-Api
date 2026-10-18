"""Standalone scanner/robot API. No editor, simulator, or model dependencies."""
import asyncio
from contextlib import asynccontextmanager
import hmac
import json
import logging
import os
from pathlib import Path
import sqlite3

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .queue import Queue, DispatchError


def json_schema(properties, required):
    return {'requestBody': {'required': True, 'content': {'application/json': {'schema':
            {'type': 'object', 'properties': properties, 'required': required}}}}}


CALL_SCHEMA = json_schema({'request_id': {'type': 'string', 'example': 'scan-unique-uuid'},
                           'marker_id': {'type': 'integer', 'example': 12},
                           'dictionary': {'type': 'string', 'example': 'DICT_4X4_50'},
                           'caller': {'type': 'string', 'example': 'android-scanner'}},
                          ['request_id', 'marker_id', 'dictionary'])
CLAIM_SCHEMA = json_schema({'worker': {'type': 'string', 'example': 'kenny-pi'},
                            'job_id': {'type': 'string', 'description': 'Optional selected queued job'}}, ['worker'])
SWITCH_SCHEMA = json_schema({'worker': {'type': 'string'}, 'current_job_id': {'type': 'string'},
                             'next_job_id': {'type': 'string'}}, ['worker', 'current_job_id', 'next_job_id'])
CANCEL_SCHEMA = json_schema({'job_id': {'type': 'string', 'description': 'ID of an unclaimed queued call'}}, ['job_id'])
RESULT_SCHEMA = json_schema({'worker': {'type': 'string', 'example': 'kenny-pi'},
                             'status': {'type': 'string', 'enum': ['succeeded', 'failed']},
                             'detail': {'type': 'string'}}, ['worker', 'status'])


def create_app(environment=None, db_path=None, scanner_token=None, pi_token=None):
    @asynccontextmanager
    async def lifespan(app):
        shared = os.environ.get('KENNY_DISPATCH_TOKEN', '')
        app.state.scanner_token = scanner_token if scanner_token is not None else os.environ.get('KENNY_SCANNER_TOKEN', shared)
        app.state.pi_token = pi_token if pi_token is not None else os.environ.get('KENNY_PI_TOKEN', shared)
        path = os.environ.get('KENNY_DISPATCH_ENVIRONMENT') or 'examples/environment.json'
        try:
            env = environment if environment is not None else json.loads(Path(path).read_text())
            app.state.queue = Queue(db_path or os.environ.get('KENNY_DISPATCH_DB', 'data/dispatch.sqlite3'), env)
        except (OSError, ValueError, KeyError, TypeError, sqlite3.Error) as error:
            raise RuntimeError(f'Cannot initialize queue/map: {error}') from error
        yield

    app = FastAPI(title='Kenny Dispatch API', version='1.0.0', lifespan=lifespan)
    default_origins = 'https://localhost,http://localhost,http://localhost:5173,http://127.0.0.1:5173'
    origins = [origin.strip() for origin in os.environ.get('KENNY_CORS_ORIGINS', default_origins).split(',') if origin.strip()]
    if origins:
        app.add_middleware(CORSMiddleware, allow_origins=origins,
                           allow_methods=['GET', 'POST'], allow_headers=['Authorization', 'Content-Type'])

    def authorize(request, pi_only=False):
        required = app.state.pi_token if pi_only else app.state.scanner_token
        if not required:
            return
        supplied = request.headers.get('authorization', '').encode()
        tokens = [required] if pi_only else [required, app.state.pi_token]
        tokens = [token for token in tokens if token]
        if not any(hmac.compare_digest(supplied, f'Bearer {token}'.encode()) for token in tokens):
            raise HTTPException(401, 'Invalid API token', headers={'WWW-Authenticate': 'Bearer'})

    async def body(request, limit=16384):
        content = bytearray()
        async for chunk in request.stream():
            content.extend(chunk)
            if len(content) > limit:
                raise HTTPException(413, f'Request body must be at most {limit} bytes')
        try:
            value = json.loads(content)
        except (ValueError, UnicodeDecodeError):
            raise HTTPException(422, 'Expected valid JSON')
        if not isinstance(value, dict):
            raise HTTPException(422, 'Expected a JSON object')
        return value

    @app.exception_handler(DispatchError)
    async def dispatch_error(request, error):
        return JSONResponse({'detail': str(error)}, status_code=error.status)

    @app.exception_handler(sqlite3.Error)
    async def storage_error(request, error):
        logging.getLogger('kenny-api').exception('Queue storage failed', exc_info=error)
        return JSONResponse({'detail': 'Queue storage unavailable; retry with the same request ID'}, status_code=503)

    @app.get('/health')
    async def health():
        def check():
            with app.state.queue.connect() as db:
                db.execute('SELECT 1 FROM jobs LIMIT 1').fetchone()
        await asyncio.to_thread(check)
        return {'status': 'ok', 'service': 'kenny-api'}

    @app.get('/')
    async def index():
        return {'service': 'Kenny Dispatch API', 'version': '1.0.0', 'docs': '/docs', 'health': '/health'}

    @app.post('/api/calls', openapi_extra=CALL_SCHEMA)
    async def submit(request: Request):
        authorize(request)
        payload = await body(request)
        return await asyncio.to_thread(app.state.queue.submit, payload)

    @app.get('/api/map')
    async def fetch_map(request: Request):
        authorize(request)
        return await asyncio.to_thread(app.state.queue.map)

    @app.post('/api/map', openapi_extra=json_schema({
        'environment': {'type': 'object', 'description': 'Exported version 1 environment.json'},
        'expected_sha256': {'type': 'string', 'description': 'Optional hash of the map being replaced'}}, ['environment']))
    async def upload_map(request: Request):
        authorize(request, pi_only=True)
        payload = await body(request, limit=32_000_000)
        return await asyncio.to_thread(app.state.queue.upload_map, payload)

    @app.get('/api/robot')
    async def robot_position(request: Request):
        authorize(request)
        return await asyncio.to_thread(app.state.queue.robot)

    @app.post('/api/pi/pose', openapi_extra=json_schema({
        'worker': {'type': 'string'}, 'pose': {'type': 'object', 'description': 'map-frame {frame_id,x,y,yaw}'},
        'source': {'type': 'string', 'enum': ['physical', 'simulation']},
        'observed_at': {'type': 'number', 'description': 'Unix seconds of pose acquisition'},
        'goal': {'type': 'object'}, 'route': {'type': 'array'}}, ['worker', 'pose']))
    async def publish_robot_position(request: Request):
        authorize(request, pi_only=True)
        payload = await body(request)
        return await asyncio.to_thread(app.state.queue.publish_pose, payload)

    @app.get('/api/calls')
    async def calls(request: Request):
        authorize(request)
        return {'jobs': await asyncio.to_thread(app.state.queue.list)}

    @app.get('/api/calls/{job_id}')
    async def call(job_id: str, request: Request):
        authorize(request)
        return await asyncio.to_thread(app.state.queue.get, job_id)

    @app.post('/api/pi/claim', openapi_extra=CLAIM_SCHEMA)
    async def claim(request: Request):
        authorize(request, pi_only=True)
        payload = await body(request)
        return await asyncio.to_thread(app.state.queue.claim, payload)

    @app.get('/api/pi/queue')
    async def pending(request: Request):
        authorize(request, pi_only=True)
        return {'jobs': await asyncio.to_thread(app.state.queue.pending)}

    @app.post('/api/pi/cancel', openapi_extra=CANCEL_SCHEMA)
    async def cancel(request: Request):
        authorize(request, pi_only=True)
        payload = await body(request)
        return await asyncio.to_thread(app.state.queue.cancel, payload)

    @app.post('/api/pi/switch', openapi_extra=SWITCH_SCHEMA)
    async def switch(request: Request):
        authorize(request, pi_only=True)
        payload = await body(request)
        return await asyncio.to_thread(app.state.queue.switch, payload)

    @app.post('/api/calls/{job_id}/result', openapi_extra=RESULT_SCHEMA)
    async def result(job_id: str, request: Request):
        authorize(request, pi_only=True)
        payload = await body(request)
        return await asyncio.to_thread(app.state.queue.finish, job_id, payload)

    return app


app = create_app()
