"""Durable single-robot destination queue shared by both HTTP adapters."""
import copy
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time
import uuid

from .environment import destination_goal, validate_environment, arrival_is_free


class DispatchError(ValueError):
    def __init__(self, message, status=422):
        super().__init__(message)
        self.status = status


def label(value, name):
    if not isinstance(value, str) or not 1 <= len(value.strip()) <= 128:
        raise DispatchError(f'{name} must be 1–128 characters')
    return value.strip()


class Queue:
    def __init__(self, path, environment):
        self.path = str(path)
        self.environment = validate_environment(copy.deepcopy(environment))
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            db.execute('PRAGMA journal_mode=WAL')
            db.execute('''CREATE TABLE IF NOT EXISTS jobs (
                sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                id TEXT UNIQUE NOT NULL, request_id TEXT UNIQUE NOT NULL,
                fingerprint TEXT NOT NULL, payload TEXT NOT NULL,
                status TEXT NOT NULL, worker TEXT, created REAL NOT NULL,
                updated REAL NOT NULL, detail TEXT NOT NULL DEFAULT '')''')

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=10)
        db.row_factory = sqlite3.Row
        try:
            with db:
                yield db
        finally:
            db.close()

    @staticmethod
    def packet(row):
        return {**json.loads(row['payload']), **{key: row[key] for key in
                ('id', 'request_id', 'status', 'worker', 'created', 'updated', 'detail')}}

    def submit(self, body):
        if not isinstance(body, dict):
            raise DispatchError('Expected a JSON object')
        request_id = label(body.get('request_id'), 'request_id')
        caller = label(body.get('caller', 'scanner'), 'caller')
        mid = body.get('marker_id')
        if type(mid) is not int:
            raise DispatchError('marker_id must be an integer')
        if body.get('dictionary') != self.environment['dictionary']:
            raise DispatchError('Scanner dictionary must match the loaded environment')
        fingerprint = json.dumps([mid, body['dictionary'], caller])
        # Check the idempotency key before resolving against a potentially changed map.
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            existing = db.execute('SELECT * FROM jobs WHERE request_id=?', (request_id,)).fetchone()
            if existing:
                if existing['fingerprint'] != fingerprint:
                    raise DispatchError('request_id already used for a different call', 409)
                return self.packet(existing)
            marker = next((m for m in self.environment.get('markers', []) if m['id'] == mid), None)
            if marker is None:
                raise DispatchError('Marker is not registered in the server map')
            env = {**self.environment, 'destination_marker_id': mid}
            # Use the mount-specific default, rather than another marker's saved offset.
            env.pop('destination_offset_m', None)
            goal = destination_goal(env)
            if not arrival_is_free(env, goal):
                raise DispatchError('Marker arrival point must be inside a free map cell')
            packet = {'marker_id': mid, 'dictionary': body['dictionary'], 'caller': caller,
                      'destination': {'frame_id': 'map', 'x': goal[0], 'y': goal[1],
                                      'yaw': marker['yaw'] + (math.pi if marker['mount'] == 'wall' else 0)},
                      'environment_sha256': hashlib.sha256(json.dumps(self.environment, sort_keys=True).encode()).hexdigest()}
            now = time.time()
            job_id = str(uuid.uuid4())
            db.execute('INSERT INTO jobs (id,request_id,fingerprint,payload,status,created,updated) VALUES (?,?,?,?,?,?,?)',
                       (job_id, request_id, fingerprint, json.dumps(packet), 'queued', now, now))
            return self.packet(db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone())

    def list(self):
        with self.connect() as db:
            return [self.packet(r) for r in db.execute('SELECT * FROM jobs ORDER BY sequence DESC LIMIT 200')]

    def get(self, job_id):
        with self.connect() as db:
            row = db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone()
            if row is None:
                raise DispatchError('Job not found', 404)
            return self.packet(row)

    def claim(self, body):
        worker = label(body.get('worker'), 'worker')
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            # A lost receiver stays running until explicitly resolved. Never replay
            # a physical trip automatically after a timeout or a Pi restart.
            if db.execute("SELECT 1 FROM jobs WHERE status='running'").fetchone():
                return {'job': None}
            row = db.execute("SELECT * FROM jobs WHERE status='queued' ORDER BY sequence LIMIT 1").fetchone()
            if row is None:
                return {'job': None}
            db.execute("UPDATE jobs SET status='running',worker=?,updated=? WHERE id=?", (worker, time.time(), row['id']))
            return {'job': self.packet(db.execute('SELECT * FROM jobs WHERE id=?', (row['id'],)).fetchone())}

    def finish(self, job_id, body):
        worker = label(body.get('worker'), 'worker')
        status = body.get('status')
        detail = body.get('detail', '')
        if status not in ('succeeded', 'failed') or not isinstance(detail, str) or len(detail) > 2000:
            raise DispatchError('Use succeeded/failed and a detail of at most 2000 characters')
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone()
            if row is None:
                raise DispatchError('Job not found', 404)
            if row['worker'] != worker or row['status'] not in ('running', status):
                raise DispatchError('Job is not running for this worker', 409)
            if row['status'] == 'running':
                db.execute('UPDATE jobs SET status=?,detail=?,updated=? WHERE id=?', (status, detail, time.time(), job_id))
            return self.packet(db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone())
