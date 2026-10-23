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

from .environment import destination_goal, validate_environment, arrival_is_free, number


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
        environment = validate_environment(copy.deepcopy(environment))
        serialized = json.dumps(environment, sort_keys=True, allow_nan=False)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            db.execute('PRAGMA journal_mode=WAL')
            db.execute('''CREATE TABLE IF NOT EXISTS jobs (
                sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                id TEXT UNIQUE NOT NULL, request_id TEXT UNIQUE NOT NULL,
                fingerprint TEXT NOT NULL, payload TEXT NOT NULL,
                status TEXT NOT NULL, worker TEXT, created REAL NOT NULL,
                updated REAL NOT NULL, detail TEXT NOT NULL DEFAULT '')''')
            db.execute('''CREATE TABLE IF NOT EXISTS active_map (
                singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                environment TEXT NOT NULL, sha256 TEXT NOT NULL,
                revision INTEGER NOT NULL, updated REAL NOT NULL)''')
            db.execute('INSERT OR IGNORE INTO active_map VALUES (1,?,?,1,?)',
                       (serialized, hashlib.sha256(serialized.encode()).hexdigest(), time.time()))
            db.execute('''CREATE TABLE IF NOT EXISTS robot_pose (
                singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                worker TEXT NOT NULL, payload TEXT NOT NULL,
                observed REAL NOT NULL, received REAL NOT NULL)''')
            db.execute('''CREATE TABLE IF NOT EXISTS robot_display (
                singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                status TEXT NOT NULL, updated REAL NOT NULL)''')

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

    @staticmethod
    def map_packet(row):
        return {'environment': json.loads(row['environment']), 'sha256': row['sha256'],
                'revision': row['revision'], 'updated': row['updated']}

    def map(self):
        with self.connect() as db:
            return self.map_packet(db.execute('SELECT * FROM active_map WHERE singleton=1').fetchone())

    @property
    def environment(self):
        return self.map()['environment']

    def upload_map(self, body):
        try:
            environment = validate_environment(copy.deepcopy(body['environment']))
            for key in ('start', 'goal'):
                if not isinstance(environment.get(key), list) or len(environment[key]) != 2 or not all(map(number, environment[key])):
                    raise ValueError(f'Map {key} must contain finite x/y')
            marker_id = environment.get('destination_marker_id')
            if marker_id is not None and (type(marker_id) is not int or not any(m['id'] == marker_id for m in environment['markers'])):
                raise ValueError('Selected destination marker is missing')
            for marker in environment['markers']:
                destination = destination_goal({**environment, 'destination_marker_id': marker['id']})
                if not arrival_is_free(environment, destination):
                    raise ValueError(f"ArUco ID {marker['id']} arrival point is blocked, unknown, or outside the map")
            # Camera/background images are editor assets, not navigation geometry.
            environment.pop('background', None)
            serialized = json.dumps(environment, sort_keys=True, allow_nan=False)
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            raise DispatchError(f'Invalid uploaded map: {error}') from error
        digest = hashlib.sha256(serialized.encode()).hexdigest()
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            current = db.execute('SELECT * FROM active_map WHERE singleton=1').fetchone()
            if digest == current['sha256']:
                return self.map_packet(current)
            expected = body.get('expected_sha256')
            if expected is not None and expected != current['sha256']:
                raise DispatchError('Render map changed since it was fetched. Fetch the latest map before uploading.', 409)
            if db.execute("SELECT 1 FROM jobs WHERE status IN ('queued','running') LIMIT 1").fetchone():
                raise DispatchError('Finish pending/running calls before replacing the active map.', 409)
            db.execute('UPDATE active_map SET environment=?,sha256=?,revision=revision+1,updated=? WHERE singleton=1',
                       (serialized, digest, time.time()))
            db.execute('DELETE FROM robot_pose')
            return self.map_packet(db.execute('SELECT * FROM active_map WHERE singleton=1').fetchone())

    @staticmethod
    def pose_packet(row, now):
        if row is None:
            return {'pose': None, 'stale': True, 'age_seconds': None, 'stale_after_seconds': 5.,
                    'status': 'offline', 'detail': 'No robot position has been received yet'}
        age = max(0., now - row['observed'], now - row['received'])
        return {**json.loads(row['payload']), 'worker': row['worker'], 'observed_at': row['observed'],
                'received_at': row['received'], 'age_seconds': age, 'stale': age > 5., 'stale_after_seconds': 5.}

    def robot(self):
        with self.connect() as db:
            return self.robot_packet(db, time.time())

    def robot_packet(self, db, now):
        packet = self.pose_packet(db.execute('SELECT * FROM robot_pose WHERE singleton=1').fetchone(), now)
        display = db.execute('SELECT status,updated FROM robot_display WHERE singleton=1').fetchone()
        if display:
            packet.update(status=display['status'], display_updated_at=display['updated'])
        return packet

    def set_display_state(self, body):
        status = body.get('status') if isinstance(body, dict) else None
        if status not in ('idle', 'navigating'):
            raise DispatchError('Display status must be idle or navigating')
        with self.connect() as db:
            now = time.time()
            db.execute('INSERT OR REPLACE INTO robot_display VALUES (1,?,?)', (status, now))
            return self.robot_packet(db, now)

    def publish_pose(self, body):
        worker = label(body.get('worker'), 'worker')
        pose = body.get('pose')
        if not isinstance(pose, dict) or pose.get('frame_id') != 'map' or not all(number(pose.get(k)) for k in ('x', 'y', 'yaw')) or max(abs(pose['x']), abs(pose['y'])) > 1_000_000:
            raise DispatchError('Pose must contain finite map-frame x/y/yaw in meters/radians')
        source = body.get('source', 'physical')
        if source not in ('physical', 'simulation'):
            raise DispatchError('Pose source must be physical or simulation')
        status = body.get('status', 'idle')
        if status not in ('idle', 'navigating', 'waiting', 'stopped', 'error'):
            raise DispatchError('Invalid robot tracking status')
        detail = body.get('detail', '')
        if not isinstance(detail, str) or len(detail) > 256:
            raise DispatchError('Tracking detail must be at most 256 characters')
        now = time.time()
        observed = body.get('observed_at', now)
        if not number(observed) or observed > now + 5 or observed < now - 60:
            raise DispatchError('Pose timestamp must be recent; synchronize the robot clock')
        uncertainty = body.get('uncertainty_m')
        if uncertainty is not None and (not number(uncertainty) or not 0 <= uncertainty <= 100):
            raise DispatchError('Invalid pose uncertainty')
        goal = body.get('goal')
        if goal is not None and (not isinstance(goal, dict) or not all(number(goal.get(k)) for k in ('x', 'y')) or (goal.get('marker_id') is not None and type(goal['marker_id']) is not int)):
            raise DispatchError('Invalid tracking destination')
        route = body.get('route', [])
        if not isinstance(route, list) or len(route) > 200 or any(not isinstance(p, list) or len(p) != 2 or not all(map(number, p)) for p in route):
            raise DispatchError('Tracking route must contain at most 200 finite x/y pairs')
        job_id = body.get('job_id')
        if job_id is not None:
            job_id = label(job_id, 'job_id')
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            running = db.execute("SELECT * FROM jobs WHERE status='running'").fetchone()
            if running is not None and running['worker'] != worker:
                raise DispatchError('Another worker owns the active robot trip', 409)
            current_map = db.execute('SELECT sha256,revision FROM active_map WHERE singleton=1').fetchone()
            expected = body.get('map_sha256')
            if expected is not None and expected != current_map['sha256']:
                raise DispatchError('Tracking map changed; reload the robot map before publishing a new pose', 409)
            previous = db.execute('SELECT * FROM robot_pose WHERE singleton=1').fetchone()
            if previous is not None and previous['worker'] == worker and observed < previous['observed']:
                raise DispatchError('Older pose update rejected', 409)
            packet = {'pose': {'frame_id': 'map', 'x': pose['x'], 'y': pose['y'], 'yaw': math.atan2(math.sin(pose['yaw']), math.cos(pose['yaw']))},
                      'source': source, 'status': status, 'detail': detail, 'uncertainty_m': uncertainty,
                      'goal': {k: goal[k] for k in ('x', 'y', 'marker_id') if k in goal} if goal is not None else None,
                      'route': route, 'job_id': job_id,
                      'map_sha256': current_map['sha256'], 'map_revision': current_map['revision']}
            db.execute('INSERT OR REPLACE INTO robot_pose VALUES (1,?,?,?,?)',
                       (worker, json.dumps(packet, allow_nan=False), observed, now))
            return self.robot_packet(db, now)

    def submit(self, body):
        if not isinstance(body, dict):
            raise DispatchError('Expected a JSON object')
        request_id = label(body.get('request_id'), 'request_id')
        caller = label(body.get('caller', 'scanner'), 'caller')
        mid = body.get('marker_id')
        if type(mid) is not int:
            raise DispatchError('marker_id must be an integer')
        dictionary = body.get('dictionary')
        limits = {'DICT_4X4_50': 50, 'DICT_4X4_1000': 1000, 'ARUCO': 1024}
        limit = limits.get(dictionary) if isinstance(dictionary, str) else None
        if limit is None or not 0 <= mid < limit:
            raise DispatchError('Use a supported scanner dictionary and a marker_id within its range')
        fingerprint = json.dumps([mid, dictionary, caller])
        # Check the idempotency key before resolving against a potentially changed map.
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            existing = db.execute('SELECT * FROM jobs WHERE request_id=?', (request_id,)).fetchone()
            if existing:
                if existing['fingerprint'] != fingerprint:
                    raise DispatchError('request_id already used for a different call', 409)
                return self.packet(existing)
            current = db.execute('SELECT * FROM active_map WHERE singleton=1').fetchone()
            environment = json.loads(current['environment'])
            marker = None
            if dictionary == environment['dictionary']:
                marker = next((m for m in environment['markers'] if m['id'] == mid), None)
            destination = None
            if marker is not None:
                env = {**environment, 'destination_marker_id': mid}
                # Use the mount-specific default, rather than another marker's saved offset.
                env.pop('destination_offset_m', None)
                goal = destination_goal(env)
                if arrival_is_free(env, goal):
                    destination = {'frame_id': 'map', 'x': goal[0], 'y': goal[1],
                                   'yaw': marker['yaw'] + (math.pi if marker['mount'] == 'wall' else 0)}
            status = 'queued' if destination is not None else 'awaiting_destination'
            packet = {'marker_id': mid, 'dictionary': body['dictionary'], 'caller': caller,
                      'destination': destination,
                       'environment_sha256': current['sha256']}
            now = time.time()
            job_id = str(uuid.uuid4())
            db.execute('INSERT INTO jobs (id,request_id,fingerprint,payload,status,created,updated) VALUES (?,?,?,?,?,?,?)',
                       (job_id, request_id, fingerprint, json.dumps(packet), status, now, now))
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

    def pending(self):
        with self.connect() as db:
            return [self.packet(r) for r in db.execute(
                "SELECT * FROM jobs WHERE status IN ('queued','running') ORDER BY sequence")]

    def cancel(self, body):
        """Atomically remove a queued call; never cancel a claimed robot trip."""
        if not isinstance(body, dict):
            raise DispatchError('Expected a JSON object')
        job_id = label(body.get('job_id'), 'job_id')
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone()
            if row is None:
                raise DispatchError('Job not found', 404)
            if row['status'] == 'cancelled':
                return self.packet(row)
            if row['status'] != 'queued':
                raise DispatchError('Only queued calls can be cancelled; a running trip must stop and resolve first', 409)
            db.execute("UPDATE jobs SET status='cancelled',worker=NULL,detail=?,updated=? WHERE id=?",
                       ('Cancelled from operations queue', time.time(), job_id))
            return self.packet(db.execute('SELECT * FROM jobs WHERE id=?', (job_id,)).fetchone())

    def claim(self, body):
        worker = label(body.get('worker'), 'worker')
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            # A lost receiver stays running until explicitly resolved. Never replay
            # a physical trip automatically after a timeout or a Pi restart.
            selected = body.get('job_id')
            if selected is not None:
                selected = label(selected, 'job_id')
            running = db.execute("SELECT * FROM jobs WHERE status='running'").fetchone()
            if running is not None:
                if selected == running['id'] and worker == running['worker']:
                    return {'job': self.packet(running)}
                return {'job': None}
            if selected is not None:
                row = db.execute("SELECT * FROM jobs WHERE status='queued' AND id=?", (selected,)).fetchone()
                if row is None:
                    raise DispatchError('Selected job is no longer queued', 409)
            else:
                row = db.execute("SELECT * FROM jobs WHERE status='queued' ORDER BY sequence LIMIT 1").fetchone()
            if row is None:
                return {'job': None}
            db.execute("UPDATE jobs SET status='running',worker=?,updated=? WHERE id=?", (worker, time.time(), row['id']))
            return {'job': self.packet(db.execute('SELECT * FROM jobs WHERE id=?', (row['id'],)).fetchone())}

    def switch(self, body):
        """After navigation has stopped, atomically requeue old and claim new."""
        worker = label(body.get('worker'), 'worker')
        current = label(body.get('current_job_id'), 'current_job_id')
        selected = label(body.get('next_job_id'), 'next_job_id')
        if current == selected:
            raise DispatchError('Select a different queued job')
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            old = db.execute('SELECT * FROM jobs WHERE id=?', (current,)).fetchone()
            new = db.execute('SELECT * FROM jobs WHERE id=?', (selected,)).fetchone()
            if old is None or new is None:
                raise DispatchError('Job not found', 404)
            # Retrying a switch whose response was lost never claims twice.
            if old['status'] == 'queued' and new['status'] == 'running' and new['worker'] == worker:
                return {'job': self.packet(new), 'paused_job_id': current}
            if old['status'] != 'running' or old['worker'] != worker or new['status'] != 'queued':
                raise DispatchError('Current job ownership or next job state changed', 409)
            now = time.time()
            db.execute("UPDATE jobs SET status='queued',worker=NULL,updated=?,detail=? WHERE id=?",
                       (now, 'Paused to serve a nearer call', current))
            db.execute("UPDATE jobs SET status='running',worker=?,updated=?,detail='' WHERE id=?",
                       (worker, now, selected))
            return {'job': self.packet(db.execute('SELECT * FROM jobs WHERE id=?', (selected,)).fetchone()),
                    'paused_job_id': current}

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
