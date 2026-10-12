"""Durable cloud scheduling around a resident navigation driver."""
import fcntl
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen

from pi.receiver import API, save
from .planning import nearest


class CloudAPI(API):
    def get(self, path):
        headers = {'Authorization': f'Bearer {self.token}'} if self.token else {}
        with urlopen(Request(self.url + path, headers=headers), timeout=15) as response:
            return json.load(response)


class Missions:
    def __init__(self, api, driver, state_path, worker='kenny-main', margin=.25, timeout=1800):
        self.api, self.driver, self.path = api, driver, Path(state_path)
        self.worker, self.margin, self.timeout = worker, margin, timeout
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = self.path.with_suffix('.lock').open('w')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.state = json.loads(self.path.read_text()) if self.path.exists() else None
        if self.state is not None:
            self.worker = self.state['worker']
        if self.state and self.state['phase'] in ('running', 'switching'):
            self.lock.close()
            raise RuntimeError(f'Interrupted physical mission in {self.path}; stop the robot and resolve the recorded API job before removing this journal')
        self.started = None

    def close(self):
        self.lock.close()

    def write(self):
        save(self.path, self.state)

    def step(self, now=None):
        now = time.monotonic() if now is None else now
        if self.state and self.state['phase'] == 'finished':
            self.api.post(f"/api/calls/{self.state['job']['id']}/result",
                          {'worker': self.state['worker'], **self.state['outcome']})
            self.path.unlink()
            self.state = None
            return 'Result delivered; waiting for calls'
        if self.state and self.state['phase'] == 'claiming':
            response = self.api.post('/api/pi/claim', {'worker': self.worker, 'job_id': self.state['job']['id']})
            if response['job'] is None:
                self.path.unlink()
                self.state = None
                return 'Another receiver owns the running trip'
            self.activate(response['job'], now)
            return 'Selected call claimed'
        if self.state and self.state['phase'] == 'switching':
            response = self.api.post('/api/pi/switch', {'worker': self.worker,
                'current_job_id': self.state['job']['id'], 'next_job_id': self.state['next_job']['id']})
            self.activate(response['job'], now)
            return 'Switched to nearer call; previous call remains queued'
        snapshot = self.driver.snapshot()
        if not snapshot['ready']:
            return f"Waiting: {snapshot['reason']}"
        jobs = self.api.get('/api/pi/queue')['jobs']
        if self.state is None:
            selected = nearest(jobs, snapshot['pose'], self.driver.destination)
            if selected is None:
                return 'Waiting for a registered ArUco call'
            self.state = {'phase': 'claiming', 'job': selected, 'worker': self.worker}
            self.write()
            self.step(now)
            return f"Navigating to ArUco ID {selected['marker_id']}"
        current = self.state['job']
        if self.driver.arrived():
            self.finish('succeeded', 'Reached destination with live pose; robot stopped')
            return 'Destination reached'
        if now - self.started > self.timeout:
            self.finish('failed', 'Trip timed out; robot stopped')
            return 'Trip timed out'
        selected = nearest(jobs, snapshot['pose'], self.driver.destination, current, self.margin)
        if selected is not None:
            # Stop and confirm before changing server ownership. HTTP retries
            # cannot accidentally resurrect the old physical navigation goal.
            self.driver.stop()
            self.state = {**self.state, 'phase': 'switching', 'next_job': selected}
            self.write()
            return self.step(now)
        return f"Navigating to ArUco ID {current['marker_id']}"

    def activate(self, job, now):
        self.state = {'phase': 'running', 'job': job, 'worker': self.worker}
        self.write()
        self.started = now
        self.driver.set_goal(job)

    def finish(self, status, detail):
        self.driver.stop()
        self.state = {**self.state, 'phase': 'finished', 'outcome': {'status': status, 'detail': detail}}
        self.write()
