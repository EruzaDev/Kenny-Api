"""Pi queue consumer. HTTP uses stdlib; ROS is only imported for Nav2 mode.

Model controller contract: JSON job on stdin, JSON {status, detail} on stdout.
Terminal status means the controller has stopped and can accept the next job.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import time
from urllib.request import Request, urlopen
from urllib.error import URLError


class API:
    def __init__(self, url, token=''):
        if not url.startswith(('http://', 'https://')):
            raise ValueError('API URL must start with http:// or https://')
        self.url, self.token = url.rstrip('/'), token

    def post(self, path, payload):
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['Authorization'] = f'Bearer {self.token}'
        req = Request(self.url+path, data=json.dumps(payload).encode(),
                      headers=headers)
        with urlopen(req, timeout=15) as response:
            return json.load(response)


def save(path, value):
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def command_controller(command, job, timeout):
    result = subprocess.run(command, input=json.dumps(job), text=True,
                            stdout=subprocess.PIPE, timeout=timeout, check=True)
    packet = json.loads(result.stdout)
    if packet.get('status') not in ('succeeded', 'failed'):
        raise ValueError('Controller must return succeeded or failed after stopping')
    detail = packet.get('detail', '')
    if not isinstance(detail, str) or len(detail) > 2000:
        raise ValueError('Controller detail must be a string of at most 2000 characters')
    return {'status': packet['status'], 'detail': detail}


class Nav2Controller:
    def __init__(self):
        import rclpy
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
        from geometry_msgs.msg import PoseStamped
        rclpy.init()
        self.nav = BasicNavigator()
        self.result = TaskResult
        self.pose = PoseStamped
        # Existing localization must already use the exported map. No fake pose.
        self.nav.waitUntilNav2Active()

    def __call__(self, job, timeout):
        goal = job['destination']
        pose = self.pose()
        pose.header.frame_id = goal['frame_id']
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(goal['x']), float(goal['y'])
        pose.pose.orientation.z = math.sin(goal['yaw']/2)
        pose.pose.orientation.w = math.cos(goal['yaw']/2)
        accepted = self.nav.goToPose(pose)
        if accepted is False:
            return {'status': 'failed', 'detail': 'Nav2 rejected destination'}
        start = time.monotonic()
        try:
            while not self.nav.isTaskComplete():
                if time.monotonic()-start > timeout:
                    self.nav.cancelTask()
                    deadline = time.monotonic()+15
                    while not self.nav.isTaskComplete():
                        if time.monotonic() > deadline:
                            raise RuntimeError('Nav2 cancellation unconfirmed; dispatch stays blocked')
                    return {'status': 'failed', 'detail': 'Navigation timed out and was canceled'}
                time.sleep(.1)
        except BaseException:
            self.nav.cancelTask()
            raise
        return {'status': 'succeeded' if self.nav.getResult() == self.result.SUCCEEDED else 'failed',
                'detail': 'Nav2 navigation completed'}


def consume(api, worker, state_path, controller, timeout, once=False):
    state_path.parent.mkdir(parents=True, exist_ok=True)
    # The lock lives as long as this process. Two local receivers cannot drive.
    with state_path.with_suffix('.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = json.loads(state_path.read_text()) if state_path.exists() else None
        if state and state['phase'] == 'running':
            raise RuntimeError(f"Interrupted job {state['job']['id']}; confirm robot stopped, report failed via API, then remove {state_path}. No automatic replay.")
        while True:
            try:
                if state is None:
                    job = api.post('/api/pi/claim', {'worker': worker})['job']
                    if job is None:
                        if once:
                            return
                        time.sleep(2)
                        continue
                    state = {'phase': 'running', 'job': job, 'worker': worker}
                    save(state_path, state)
                    print(f"Navigating to marker {job['marker_id']} · {job['id']}", flush=True)
                    # Errors with unknown motion state leave the server blocked.
                    outcome = controller(job, timeout)
                    state = {**state, 'phase': 'finished', 'outcome': outcome}
                    save(state_path, state)
                api.post(f"/api/calls/{state['job']['id']}/result",
                         {**state['outcome'], 'worker': state['worker']})
                print(f"Job {state['job']['id']}: {state['outcome']['status']}", flush=True)
                state_path.unlink()
                state = None
                if once:
                    return
            except (URLError, TimeoutError, ConnectionError) as error:
                # Navigation is never repeated to retry a result upload.
                if state and state['phase'] == 'running':
                    raise RuntimeError('Controller interrupted; queue remains blocked') from error
                print(f'API unavailable: {error}; retrying in 5 seconds', flush=True)
                time.sleep(5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default=os.environ.get('KENNY_API_URL', 'https://kenny-api.onrender.com'),
                        help='API base URL (default: KENNY_API_URL or https://kenny-api.onrender.com)')
    parser.add_argument('--worker', default='kenny-pi')
    parser.add_argument('--mode', choices=['nav2', 'model'], required=True)
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--state', type=Path, default=Path('data/pi-receiver.json'))
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--command', nargs=argparse.REMAINDER, help='Model controller executable and arguments; put last')
    args = parser.parse_args()
    token = os.environ.get('KENNY_PI_TOKEN') or os.environ.get('KENNY_DISPATCH_TOKEN', '')
    if args.timeout <= 0 or not math.isfinite(args.timeout):
        parser.error('--timeout must be positive and finite')
    if args.mode == 'model' and not args.command:
        parser.error('--mode model requires --command')
    controller = Nav2Controller() if args.mode == 'nav2' else lambda job, timeout: command_controller(args.command, job, timeout)
    consume(API(args.api, token), args.worker, args.state, controller, args.timeout, args.once)


if __name__ == '__main__':
    main()
