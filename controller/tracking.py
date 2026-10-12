"""Pose publication independent of cloud mission scheduling and model inference."""
class Tracker:
    def __init__(self, api, worker):
        self.api, self.worker = api, worker
        self.map_sha256 = None

    def publish(self, packet):
        if packet is None:
            return None
        if self.map_sha256 is None:
            self.map_sha256 = self.api.get('/api/map')['sha256']
        return self.api.post('/api/pi/pose', {**packet, 'worker': self.worker, 'map_sha256': self.map_sha256})
