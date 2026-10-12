"""ROS velocity/encoder adapter for KENNY_USB_V1 (LF, RF, LR, RR)."""
import argparse
import json
import math
import threading
import time

def signed_delta(value, previous):
    return (int(value) - int(previous) + 2**31) % 2**32 - 2**31


class WheelOdometry:
    def __init__(self, radius=.04, track=.235):
        if not math.isfinite(radius) or not math.isfinite(track) or radius <= 0 or track <= 0:
            raise ValueError('Wheel radius and track must be positive and finite')
        self.radius, self.track = radius, track
        self.previous = None
        self.pose = [0., 0., 0.]
        self.distance = 0.

    def update(self, status):
        if not status.get('encodersEnabled') or status.get('cpr', 0) <= 0:
            raise ValueError('Autonomous odometry requires real, enabled encoders')
        if len(status['counts']) != 4:
            raise ValueError('Expected four encoder counters')
        previous, self.previous = self.previous, status
        if previous is None:
            return None
        if status['boot'] != previous['boot']:
            raise RuntimeError('ESP32 rebooted; stop and restart localization before continuing')
        elapsed = (status['uptime'] - previous['uptime']) % 2**32 / 1000
        if not 0 < elapsed < 1:
            raise ValueError('Stale encoder telemetry')
        meters = [signed_delta(a, b) * 2 * math.pi * self.radius / status['cpr']
                  for a, b in zip(status['counts'], previous['counts'])]
        if any(abs(d) > 2 * elapsed for d in meters):
            raise RuntimeError('Encoder discontinuity; stopping')
        left, right = (meters[0] + meters[2]) / 2, (meters[1] + meters[3]) / 2
        distance, angle = (left + right) / 2, (right - left) / self.track
        arc = distance if abs(angle) < 1e-9 else distance * math.sin(angle / 2) / (angle / 2)
        self.pose[0] += arc * math.cos(self.pose[2] + angle / 2)
        self.pose[1] += arc * math.sin(self.pose[2] + angle / 2)
        self.pose[2] = math.atan2(math.sin(self.pose[2] + angle), math.cos(self.pose[2] + angle))
        self.distance += abs(distance)
        return list(self.pose), [distance / elapsed, angle / elapsed]


class SerialLink:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=.03, write_timeout=.15, exclusive=True)
        time.sleep(2)
        self.serial.reset_input_buffer()
        if self.request('HELLO').get('protocol') != 'KENNY_USB_V1':
            self.serial.close()
            raise RuntimeError('Not a KENNY_USB_V1 motor controller')
        self.request('STOP')

    def request(self, command):
        self.serial.write((command + '\n').encode('ascii'))
        deadline = time.monotonic() + .2
        while time.monotonic() < deadline:
            try:
                packet = json.loads(self.serial.readline())
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(packet, dict):
                if packet.get('ok') is False:
                    raise RuntimeError(packet.get('error', 'Motor command rejected'))
                return packet
        self.serial.write(b'STOP\n')
        self.serial.close()
        raise RuntimeError('Motor reply timeout; connection closed to discard late replies')

    def close(self):
        if self.serial.is_open:
            try:
                self.request('STOP')
            finally:
                self.serial.close()


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from tf2_ros import TransformBroadcaster
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--wheel-radius', type=float, default=.04)
    parser.add_argument('--track', type=float, default=.235)
    args, ros_args = parser.parse_known_args()
    odometry = WheelOdometry(args.wheel_radius, args.track)
    rclpy.init(args=ros_args)
    node = Node('kenny_motor_bridge')
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    status_pub = node.create_publisher(String, '/kenny/motors/status', 10)
    broadcaster = TransformBroadcaster(node)
    mutex = threading.Lock()
    command, allowed = [0., 0.], [False, -math.inf]
    received = [-math.inf]
    stopped = threading.Event()

    def velocity(msg):
        age = node.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        values = [msg.twist.linear.x, msg.twist.angular.z]
        if not -.1 <= age <= .2 or not all(math.isfinite(v) for v in values) or abs(values[0]) > .1 or abs(values[1]) > .6:
            return
        with mutex:
            command[:] = values
            received[0] = time.monotonic()

    def permission(msg):
        with mutex:
            allowed[:] = [msg.data, time.monotonic()]

    node.create_subscription(TwistStamped, '/kenny/cmd_vel', velocity, 10)
    node.create_subscription(Bool, '/kenny/motion_allowed', permission, 10)

    def run():
        link = None
        try:
            link = SerialLink(args.port)
            lease, sequence, last_status = None, 0, -math.inf
            telemetry = {}
            while not stopped.is_set() and rclpy.ok():
                begin = time.monotonic()
                if begin - last_status >= .1:
                    telemetry = link.request('STATUS')
                    last_status = begin
                    status_pub.publish(String(data=json.dumps(telemetry)))
                    measured = odometry.update(telemetry)
                    if measured is not None:
                        pose, twist = measured
                        msg = Odometry()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
                        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
                        msg.twist.twist.linear.x, msg.twist.twist.angular.z = twist
                        # Configurable/calibratable wheel-slip uncertainty, not perfect pose.
                        msg.pose.covariance[0] = msg.pose.covariance[7] = .0004 + .01 * odometry.distance
                        msg.pose.covariance[35] = .001 + .01 * odometry.distance
                        msg.twist.covariance[0], msg.twist.covariance[35] = .0025, .01
                        odom_pub.publish(msg)
                        tf = TransformStamped()
                        tf.header, tf.child_frame_id = msg.header, 'base_link'
                        tf.transform.translation.x, tf.transform.translation.y = pose[:2]
                        tf.transform.rotation = msg.pose.pose.orientation
                        broadcaster.sendTransform(tf)
                with mutex:
                    permitted = allowed[0] and begin - allowed[1] < .25 and begin - received[0] < .25
                    v, w = command if permitted else [0., 0.]
                permitted = permitted and telemetry.get('ready', False) and not telemetry.get('faults') and not telemetry.get('estop')
                if not permitted:
                    if lease is not None:
                        link.request('STOP')
                        lease = None
                else:
                    if lease is None:
                        lease = link.request('ARM')['lease']
                        sequence = 0
                    sequence = (sequence + 1) % 2**32
                    link.request(f'V {lease} {sequence} {v:.5f} {w:.5f}')
                stopped.wait(max(0., .05 - (time.monotonic() - begin)))
        except Exception as error:
            node.get_logger().error(f'Motor bridge stopped: {error}')
        finally:
            if link is not None:
                link.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        thread.join(timeout=3.)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
