"""Inspect trusted bridge ZMQ samples without creating a dataset or commands."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import time

import numpy as np


def summary(values):
    if not values:
        return None
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, [0, 50, 95, 99, 100]))))


class SampleAudit:
    def __init__(self, validator, max_age=0.5, max_camera_skew=0.067):
        self.validator = validator
        self.max_age = max_age
        self.max_camera_skew = max_camera_skew
        self.count = 0
        self.valid = 0
        self.errors = Counter()
        self.metrics = defaultdict(list)
        self.first_receive = None
        self.last_receive = None
        self.last_stamp = None
        self.last_target = None
        self.stamp_updates = 0
        self.target_changes = 0
        self.latest_bridge_diagnostics = None

    def add(self, packet, receive_monotonic, receive_wall):
        self.count += 1
        if self.last_receive is not None:
            self.metrics['receive_interval_s'].append(receive_monotonic - self.last_receive)
        else:
            self.first_receive = receive_monotonic
        self.last_receive = receive_monotonic
        try:
            ready, reason = self.validator(packet)
            if not ready:
                raise ValueError(reason)
            if (packet.get('teleop_source') != 'quest' or packet.get('arm_mode') != 'left'
                    or packet.get('include_hand')):
                raise ValueError('expected Quest left arm/gripper sample')
            if set(packet['camera_names']) != {'cam_left'}:
                raise ValueError('expected cam_left only')
            expected_width = 8 if packet.get('include_gripper') else 7
            for key in ('state', 'action', 'joint_state', 'target_joint'):
                vector = np.asarray(packet[key])
                if vector.shape != (expected_width,) or not np.isfinite(vector).all():
                    raise ValueError(f'invalid {key}')
            publish = float(packet['bridge_publish_s'])
            action_stamp = float(packet['action_freshness']['left']['stamp_s'])
            state_stamp = float(packet['robot_state_freshness']['left_arm']['stamp_s'])
            camera_stamps = [float(packet['cameras'][name]['stamp_s']) for name in packet['camera_names']]
            stamps = [publish, action_stamp, state_stamp, *camera_stamps]
            if not all(math.isfinite(x) and x > 0 for x in stamps):
                raise ValueError('invalid timestamps')
            ages = [publish - stamp for stamp in stamps[1:]]
            if min(ages) < -0.1:
                raise ValueError('future source timestamps: check clock alignment')
            if max(ages) > self.max_age:
                raise ValueError('stale source timestamps')
            if max(camera_stamps) - min(camera_stamps) > self.max_camera_skew:
                raise ValueError('camera skew exceeds threshold')
            transport_age = receive_wall - publish
            if not math.isfinite(transport_age) or transport_age < -0.1 or transport_age > self.max_age:
                raise ValueError('stale/future bridge packet: check clocks or receiver backlog')
            if self.last_stamp is not None and action_stamp < self.last_stamp:
                raise ValueError('action timestamp regressed')
            self.metrics['action_age_s'].append(publish - action_stamp)
            self.metrics['state_age_s'].append(publish - state_stamp)
            self.metrics['state_action_skew_s'].append(action_stamp - state_stamp)
            self.metrics['camera_skew_s'].append(max(camera_stamps) - min(camera_stamps))
            self.metrics['receive_wall_minus_publish_s'].append(transport_age)
            if self.last_stamp is None or action_stamp != self.last_stamp:
                self.stamp_updates += 1
            target = np.asarray(packet['target_joint'])
            if self.last_target is not None and not np.array_equal(target, self.last_target):
                self.target_changes += 1
            self.last_stamp = action_stamp
            self.last_target = target.copy()
            self.latest_bridge_diagnostics = packet.get('bridge_diagnostics')
            self.valid += 1
        except (KeyError, TypeError, ValueError, IndexError, OverflowError) as exc:
            self.errors[str(exc)] += 1

    def report(self):
        elapsed = (self.last_receive - self.first_receive) if self.count > 1 else 0
        return {
            'received': self.count, 'valid': self.valid, 'invalid': self.count - self.valid,
            'errors': dict(self.errors),
            'observed_rate_hz': (self.count - 1) / elapsed if elapsed > 0 else None,
            'action_stamp_updates_observed': self.stamp_updates,
            'target_value_changes_observed': self.target_changes,
            'metrics': {name: summary(values) for name, values in self.metrics.items()},
            'latest_bridge_diagnostics': self.latest_bridge_diagnostics,
            'time_semantics': 'Intervals use receiver monotonic time only. Wall time minus publish time includes clock offset; it is not a measured network latency. Action updates are sampled, not the original publisher rate.',
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=5555)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--silence-timeout', type=float, default=5)
    parser.add_argument('--max-age', type=float, default=0.5)
    parser.add_argument('--min-rate', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if min(args.duration, args.silence_timeout, args.max_age, args.min_rate) <= 0:
        parser.error('duration, timeouts, age and rate must be positive')
    import zmq
    from data_collection.lerobot_collection import bridge_packet_readiness
    audit = SampleAudit(bridge_packet_readiness, max_age=args.max_age)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b'')
    socket.setsockopt(zmq.RCVHWM, 10)
    socket.connect(f'tcp://{args.host}:{args.port}')
    start = last = time.monotonic()
    stop_reason = 'duration_complete'
    print(f'Inspecting tcp://{args.host}:{args.port}; press Ctrl+C to save partial report.', flush=True)
    try:
        while time.monotonic() - start < args.duration:
            if socket.poll(200):
                # Existing bridge transport uses pickle: connect only to a trusted bridge.
                packet = socket.recv_pyobj()
                last = time.monotonic()
                audit.add(packet, last, time.time())
            if time.monotonic() - last > args.silence_timeout:
                stop_reason = 'sample_stream_timeout'
                break
    except KeyboardInterrupt:
        stop_reason = 'interrupted'
    finally:
        socket.close(linger=0)
        context.term()
    report = audit.report()
    report.update(stop_reason=stop_reason, duration_s=time.monotonic() - start)
    report['passed'] = (stop_reason == 'duration_complete' and report['valid'] >= 2
                        and report['invalid'] == 0
                        and (report['observed_rate_hz'] or 0) >= args.min_rate
                        and report['metrics'].get('receive_interval_s', {}).get('max', math.inf) <= args.silence_timeout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
