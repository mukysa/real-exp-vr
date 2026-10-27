"""Exercise the real bridge with synthetic ROS data on domain 173 (no hardware)."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import zmq

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['normal', 'action', 'state', 'camera', 'future'], default='normal')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sample-python', help='Optional LeRobot Python for full schema audit (normal scenario).')
    args = parser.parse_args()
    if args.sample_python and args.scenario != 'normal':
        parser.error('--sample-python is supported for the normal scenario only')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # XML below enforces loopback; ROS_LOCALHOST_ONLY=1 adds lo a second
    # time in Humble's Cyclone RMW and prevents domain creation.
    env.update(ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='0')
    env.pop('ROS_AUTOMATIC_DISCOVERY_RANGE', None)
    env.pop('ROS_STATIC_PEERS', None)
    # Isolated tests must not inherit a real-network interface or stale IP.
    env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    env['CYCLONEDDS_URI'] = '''<CycloneDDS xmlns="https://cdds.io/config">
      <Domain Id="any"><General>
        <Interfaces><NetworkInterface name="lo"/></Interfaces>
        <AllowMulticast>false</AllowMulticast>
      </General><Discovery><ParticipantIndex>auto</ParticipantIndex>
        <Peers><Peer Address="127.0.0.1"/></Peers>
      </Discovery></Domain></CycloneDDS>'''
    window = 45 if args.sample_python else 13
    bridge = ROOT / 'gello_software/ros2/src/franka_lerobot_data_bridge/franka_lerobot_data_bridge/lerobot_data_bridge.py'
    command = [sys.executable, '-c', 'import runpy,sys; runpy.run_path(sys.argv.pop(1))["main"]()', str(bridge),
               '--ros-args', '--params-file', str(ROOT / 'config/data_collection/quest_left.yaml'),
               '-p', 'publish_host:=127.0.0.1', '-p', 'publish_port:=15555',
               '-p', 'camera_cache_host:=127.0.0.1', '-p', 'camera_cache_port:=15557',
               '-p', 'deployment_mode:=false']
    for topic in ('/left/joint_states', '/left/vr/accepted_joint_states',
                  '/left/franka_robot_state_broadcaster/robot_state',
                  '/cameras/cam_left/image_raw', '/cameras/cam_front/image_raw'):
        command += ['-r', f'{topic}:=/m2{topic}']
    fixture = [sys.executable, '-m', 'data_collection.diagnostics.synthetic_ros', '--duration', str(window + 5),
               '--drop-after', '7']
    if args.scenario in ('action', 'state', 'camera'):
        fixture += ['--drop', args.scenario]
    elif args.scenario == 'future':
        fixture += ['--stamp-offset', '5']
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.SUBSCRIBE, b'')
    subscriber.connect('tcp://127.0.0.1:15555')
    children, logs = [], []
    packets = []
    failure = None
    checker = None
    passed = False
    exit_codes_before_cleanup = {}
    start = time.monotonic()
    try:
        for name, cmd in [('bridge', command), ('fixture', fixture)]:
            log = (args.output_dir / f'{name}.log').open('w')
            logs.append(log)
            children.append(subprocess.Popen(cmd, env=env, cwd=ROOT, stdout=log,
                                             stderr=subprocess.STDOUT, start_new_session=True))
        if args.sample_python:
            log = (args.output_dir / 'schema-audit.log').open('w')
            logs.append(log)
            checker = subprocess.Popen([
                args.sample_python, '-m', 'data_collection.diagnostics.samples',
                '--port', '15555', '--duration', '8', '--output',
                str(args.output_dir / 'schema-audit.json')], cwd=ROOT, env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while time.monotonic() - start < window:
            if any(child.poll() is not None for child in children):
                failure = 'child exited early; inspect logs'
                break
            if subscriber.poll(200):
                packet = subscriber.recv_pyobj()
                # Payload schema is additionally checked with samples.py under LeRobot Python.
                if (packet.get('teleop_source') != 'quest' or len(packet.get('target_joint', [])) != 7
                        or set(packet.get('cameras', {})) != {'cam_left', 'cam_front'}
                        or 'left' not in packet.get('action_freshness', {})):
                    failure = 'invalid bridge contract'
                    break
                packets.append(time.monotonic() - start)
            if checker is not None and checker.poll() is not None:
                if checker.returncode != 0:
                    failure = 'schema audit exited nonzero; inspect schema-audit.log'
                    break
                if time.monotonic() - start >= 13:
                    break
        if args.scenario == 'future':
            passed = not packets
        elif args.scenario == 'normal':
            passed = len(packets) >= 30 and packets[-1] > 11
        else:
            passed = len(packets) >= 20 and packets[-1] < 10
        if checker is not None and failure is None:
            code = checker.poll()
            if code is None:
                failure = 'schema audit timed out; inspect schema-audit.log'
            elif code != 0:
                failure = 'schema audit exited nonzero; inspect schema-audit.log'
        passed = passed and failure is None
    finally:
        exit_codes_before_cleanup = {
            name: child.poll() for name, child in zip(('bridge', 'fixture'), children)
        }
        if checker is not None:
            exit_codes_before_cleanup['schema_audit'] = checker.poll()
            children.append(checker)
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
        subscriber.close(linger=0)
        context.term()
    result = dict(scenario=args.scenario, passed=passed, failure=failure,
                  exit_codes_before_cleanup=exit_codes_before_cleanup,
                  exit_codes_after_cleanup={
                      name: child.returncode for name, child in
                      zip(('bridge', 'fixture', 'schema_audit'), children)},
                  received=len(packets), first_receive_s=packets[0] if packets else None,
                  last_receive_s=packets[-1] if packets else None)
    (args.output_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
