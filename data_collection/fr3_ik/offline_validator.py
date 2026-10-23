"""Run offline synthetic scenarios and generate a self-contained HTML report."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
import scipy
import pinocchio
from scipy.spatial.transform import Rotation

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.models import TargetPose
from .constraints import Limits, residual
from .interfaces import BASE_FRAME, ClockDomain, IKRequest, JointTarget, JointVector, Timestamp
from .kinematics import Kinematics
from .solver import ContinuousIK
from .trajectories import Q0, NORMAL, scenarios


def request_for(sample, sequence, clock_id):
    stamp = Timestamp(sample.time, ClockDomain.REPLAY, clock_id)
    hold = sample.event == 'hold'
    target = TargetPose(RigidPose(tuple(sample.pose[:3, 3]),
                                  tuple(Rotation.from_matrix(sample.pose[:3, :3]).as_quat())),
                        BASE_FRAME, sequence, sample.connection, sample.time,
                        'MANUAL_HOLD' if hold else 'FOLLOWING', not hold,
                        'manual_hold' if hold else None)
    return IKRequest(target, stamp, stamp)


def trajectory_digest(rows):
    excluded = {'solve_time_s', 'period_exceeded'}
    stable = [{k: v for k, v in row.items() if k not in excluded} for row in rows]
    return hashlib.sha256(json.dumps(stable, sort_keys=True, allow_nan=False).encode()).hexdigest()


def run_scenario(name, samples, kin, limits, hz, max_evaluations, timeout_s=None):
    solver = ContinuousIK(kin, limits, max_evaluations, timeout_s)
    clock = 'synthetic/' + name
    solver.reset(JointTarget(JointVector(Q0), Timestamp(0, ClockDomain.REPLAY, clock)), 1)
    rows = []
    previous_q = np.array(Q0)
    previous_velocity = np.zeros(7)
    for index, sample in enumerate(samples):
        if sample.event == 'reset':
            # Explicit local reanchor: preserve checked joints, discard old desired pose.
            sample = replace(sample, pose=kin.fk(solver.last.joints.positions_rad))
            solver.reset(JointTarget(solver.last.joints, Timestamp(sample.time-1/hz, ClockDomain.REPLAY, clock)), sample.connection)
        request = request_for(sample, index+1, clock)
        if sample.event == 'invalid':
            try:
                RigidPose((float('nan'), 0, 0), (0, 0, 0, 1))
            except ValueError:
                result = solver.reject_input(request, 'Injected non-finite raw position rejected before IKRequest')
            else:
                raise AssertionError('Non-finite input was not rejected')
        else:
            result = solver.solve(request, solver.last, None)
        q = np.array(result.last_safe_target.joints.positions_rad)
        points, actual = kin.geometry(q)
        error = residual(actual, sample.pose)
        dt = sample.time - (rows[-1]['time'] if rows else 0)
        velocity = (q - previous_q)/dt
        acceleration = (velocity-previous_velocity)/dt
        row = {
            'scenario': name, 'index': index, 'time': sample.time, 'event': sample.event,
            'connection_id': sample.connection, 'sequence': index+1,
            'state': result.state.value, 'reason': result.reason.value, 'detail': result.detail,
            'success': result.success, 'q': q.tolist(), 'velocity': velocity.tolist(),
            'acceleration': acceleration.tolist(), 'desired': sample.pose.tolist(),
            'actual': actual.tolist(), 'points': points,
            'position_error_m': error[0], 'rotation_error_rad': error[1],
            'candidate_position_error_m': result.position_residual_m,
            'candidate_rotation_error_rad': result.rotation_residual_rad,
            'solve_time_s': result.solve_time_s, 'period_exceeded': result.solve_time_s > 1/hz,
            'checks': asdict(result.checks), **solver.diagnostics,
        }
        candidate = row['candidate_q']
        row['candidate_points'] = kin.geometry(candidate)[0] if candidate is not None and not result.success else None
        rows.append(row)
        previous_q, previous_velocity = q, velocity
    return rows


def assess(name, rows, limits):
    successful = [r for r in rows if r['success']]
    failed = [r for r in rows if not r['success']]
    hold_constant = all(r['q'] == rows[i-1]['q'] for i, r in enumerate(rows) if i and not r['success'])
    passed = hold_constant
    if name in NORMAL:
        passed = passed and len(successful) == len(rows)
        if name != 'static':
            passed = passed and np.max(np.abs(np.array([r['q'] for r in rows])-Q0)) > .001
        else:
            passed = passed and np.max(np.abs(np.array([r['q'] for r in rows])-Q0)) < 1e-8
    else:
        expected = {'unreachable': 'unreachable', 'jump': 'step_limit', 'invalid': 'invalid_input',
                    'interruption': 'stale_input', 'reconnect': 'connection_changed', 'manual_hold': 'none'}[name]
        passed = passed and len(failed) >= 20 and rows[20]['reason'] == expected and not rows[20]['success']
        if name in ('interruption', 'reconnect', 'manual_hold'):
            passed = passed and all(r['success'] for r in rows[40:])
        else:
            passed = passed and all(not r['success'] for r in rows[20:])
    successful_errors = [r['position_error_m'] for r in successful]
    return {
        'passed': bool(passed), 'frames': len(rows), 'successful': len(successful),
        'states': dict(Counter(r['state'] for r in rows)),
        'reasons': dict(Counter(r['reason'] for r in failed)), 'hold_target_unchanged': hold_constant,
        'max_success_position_error_m': max(successful_errors, default=0),
        'max_success_rotation_error_rad': max((r['rotation_error_rad'] for r in successful), default=0),
        'max_success_joint_speed_rad_s': max((max(abs(x) for x in r['velocity']) for r in successful), default=0),
        'max_success_joint_acceleration_rad_s2': max((max(abs(x) for x in r['acceleration']) for r in successful), default=0),
        'max_joint_step_rad': max((max(abs(a-b) for a,b in zip(rows[i]['q'], rows[i-1]['q'])) for i in range(1,len(rows))), default=0),
        'endpoint_joint_difference_rad': max(abs(a-b) for a,b in zip(rows[-1]['q'], Q0)),
        'trajectory_sha256': trajectory_digest(rows),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--hz', type=float, default=50)
    parser.add_argument('--duration', type=float, default=4)
    parser.add_argument('--max-evaluations', type=int, default=80)
    parser.add_argument('--scenarios', nargs='+', help='Selected scenario names; default all')
    parser.add_argument('--verify-repeat', action='store_true')
    parser.add_argument('--timeout-ms', type=float, help='Optional post-return wall-clock rejection (not deterministic)')
    args = parser.parse_args()
    if not np.isfinite(args.hz) or not 10 <= args.hz <= 200 or not np.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error('hz must be 10..200 and duration 1..60 seconds')
    if args.max_evaluations < 1 or (args.timeout_ms is not None and (not np.isfinite(args.timeout_ms) or args.timeout_ms <= 0)):
        parser.error('Invalid iteration/time budget')
    if args.verify_repeat and args.timeout_ms is not None:
        parser.error('Determinism verification and wall-clock rejection are separate modes')
    if args.output.exists():
        parser.error('Output already exists; use a new directory')
    start = time.perf_counter()
    kin = Kinematics()
    limits = Limits.from_controller()
    available = dict(scenarios(kin.fk(Q0), args.hz, args.duration))
    selected = args.scenarios or list(available)
    if not selected or len(set(selected)) != len(selected) or any(n not in available for n in selected):
        parser.error('Unknown or duplicate scenario; choose ' + ', '.join(available))
    # Warm lazy imports and solver code before recording benchmark samples.
    run_scenario('warmup', available['static'][:5], kin, limits, args.hz, args.max_evaluations)
    initialization_s = time.perf_counter()-start
    args.output.mkdir(parents=True, exist_ok=False)
    all_rows, summaries = [], {}
    for name in selected:
        rows = run_scenario(name, available[name], kin, limits, args.hz, args.max_evaluations,
                            None if args.timeout_ms is None else args.timeout_ms/1000)
        summary = assess(name, rows, limits)
        if args.verify_repeat:
            repeat = run_scenario(name, available[name], kin, limits, args.hz, args.max_evaluations)
            summary['repeat_identical'] = trajectory_digest(rows) == trajectory_digest(repeat)
            summary['passed'] &= summary['repeat_identical']
        summaries[name] = summary
        all_rows.extend(rows)
        print(f'{name}: {summary["successful"]}/{summary["frames"]} following, passed={summary["passed"]}', flush=True)
    times = np.array([r['solve_time_s'] for r in all_rows if r['ik_attempted']])
    summary = {
        'passed': all(s['passed'] for s in summaries.values()), 'scope': 'A2 synthetic offline references only',
        'robot_connected': False, 'measured_state_available': False, 'hz': args.hz,
        'duration_s': args.duration, 'q0_rad': Q0, 'limits': asdict(limits),
        'max_evaluations': args.max_evaluations, 'post_return_timeout_ms': args.timeout_ms,
        'initialization_and_warmup_s': initialization_s, 'model': kin.provenance,
        'python': platform.python_version(), 'numpy': np.__version__,
        'scipy': scipy.__version__, 'pinocchio': pinocchio.__version__,
        'determinism_verified': args.verify_repeat,
        'source_hashes': {str(path.relative_to(Path(__file__).resolve().parents[2])):
                          hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted(Path(__file__).parent.glob('*.py'))},
        'timing_scope': 'solve call including checks; excludes visualization, logging, ROS and hardware',
        'solver_time_ms': {k: float(v*1000) for k,v in zip(('p50','p95','p99','max'),
                             np.r_[np.percentile(times, [50,95,99]), max(times)])} if len(times) else {},
        'attempted_solves': len(times), 'solve_period_overruns': int(np.sum(times > 1/args.hz)),
        'scenarios': summaries,
    }
    (args.output/'samples.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in all_rows))
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    template_dir = Path(__file__).parent/'visualization'
    data = json.dumps({'summary': summary, 'samples': all_rows}, allow_nan=False).replace('<', '\\u003c')
    html = (template_dir/'viewer.html').read_text().replace('/*REPORT_DATA*/', data).replace('/*VIEWER_JS*/', (template_dir/'viewer.js').read_text())
    (args.output/'report.html').write_text(html)
    print(json.dumps({'passed': summary['passed'], 'report': str(args.output/'report.html'),
                      'solver_time_ms': summary['solver_time_ms'], 'overruns': summary['solve_period_overruns']}), flush=True)
    if not summary['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
