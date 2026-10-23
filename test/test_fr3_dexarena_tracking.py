import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from data_collection.fr3_ik.a3_replay import engagement_windows, replay_bilateral
from data_collection.fr3_ik.constraints import Limits, residual
from data_collection.fr3_ik.dexarena_tracking import differential_step, DEXARENA_VELOCITY_LIMITS
from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.fr3_ik.trajectories import Q0

ROOT = Path(__file__).resolve().parents[1]


def test_tcp_jacobian_against_world_axis_finite_differences():
    kin = Kinematics()
    for q in (np.array(Q0), np.array([.3, -.6, .2, -2., .1, 1.8, -.2])):
        expected = np.empty((6, 7))
        eps = 1e-6
        for j in range(7):
            delta = np.eye(7)[j] * eps
            plus, minus = kin.fk(q + delta), kin.fk(q - delta)
            expected[:3, j] = (plus[:3, 3] - minus[:3, 3]) / (2 * eps)
            expected[3:, j] = Rotation.from_matrix(plus[:3, :3] @ minus[:3, :3].T).as_rotvec() / (2 * eps)
        np.testing.assert_allclose(kin.tcp_jacobian(q), expected, atol=2e-8, rtol=0)


def test_rate_limited_step_keeps_direction_and_catches_up():
    kin, limits = Kinematics(), Limits.from_controller()
    q = np.array(Q0)
    target = kin.fk(q)
    target[:3, 3] += [.04, .03, .03]
    target[:3, :3] = Rotation.from_rotvec([.15, -.1, .1]).as_matrix() @ target[:3, :3]
    dt = .001
    initial_error = sum(residual(kin.fk(q), target))
    first, info = differential_step(kin, q, target, dt, limits.lower, limits.upper)
    assert info['rate_limited']
    assert np.max(np.abs(first - q) / (DEXARENA_VELOCITY_LIMITS * dt)) <= 1 + 1e-10
    for _ in range(500):
        next_q, _ = differential_step(kin, q, target, dt, limits.lower, limits.upper)
        assert np.all(np.abs(next_q - q) <= DEXARENA_VELOCITY_LIMITS * dt + 1e-12)
        q = next_q
    assert sum(residual(kin.fk(q), target)) < initial_error * .01


def test_adaptive_damping_is_explicit_and_preserves_finite_absolute_targets():
    kin, limits = Kinematics(), Limits.from_controller()
    q = np.array(Q0)
    target = kin.fk(q)
    target[:3, 3] += [.005, 0, 0]
    result, info = differential_step(
        kin, q, target, .05, limits.lower, limits.upper,
        damping_mode='adaptive')
    assert info['damping_mode'] == 'adaptive'
    assert np.isfinite(result).all()
    assert np.all(result >= limits.lower) and np.all(result <= limits.upper)
    with np.testing.assert_raises(ValueError):
        differential_step(kin, q, target, .05, limits.lower, limits.upper,
                          damping_mode='unknown')


def test_display_window_ends_on_second_press_even_after_fault():
    events = [dict(action=a, source_monotonic_s=t) for a, t in
              [('engage_both', 12.), ('fault_hold_both', 13.), ('hold_both', 15.)]]
    assert engagement_windows(events, 10.) == [dict(start_s=2., end_s=5.)]


def test_stage11_follows_entire_requested_interval_without_acceleration_latch(tmp_path):
    result = replay_bilateral(
        ROOT/'recordings/quest_raw/stage11-openvr-guided-20261020.jsonl',
        ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json',
        tmp_path/'following', tracking_mode='dexarena_velocity')
    assert result['acceptance'] == 'DIAGNOSTIC_ONLY'
    assert result['control_eligible'] is False
    assert [e['action'] for e in result['control_events']] == ['engage_both', 'hold_both']
    window, = result['robot_display_windows']
    assert abs(window['start_s'] - 8.797274081) < 1e-6
    assert abs(window['end_s'] - 33.765224617) < 1e-6
    for side in ('left', 'right'):
        assert result['arms'][side]['diagnostic_target_count'] == 749
        rows = [json.loads(line) for line in (tmp_path/'following'/side/'samples.jsonl').read_text().splitlines()]
        assert all(not r['control_eligible'] for r in rows)
        assert all(r['ik_state'] in {'HOLD', 'FOLLOWING'} for r in rows)
        assert all(r['checks'][key] for r in rows if r['command_emitted']
                   for key in ('joint_limits', 'step', 'velocity', 'acceleration'))
        first = next(r for r in rows if r['command_emitted'])
        assert first['q_reference_rad'] == rows[0]['q_reference_rad']
        assert first['q_desired_rad'] == rows[0]['q_reference_rad']
        assert all(r['measured_joint_state'] is None for r in rows)
        # Controller keeps moving when robot targets are held before/after engagement.
        for a, b in ((0, 100), (-100, -1)):
            assert rows[a]['q_safe_rad'] == rows[b]['q_safe_rad']
            assert not np.allclose(rows[a]['controller_world'], rows[b]['controller_world'])
        active = [r for r in rows if r['target_valid']]
        np.testing.assert_allclose([r['controller_world'] for r in active],
                                   [r['raw_desired_world'] for r in active], atol=1e-10)


def test_reference_step_jitter_reversal_braking_and_bounds():
    from data_collection.fr3_ik.joint_reference import BoundedJointReference
    limits = Limits.from_controller()
    reference = BoundedJointReference(limits, Q0)
    old_q, old_v = np.array(Q0), np.zeros(7)
    peak_speed = 0
    for i in range(700):
        dt = (.01, .02, .033, .04)[i % 4]
        target = np.array(Q0) + (0.3 if i < 130 else -0.3)
        stop = i >= 260
        q, _ = reference.advance(target, dt, stop=stop)
        v = (q-old_q)/dt
        a = (v-old_v)/dt
        assert np.all(np.abs(v) <= np.asarray(limits.velocity)+1e-10)
        assert np.all(np.abs(a) <= np.asarray(limits.acceleration)+1e-9)
        assert np.max(np.abs(q-old_q)) <= limits.max_step_rad
        assert np.all(q >= limits.lower) and np.all(q <= limits.upper)
        peak_speed = max(peak_speed, np.max(np.abs(v)))
        if i == 300:
            stopped_q = q.copy()
        if i > 300:
            np.testing.assert_array_equal(q, stopped_q)
        old_q, old_v = q, v
    assert peak_speed > .1 and reference.stopped


def test_reference_approaches_joint_envelope_without_clipping_jump():
    from data_collection.fr3_ik.joint_reference import BoundedJointReference
    limits = Limits.from_controller()
    for target in (limits.lower, limits.upper):
        reference = BoundedJointReference(limits, Q0)
        for i in range(2000):
            q, _ = reference.advance(target, (.01, .02, .04, .03)[i % 4])
            assert np.all(q >= limits.lower) and np.all(q <= limits.upper)
        np.testing.assert_allclose(q, target, atol=1e-7)


def test_stationary_takeover_catchup_hold_and_explicit_reanchor():
    from data_collection.fr3_ik.solver import ContinuousIK, DiagnosticIKResult
    from data_collection.fr3_ik.interfaces import ClockDomain, JointTarget, JointVector, Timestamp
    from data_collection.fr3_ik.offline_validator import request_for
    from data_collection.fr3_ik.trajectories import Sample
    kin = Kinematics()
    solver = ContinuousIK(kin, tracking_mode='dexarena_velocity')
    solver.reset(JointTarget(JointVector(Q0), Timestamp(0, ClockDomain.REPLAY, 'static')), 1)
    pose = kin.fk(Q0)
    for i in range(1, 251):
        result = solver.solve(request_for(Sample(i*.02, pose), i, 'static'), solver.last)
        assert isinstance(result, DiagnosticIKResult) and not hasattr(result, 'command_for_adapter')
        np.testing.assert_array_equal(result.command.joints.positions_rad, Q0)
    pose = pose.copy()
    pose[0, 3] += .03
    for i in range(251, 1801):
        result = solver.solve(request_for(Sample(i*.02, pose), i, 'static'), solver.last)
        assert result.success
        assert result.checks.acceleration and result.checks.velocity
        if i == 1500:
            settled = solver.last.joints.positions_rad
        if i > 1500:
            assert solver.last.joints.positions_rad == settled
    assert residual(kin.fk(settled), pose)[0] < 1e-6
    hold = request_for(Sample(36.02, pose, 'hold'), 1801, 'static')
    assert solver.solve(hold, solver.last).state.value == 'HOLD'
    follow = request_for(Sample(36.04, pose), 1802, 'static')
    assert solver.solve(follow, solver.last).state.value == 'REANCHOR_REQUIRED'
    solver.reset(JointTarget(solver.last.joints, Timestamp(36.04, ClockDomain.REPLAY, 'static')), 1)
    first = solver.solve(request_for(Sample(36.06, kin.fk(settled)), 1803, 'static'), solver.last)
    assert first.success and first.command.joints.positions_rad == settled
