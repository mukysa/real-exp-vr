"""Strict offline Quest recording -> safety state machine -> continuous IK replay."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
from bisect import bisect_right
import hashlib
import html
import json
from pathlib import Path
import statistics
import time

import numpy as np
from scipy.spatial.transform import Rotation

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.models import TargetPose
from data_collection.meta_quest.replay import load_recording
from data_collection.meta_quest.state_machine import DebouncedButtonEdge, TeleoperationStateMachine
from .interfaces import ClockDomain, IKRequest, JointTarget, JointVector, ResultState, Timestamp
from .constraints import Limits, check_candidate
from .kinematics import Kinematics
from .mount_calibration import load_tabletop_calibration
from .solver import ContinuousIK
from .preview_mapping import converted_matrix
from .dexarena_tracking import DEXARENA_VELOCITY_LIMITS


def timing_summary(values):
    values = np.asarray(values, dtype=float) * 1000
    if not len(values):
        return dict(samples=0, p50_ms=None, p95_ms=None, p99_ms=None, max_ms=None, over_20_ms=0)
    return dict(samples=len(values), p50_ms=float(np.percentile(values, 50)),
                p95_ms=float(np.percentile(values, 95)), p99_ms=float(np.percentile(values, 99)),
                max_ms=float(max(values)), over_20_ms=int(np.sum(values > 20)))


def matrix_pose(matrix):
    matrix = np.asarray(matrix, dtype=float)
    return RigidPose(tuple(matrix[:3, 3]), tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()))


def pose_matrix(pose):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
    result[:3, 3] = pose.position
    return result


class PoseReferenceShaper:
    """Critically damped second-order end-effector target shaper."""

    def __init__(self, bandwidth):
        if not np.isfinite(bandwidth) or bandwidth <= 0:
            raise ValueError('Pose shaper bandwidth must be positive and finite')
        self.bandwidth = float(bandwidth)
        self.reset(np.zeros(3), Rotation.identity())

    def reset(self, position, rotation):
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = np.zeros(3)
        self.rotation = rotation
        self.angular_velocity = np.zeros(3)

    def update(self, target, dt):
        if dt <= 0 or not np.isfinite(dt):
            raise ValueError('Pose shaper dt must be positive and finite')
        target_position = np.asarray(target[:3, 3], dtype=float)
        target_rotation = Rotation.from_matrix(target[:3, :3])
        decay = np.exp(-self.bandwidth * dt)

        error = self.position - target_position
        term = self.bandwidth * error + self.velocity
        shaped_error = (error + term * dt) * decay
        self.velocity = (self.velocity - self.bandwidth * term * dt) * decay
        self.position = target_position + shaped_error

        rotation_error = (self.rotation * target_rotation.inv()).as_rotvec()
        angular_term = self.bandwidth * rotation_error + self.angular_velocity
        shaped_rotation_error = (rotation_error + angular_term * dt) * decay
        self.angular_velocity = (self.angular_velocity - self.bandwidth * angular_term * dt) * decay
        self.rotation = Rotation.from_rotvec(shaped_rotation_error) * target_rotation

        shaped = np.eye(4)
        shaped[:3, :3] = self.rotation.as_matrix()
        shaped[:3, 3] = self.position
        return shaped


class ArmReplay:
    """One arm's state, checked joint reference and immutable source samples."""

    def __init__(self, recording, snapshot_bytes, source_hash, side, mount_path, position_scale,
                 offline_acceleration_limit=None, pose_shaper_bandwidth=25.0,
                 tracking_mode='strict', motion_mapping='legacy-local'):
        self.side = side
        self.recording = recording
        self.snapshot_bytes = snapshot_bytes
        self.source_hash = source_hash
        self.snapshot = json.loads(snapshot_bytes)
        if self.snapshot.get('schema') != 'franka_readonly_initial_snapshot_v1':
            raise ValueError('Unsupported initial snapshot schema')
        self.arm = self.snapshot['arms'][side]
        if self.arm['joint_names'] != [f'{side}_fr3_joint{i}' for i in range(1, 8)]:
            raise ValueError(f'Expected {side} snapshot joint order')
        self.poses = [p for p in recording.poses if p.sample.source == 'controller' and p.sample.side == side]
        if len(self.poses) < 2:
            raise ValueError(f'Expected non-empty {side} controller input')
        intervals = [b.received_monotonic_s-a.received_monotonic_s for a, b in zip(self.poses, self.poses[1:])]
        if any(dt <= 0 for dt in intervals):
            raise ValueError('Selected arm timestamps must increase strictly')
        self.cadence = statistics.median(intervals)
        self.clock_id = f'replay:{source_hash}'
        self.q0 = np.asarray(self.arm['q_rad'], dtype=float)
        self.kin = Kinematics(side)
        if self.q0.shape != (7,) or not np.isfinite(self.q0).all() or not np.allclose(self.arm['F_T_EE_matrix'], self.kin.tool, atol=1e-6, rtol=0):
            raise ValueError('Snapshot joint vector or hand TCP does not match the model')
        bases, self.mount_provenance, self.mount_bytes = load_tabletop_calibration(mount_path)
        self.world_from_base = bases[side]
        self.base_from_world = np.linalg.inv(self.world_from_base)
        if motion_mapping not in {'legacy-local', 'tabletop-relative'}:
            raise ValueError('Expected legacy-local or tabletop-relative motion mapping')
        self.motion_mapping = motion_mapping
        self.machine = TeleoperationStateMachine(
            matrix_pose(self.world_from_base @ self.kin.fk(self.q0)), side=side,
            frame_id='tabletop_world', position_scale=position_scale,
            source_to_target_frame=matrix_pose(self.world_from_base if motion_mapping == 'legacy-local' else np.eye(4)),
            motion_mapping='local-relative' if motion_mapping == 'legacy-local' else 'spatial-relative')
        self.position_scale = position_scale
        self.pose_shaper = PoseReferenceShaper(pose_shaper_bandwidth)
        self.pose_shaper_bandwidth = float(pose_shaper_bandwidth)
        initial_stamp = self.stamp(max(0.0, self.poses[0].received_monotonic_s-self.cadence))
        self.safe_target = JointTarget(JointVector(tuple(self.q0)), initial_stamp)
        controller_limits = Limits.from_controller()
        effective_limits = controller_limits
        if offline_acceleration_limit is not None:
            if not np.isfinite(offline_acceleration_limit) or offline_acceleration_limit <= 0:
                raise ValueError('Offline acceleration limit must be positive and finite')
            if offline_acceleration_limit < max(controller_limits.acceleration):
                raise ValueError('Offline acceleration override may only raise the controller limit')
            effective_limits = replace(controller_limits,
                acceleration=(float(offline_acceleration_limit),) * 7)
        self.controller_acceleration_limit = tuple(controller_limits.acceleration)
        self.offline_acceleration_limit = offline_acceleration_limit
        if tracking_mode != 'strict' and offline_acceleration_limit is not None:
            raise ValueError('Acceleration overrides apply only to strict diagnostics')
        self.solver = ContinuousIK(self.kin, limits=effective_limits, timeout_s=None,
                                   tracking_mode=tracking_mode)
        self.solver.reset(self.safe_target, self.poses[0].connection_id)
        self.previous_ik_state = ResultState.HOLD
        self.rows = []
        self.latest = None

    def stamp(self, seconds):
        return Timestamp(seconds, ClockDomain.REPLAY, self.clock_id)

    def suspend(self, reason, *, fault):
        safe_pose = matrix_pose(self.world_from_base @ self.kin.fk(self.safe_target.joints.positions_rad))
        self.machine.suspend(safe_pose, reason, fault=fault)

    def step(self, received, *, engage_edge=None):
        step_started = time.perf_counter()
        side = self.side
        previous_q = np.asarray(self.safe_target.joints.positions_rad)
        previous_velocity = self.solver.velocity.copy()
        braking_blocks_engage = (self.solver.tracking_mode == 'dexarena_velocity'
            and self.previous_ik_state != ResultState.FOLLOWING and not self.solver.reference.stopped)
        if braking_blocks_engage:
            engage_edge = False
        target_world = self.machine.update(received, engage_edge=engage_edge)
        stamp = self.stamp(received.received_monotonic_s)
        raw_world = pose_matrix(target_world.pose)
        shaped_world = raw_world
        entering_follow = (
            target_world.valid and target_world.state == 'FOLLOWING'
            and self.previous_ik_state != ResultState.FOLLOWING
        )
        safe_world = self.world_from_base @ self.kin.fk(self.safe_target.joints.positions_rad)
        if entering_follow:
            self.pose_shaper.reset(safe_world[:3, 3], Rotation.from_matrix(safe_world[:3, :3]))
            shaped_world = safe_world.copy()
        elif target_world.valid and target_world.state == 'FOLLOWING':
            shaped_world = self.pose_shaper.update(
                raw_world, received.received_monotonic_s-self.latest.received_monotonic_s)
        else:
            self.pose_shaper.reset(safe_world[:3, 3], Rotation.from_matrix(safe_world[:3, :3]))
            shaped_world = safe_world.copy()
        target_world = replace(target_world, pose=matrix_pose(shaped_world))
        target = replace(target_world,
            pose=matrix_pose(self.base_from_world @ pose_matrix(target_world.pose)),
            frame_id=f'{side}_fr3_link0', generated_monotonic_s=stamp.seconds)
        request = IKRequest(target, stamp, stamp, tool_frame=f'{side}_fr3_hand_tcp', side=side)
        if target.valid and target.state == 'FOLLOWING' and self.previous_ik_state != ResultState.FOLLOWING:
            self.safe_target = JointTarget(self.safe_target.joints, self.stamp(max(0.0, stamp.seconds-self.cadence)))
            self.solver.reset(self.safe_target, received.connection_id)
        result = self.solver.solve(request, self.safe_target, measured_joint_state=None)
        self.last_result = result
        if result.success:
            self.safe_target = result.command
        elif self.solver.tracking_mode == 'dexarena_velocity':
            # Stopping the accepted desired stream is distinct from an abrupt
            # freeze of a moving reference. No new desired IK command is emitted.
            dt = stamp.seconds-self.latest.received_monotonic_s if self.latest else self.cadence
            was_stopped = self.solver.reference.stopped
            self.safe_target = self.solver.advance_stop(stamp, dt)
            if self.solver.reference.stopped and not was_stopped:
                # Anchor the next takeover at the end of braking, rather than
                # the earlier stop-request pose. Preserve fault recovery gates.
                self.suspend(target.hold_reason or result.detail, fault=result.state != ResultState.HOLD)
        self.previous_ik_state = result.state
        q = np.asarray(self.safe_target.joints.positions_rad)
        points, achieved = self.kin.geometry(q)
        actual_world = self.world_from_base @ achieved
        desired = pose_matrix(target.pose)
        frame_dt = received.received_monotonic_s-self.latest.received_monotonic_s if self.latest else self.cadence
        reference_checks = check_candidate(q, previous_q, previous_velocity, frame_dt, 0, 0,
                                           self.solver.limits)[0]
        position_error = float(np.linalg.norm(achieved[:3, 3]-desired[:3, 3])) if result.success else None
        rotation_error = float(Rotation.from_matrix(desired[:3, :3].T@achieved[:3, :3]).magnitude()) if result.success else None
        row = dict(index=len(self.rows), side=side, source_sequence=received.sequence,
            connection_id=received.connection_id, source_monotonic_s=received.received_monotonic_s,
            dt_s=frame_dt,
            clutch=received.sample.clutch, grasp=received.sample.grasp, tracked=received.sample.tracked,
            external_engage_edge=engage_edge,
            target_state=target.state, target_valid=target.valid, hold_reason=target.hold_reason,
            ik_state=result.state.value, ik_reason=result.reason.value, command_emitted=result.success,
            q_safe_rad=list(q), new_command_rad=list(result.command.joints.positions_rad) if result.success else None,
            fk_position_error_m=position_error, fk_rotation_error_rad=rotation_error,
            checks=asdict(result.checks), solve_time_s=result.solve_time_s,
            control_eligible=False,
            q_desired_rad=self.solver.diagnostics.get('desired_joint_rad'),
            q_reference_rad=list(q), measured_joint_state=None,
            reference_phase=self.solver.diagnostics.get('reference_phase'),
            takeover_blocked_braking=braking_blocks_engage,
            reference_checks=asdict(reference_checks),
            reference_velocity_rad_s=self.solver.velocity.tolist(),
            desired_to_reference_max_rad=(float(np.max(np.abs(np.asarray(self.solver.diagnostics['desired_joint_rad'])-q)))
                if self.solver.diagnostics.get('desired_joint_rad') is not None else None),
            tracking_diagnostics=dict(self.solver.diagnostics),
            desired_base=desired.tolist(), actual_base=achieved.tolist(),
            raw_desired_world=raw_world.tolist(), desired_world=shaped_world.tolist(), actual_world=actual_world.tolist(),
            points_world=[(self.world_from_base @ np.r_[point, 1])[:3].tolist() for point in points])
        row['pipeline_time_s'] = time.perf_counter()-step_started
        self.rows.append(row)
        self.latest = received
        return row

    def save(self, recording_path, output_path, *, engage_mode='independent', engage_side=None):
        rows = self.rows
        # Display the actual controller through one fixed mapping, including
        # before engagement and after disengagement. Held robot targets cannot
        # stand in for a live controller. Anchor the display at first engagement.
        anchor_index = next((i for i, row in enumerate(rows) if row['target_valid']), 0)
        def source_matrix(received):
            source = converted_matrix(received.sample)
            source[:3, 3] *= self.position_scale
            return self.world_from_base @ source if self.motion_mapping == 'legacy-local' else source
        source_anchor = source_matrix(self.poses[anchor_index])
        target_anchor = np.asarray(rows[anchor_index]['raw_desired_world'])
        controller_map = target_anchor @ np.linalg.inv(source_anchor)
        for row, received in zip(rows, self.poses):
            source = source_matrix(received)
            if self.motion_mapping == 'tabletop-relative':
                # Display the controller's own orientation in the common
                # tracking axes. Only its origin is shifted to the robot TCP;
                # this is a relative display, not a physical registration.
                source[:3, 3] += target_anchor[:3, 3] - source_anchor[:3, 3]
                row['controller_world'] = source.tolist()
            else:
                row['controller_world'] = (controller_map @ source).tolist()
        diagnostic = self.solver.tracking_mode != 'strict'
        states = Counter(row['ik_state'] for row in rows)
        reasons = Counter(row['ik_reason'] for row in rows if row['ik_reason'] != 'none')
        trajectory = [{key: row[key] for key in ('source_sequence','connection_id','target_state','target_valid','ik_state','ik_reason','command_emitted','q_safe_rad','new_command_rad','checks')} for row in rows]
        trajectory_hash = hashlib.sha256(json.dumps(trajectory, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        summary = dict(schema='fr3_dexarena_bounded_reference_diagnostic_v2' if diagnostic else 'fr3_a3_strict_replay_v1', source=str(recording_path), source_sha256=self.source_hash,
            source_unchanged=True, side=self.side, source_pose_count=len(self.recording.poses), selected_pose_count=len(self.poses),
            ignored_pose_count=len(self.recording.poses)-len(self.poses), connection_ids=sorted({p.connection_id for p in self.poses}),
            first_sequence=self.poses[0].sequence, last_sequence=self.poses[-1].sequence,
            parameters=dict(position_scale=self.position_scale, engage_button='clutch', button_debounce_s=0.15,
                motion_mapping=self.motion_mapping,
                engage_mode=engage_mode, engage_side=engage_side or self.side,
                pose_shaper='critically_damped_second_order',
                pose_shaper_bandwidth_rad_s=self.pose_shaper_bandwidth,
                ik_tracking_mode=self.solver.tracking_mode,
                diagnostic_velocity_cap_rad_s=DEXARENA_VELOCITY_LIMITS.tolist() if diagnostic else None,
                diagnostic_algorithm='DLS .08, no joint centering, exact takeover anchor; bounded sampled 6/s joint servo' if diagnostic else None,
                reference_policy='last bounded reference feeds IK; sampled velocity/acceleration constraints; bounded braking on HOLD/fault' if diagnostic else None,
                controller_acceleration_limit_rad_s2=list(self.controller_acceleration_limit),
                offline_acceleration_limit_rad_s2=self.offline_acceleration_limit,
                min_ready_samples=3, stale_timeout_s=self.machine.limits.stale_timeout_s,
                source_to_target_frame=('historical tabletop installation transform' if self.motion_mapping == 'legacy-local'
                                        else 'shared tabletop axes; tracking yaw assumed zero, not physically calibrated'),
                ik_max_function_evaluations=self.solver.max_evaluations, ik_timeout_s=self.solver.timeout_s,
                ik_constraints=asdict(self.solver.limits), state_machine_limits=asdict(self.machine.limits)),
            clutch_true_frames=sum(bool(p.sample.clutch) for p in self.poses), initial_snapshot_sha256=hashlib.sha256(self.snapshot_bytes).hexdigest(),
            initial_joints_rad=self.q0.tolist(), initial_pose_source=self.arm['host_capture_completed_at'],
            display_frame='tabletop_world', ik_frame=f'{self.side}_fr3_link0', mount_calibration=self.mount_provenance,
            synchronization='Original host arrival timestamps; no source pose interpolation',
            state_counts=dict(states), reason_counts=dict(reasons), command_count=sum(row['command_emitted'] for row in rows),
            diagnostic_target_count=sum(row['command_emitted'] for row in rows) if diagnostic else 0,
            strict_check_failures={key: sum(row['checks'][key] is False for row in rows if row['command_emitted'])
                                   for key in ('residual', 'joint_limits', 'step', 'velocity', 'acceleration')},
            rate_limited_frames=sum(row['tracking_diagnostics'].get('rate_limited', False) for row in rows),
            reference_rate_limited_frames=sum(row['tracking_diagnostics'].get('reference_rate_limited', False) for row in rows),
            reference_constraint_failures={key: sum(row['reference_checks'][key] is False for row in rows)
                for key in ('joint_limits', 'step', 'velocity', 'acceleration')},
            measured_state_available=False,
            solve_timing=timing_summary([row['solve_time_s'] for row in rows if row['command_emitted']]),
            pipeline_timing=timing_summary([row['pipeline_time_s'] for row in rows]),
            all_commands_checked=all(row['checks'] and all(value is True for value in row['checks'].values()) for row in rows if row['command_emitted']),
            max_fk_position_error_m=max((row['fk_position_error_m'] or 0 for row in rows), default=0),
            max_fk_rotation_error_rad=max((row['fk_rotation_error_rad'] or 0 for row in rows), default=0),
            trajectory_sha256=trajectory_hash, deterministic_fields='State, checks and joint target sequence; excludes solve_time_s',
            safety_scope='Offline only; no robot connection, command publication, collision checking, or real-time guarantee',
            acceptance='DIAGNOSTIC_ONLY' if diagnostic else 'PASS' if all(row['target_valid'] == (row['ik_state']=='FOLLOWING') and row['command_emitted']==(row['ik_state']=='FOLLOWING') for row in rows) else 'FAIL')
        output_path.mkdir(parents=True)
        (output_path/'initial_snapshot.json').write_bytes(self.snapshot_bytes)
        (output_path/'mount_calibration.md').write_bytes(self.mount_bytes)
        (output_path/'samples.jsonl').write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in rows))
        (output_path/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        (output_path/'report.html').write_text(render_report(summary, rows))
        return summary


def load_inputs(recording_path, snapshot_path, output_path, mount_path):
    recording_path, snapshot_path, output_path = map(Path, (recording_path, snapshot_path, output_path))
    if output_path.exists():
        raise FileExistsError(f'Use a new output directory: {output_path}')
    mount_path = Path(mount_path) if mount_path else Path(__file__).resolve().parents[2]/'calibration/matrix.md'
    source_hash = hashlib.sha256(recording_path.read_bytes()).hexdigest()
    return recording_path, output_path, mount_path, source_hash, load_recording(recording_path), snapshot_path.read_bytes()


def check_source(recording_path, source_hash):
    if hashlib.sha256(recording_path.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError('Source recording changed during replay')


def replay(recording_path, snapshot_path, output_path, side='right', mount_path=None, position_scale=1.0,
           offline_acceleration_limit=None, pose_shaper_bandwidth=25.0, tracking_mode='strict', motion_mapping='legacy-local'):
    if side not in {'left', 'right'}:
        raise ValueError('side must be left or right')
    recording_path, output_path, mount_path, source_hash, recording, snapshot_bytes = load_inputs(recording_path, snapshot_path, output_path, mount_path)
    arm = ArmReplay(recording, snapshot_bytes, source_hash, side, mount_path, position_scale,
                    offline_acceleration_limit, pose_shaper_bandwidth, tracking_mode, motion_mapping)
    for received in arm.poses:
        arm.step(received)
    check_source(recording_path, source_hash)
    return arm.save(recording_path, output_path)


class SharedEngagement:
    """One debounced toggle for two arms. Faults stop both until stop/re-engage."""

    def __init__(self, arms, engage_side):
        self.arms = arms
        self.engage_side = engage_side
        self.button = DebouncedButtonEdge(0.15)
        self.initialized = False
        self.requested = False
        self.fault = None
        self.pending = {side: False for side in arms}
        self.events = []

    def record(self, received, action, reason=None):
        self.events.append(dict(source_monotonic_s=received.received_monotonic_s,
            source_sequence=received.sequence, action=action, reason=reason))

    def stop(self, received, reason):
        if self.fault is not None:
            return
        self.fault = reason
        self.pending = dict.fromkeys(self.arms, False)
        for arm in self.arms.values():
            arm.suspend(reason, fault=True)
        self.record(received, 'fault_hold_both', reason)

    def before(self, received):
        now = received.received_monotonic_s
        if self.requested and not self.fault:
            for side, arm in self.arms.items():
                if arm.latest is None or now-arm.latest.received_monotonic_s > arm.machine.limits.stale_timeout_s:
                    self.stop(received, f'{side}:stale_peer_input')
                    break
        if received.sample.side != self.engage_side:
            return
        pressed = bool(received.sample.clutch)
        if not self.initialized:
            self.button.reset(pressed)
            self.initialized = True
        if not self.button.update(pressed, now):
            return
        if self.requested:
            self.requested = False
            self.pending = dict.fromkeys(self.arms, False)
            if not self.fault:
                for arm in self.arms.values():
                    arm.suspend('shared_manual_hold', fault=False)
            self.record(received, 'hold_both')
            return
        # Gate the shared request using only previously observed, fresh samples.
        ready = all(arm.latest is not None and arm.latest.sample.tracked
            and now-arm.latest.received_monotonic_s <= arm.machine.limits.stale_timeout_s
            and (arm.solver.tracking_mode == 'strict' or arm.solver.reference.stopped)
            and arm.machine.ready_for_external_engage for arm in self.arms.values())
        if not ready or not received.sample.tracked:
            self.record(received, 'engage_rejected', 'both_arms_must_be_ready_fresh_and_reference_stopped')
            return
        self.requested = True
        self.fault = None
        self.pending = dict.fromkeys(self.arms, True)
        self.record(received, 'engage_both')

    def step(self, received):
        self.before(received)
        side = received.sample.side
        edge = self.pending[side] and self.requested and not self.fault
        row = self.arms[side].step(received, engage_edge=bool(edge))
        self.pending[side] = False
        if row['ik_state'] in {'FAULT', 'REANCHOR_REQUIRED'}:
            self.stop(received, f'{side}:{row["hold_reason"] or row["ik_reason"]}')
        return dict(requested_following=self.requested, fault_reason=self.fault,
                    state='FAULT_HOLD' if self.fault else 'FOLLOWING' if self.requested else 'HOLD')


def replay_bilateral(recording_path, snapshot_path, output_path, mount_path=None, position_scale=1.0,
                     engage_mode='shared', engage_side='right', offline_acceleration_limit=None,
                     pose_shaper_bandwidth=25.0, tracking_mode='strict', motion_mapping='legacy-local'):
    if engage_mode not in {'shared', 'independent'} or engage_side not in {'left', 'right'}:
        raise ValueError('Expected shared/independent mode and left/right engage side')
    recording_path, output_path, mount_path, source_hash, recording, snapshot_bytes = load_inputs(recording_path, snapshot_path, output_path, mount_path)
    arms = {side: ArmReplay(recording, snapshot_bytes, source_hash, side, mount_path, position_scale,
                            offline_acceleration_limit, pose_shaper_bandwidth, tracking_mode, motion_mapping) for side in ('left', 'right')}
    group = SharedEngagement(arms, engage_side) if engage_mode == 'shared' else None
    controls = {}
    for received in recording.poses:
        if received.sample.source != 'controller' or received.sample.side not in arms:
            continue
        if group:
            controls[received.received_monotonic_s] = group.step(received)
        else:
            arms[received.sample.side].step(received)
    check_source(recording_path, source_hash)
    summaries = {side: arm.save(recording_path, output_path/side, engage_mode=engage_mode,
                              engage_side=engage_side if group else side) for side, arm in arms.items()}
    samples = {side: arm.rows for side, arm in arms.items()}
    times = {side: [row['source_monotonic_s'] for row in samples[side]] for side in samples}
    start = min(values[0] for values in times.values())
    timeline = []
    for stamp in sorted(set(times['left']) | set(times['right'])):
        frame = dict(time=stamp-start, source_monotonic_s=stamp, arms={})
        if group:
            frame['control'] = controls[stamp]
        for side in ('left', 'right'):
            index = bisect_right(times[side], stamp)-1
            frame['arms'][side] = dict(index=max(index, 0), awaiting_first_source=index < 0,
                sample_age_s=None if index < 0 else stamp-times[side][index])
        timeline.append(frame)
    summary = dict(schema='fr3_bilateral_dexarena_bounded_reference_diagnostic_v2' if tracking_mode != 'strict' else 'fr3_a3_bilateral_strict_replay_v2', source=str(recording_path),
        source_sha256=source_hash, source_unchanged=True,
        initial_snapshot_sha256=hashlib.sha256(snapshot_bytes).hexdigest(),
        selected_pose_count={side: summaries[side]['selected_pose_count'] for side in summaries},
        ignored_pose_count={side: summaries[side]['ignored_pose_count'] for side in summaries},
        clutch_true_frames={side: summaries[side]['clutch_true_frames'] for side in summaries},
        engage_mode=engage_mode, engage_side=engage_side if group else 'per_arm',
        fault_policy='hold_both' if group else 'per_arm',
        control_events=group.events if group else [],
        arms=summaries, timeline_events=len(timeline), duration_s=timeline[-1]['time'],
        synchronization='Original host arrival timestamp union; shared edges apply on each arm next fresh sample. No interpolation or hardware synchronization.',
        tracking_mode=tracking_mode,
        motion_mapping=motion_mapping,
        acceptance=('DIAGNOSTIC_ONLY' if tracking_mode == 'dexarena_velocity' else 'FAIL' if not all(x['acceptance']=='PASS' for x in summaries.values())
                    or any(e['action'] in {'fault_hold_both', 'engage_rejected'} for e in (group.events if group else []))
                    else 'PASS_WITH_OFFLINE_OVERRIDE' if offline_acceleration_limit is not None else 'PASS'),
        offline_acceleration_override_only=True if offline_acceleration_limit is not None else False,
        display_frame='tabletop_world', display_events=len(timeline),
        scope='Strict offline state machines and IK; shared engagement with coupled fault hold or independent mode; no robot publication or collision checking')
    if group:
        summary['robot_display_windows'] = engagement_windows(group.events, start)
    summary['controller_display'] = 'Live source poses with fixed display mapping anchored at first engagement; full recording, including outside robot display windows'
    if tracking_mode != 'strict':
        summary['scope'] = 'Offline DexArena task-step with centering disabled, bounded sampled joint reference and braking; residual lag is diagnostic. No measured plant, PD simulation or hardware commands; not continuous controller validation'
        summary['control_eligible'] = False
    summary['bilateral_paired_compute_timing'] = timing_summary([
        a['pipeline_time_s'] + b['pipeline_time_s']
        for a, b in zip(samples['left'], samples['right'])])
    summary['timing_scope'] = 'Sequential per-arm step CPU wall time summed by source ordinal pair; includes mapping/FK/checks, excludes save/render/ROS/controller; not online deadline validation'
    (output_path/'timeline.json').write_text(json.dumps(timeline, allow_nan=False)+'\n')
    (output_path/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    (output_path/'report.html').write_text(render_bilateral_report(summary, timeline, samples))
    (output_path/'playback.html').write_text(render_strict_playback(summary, timeline, samples))
    return summary


def engagement_windows(events, start):
    """Operator-requested windows; a fault keeps the stopped robot visible."""
    windows = []
    for event in events:
        if event['action'] == 'engage_both':
            windows.append(dict(start_s=event['source_monotonic_s'] - start, end_s=None))
        elif event['action'] == 'hold_both' and windows and windows[-1]['end_s'] is None:
            windows[-1]['end_s'] = event['source_monotonic_s'] - start
    return windows


def render_report(summary, rows):
    diagnostic = summary['parameters']['ik_tracking_mode'] == 'dexarena_velocity'
    title = 'DexArena 微分 IK 追赶诊断' if diagnostic else 'A3 严格离线 IK 回放'
    target_label = '离线参考' if diagnostic else '已接受目标'
    explanation = ('速度超限时统一缩小关节步进，继续追赶目标；残差、步长、速度、加速度的严格 A3 检查仍记录，但不作为本诊断的锁停条件。跟踪差包含主动限速带来的滞后。没有模拟 PD 动力学或实测状态反馈，不能作为 B 线控制命令。'
                   if diagnostic else 'IK 输出逐帧执行约束并使用独立完整模型 FK 复核。')
    counts = html.escape(json.dumps({key:summary[key] for key in ('state_counts','reason_counts','command_count','all_commands_checked','trajectory_sha256','acceptance')}, indent=2))
    body = ''.join('<tr>'+''.join(f'<td>{html.escape(str(value))}</td>' for value in (
        row['index'], row['source_sequence'], f"{row['source_monotonic_s']:.3f}", row['clutch'], row['target_state'],
        row['ik_state'], row['ik_reason'], row['command_emitted'],
        '' if row['fk_position_error_m'] is None else f"{row['fk_position_error_m']*1000:.4f}",
    ))+'</tr>' for row in rows)
    return f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title>
<style>body{{font:14px system-ui;background:#101820;color:#e5edf5;margin:24px;max-width:1500px}}h1{{font-size:24px}}p{{line-height:1.6;color:#b7c7d8}}pre,table{{background:#182635;border:1px solid #30475c;border-radius:5px}}pre{{padding:14px;overflow:auto}}table{{border-collapse:collapse;width:100%;font-size:12px}}th,td{{padding:6px 8px;border-bottom:1px solid #30475c;text-align:left}}th{{position:sticky;top:0;background:#24384b}}.scroll{{max-height:65vh;overflow:auto}}.pass{{color:#73e0b0}}</style>
<h1>{title} · {html.escape(summary['source'].split('/')[-1])}</h1><p class="pass">结果：{summary['acceptance']} · {target_label} {summary['command_count']} 条 · 原始录制 SHA-256 {summary['source_sha256']}</p><p>接管模式 {html.escape(summary['parameters']['engage_mode'])}，按键来源 {html.escape(summary['parameters']['engage_side'])}；表格 clutch 保留该手原始值，external_engage_edge 见 JSONL。无效目标不输出新关节目标。{explanation} 该结果不构成机器人控制授权或安全保证。</p><pre>{counts}</pre><div class="scroll"><table><thead><tr><th>帧</th><th>序号</th><th>时间 s</th><th>clutch</th><th>目标状态</th><th>IK 状态</th><th>原因</th><th>{target_label}</th><th>FK 跟踪差 mm</th></tr></thead><tbody>{body}</tbody></table></div></html>'''


def render_bilateral_report(summary, timeline, samples):
    rows = []
    for event in timeline:
        cells = [f"{event['time']:.3f}"]
        for side in ('left', 'right'):
            arm_event = event['arms'][side]
            if arm_event['awaiting_first_source']:
                cells.extend(('--', 'WAIT_INPUT', 'HOLD', '等待首帧', '否', ''))
                continue
            sample = samples[side][arm_event['index']]
            cells.extend((str(sample['source_sequence']), sample['target_state'], sample['ik_state'],
                          sample['ik_reason'], '是' if sample['command_emitted'] else '否',
                          '' if arm_event['sample_age_s'] is None else f"{arm_event['sample_age_s']*1000:.1f}"))
        rows.append('<tr>'+''.join(f'<td>{html.escape(value)}</td>' for value in cells)+'</tr>')
    labels = {'left': '左臂', 'right': '右臂'}
    start = timeline[0]['source_monotonic_s']
    event_rows = ''.join('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in (
        f"{e['source_monotonic_s']-start:.3f}", e['source_sequence'],
        {'engage_both':'双臂接管', 'hold_both':'双臂暂停请求', 'fault_hold_both':'双臂故障保持', 'engage_rejected':'拒绝接管'}.get(e['action'],e['action']),
        e['reason'] or '—'))+'</tr>' for e in summary['control_events'])
    mode = '共享接管' if summary['engage_mode']=='shared' else '各臂独立接管'
    policy = '一臂故障，两臂保持；先按键关闭接管请求，再次按键重新锚定' if summary['fault_policy']=='hold_both' else '各臂独立故障保持'
    diagnostic = summary.get('tracking_mode') == 'dexarena_velocity'
    title = 'DexArena 双臂微分 IK 追赶诊断' if diagnostic else 'A3 双臂严格离线 IK 回放'
    if diagnostic:
        policy = '接管首帧关节参考不变，禁用关节居中；独立受限参考按原控制器速度/加速度上限追赶，残差保留为滞后。停止或输入故障后参考减速，停稳后才允许重新接管。未模拟动力学或反馈，此输出不能作为 B 线控制命令'

    state_summary = ''.join(f'<section><h2>{labels[side]}</h2><pre>{html.escape(json.dumps({key:summary["arms"][side][key] for key in ("state_counts","reason_counts","command_count","all_commands_checked","trajectory_sha256","acceptance")}, indent=2))}</pre><p><a href="{side}/report.html">打开{labels[side]}逐帧诊断</a></p></section>' for side in ('left','right'))
    return f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title>
<style>body{{font:14px system-ui;background:#101820;color:#e5edf5;margin:24px;max-width:1600px}}h1{{font-size:24px}}p{{line-height:1.6;color:#b7c7d8}}a{{color:#69d3dc}}pre,table,section{{background:#182635;border:1px solid #30475c;border-radius:5px}}pre{{padding:14px;overflow:auto}}.arms{{display:grid;grid-template-columns:1fr 1fr;gap:12px}}section{{padding:12px}}.scroll{{max-height:62vh;overflow:auto}}table{{border-collapse:collapse;width:100%;font-size:12px}}th,td{{padding:6px 8px;border-bottom:1px solid #30475c;text-align:left}}th{{position:sticky;top:0;background:#24384b}}.pass{{color:#73e0b0}}@media(max-width:900px){{.arms{{grid-template-columns:1fr}}}}</style>
<h1>{title} · {html.escape(Path(summary['source']).name)}</h1><p class="pass">结果：{summary['acceptance']} · SHA-256 {summary['source_sha256']}</p><p>接管模式：{mode}；按键来源：{summary['engage_side']}。{policy}。左右臂分别求解 IK 和检查约束。共同时间轴按原始主机到达时间取并集，每臂保持最新样本，不插值；不保证硬件同步。当前为离线结果，没有连接机器人，也没有做双臂碰撞检查。</p><p><a href="playback.html">打开本次双臂可播放空间回放与关节曲线</a></p><iframe title="双臂回放" src="playback.html" style="width:100%;height:850px;border:1px solid #30475c"></iframe><h2>接管与故障事件</h2><table><tr><th>录制时间 s</th><th>源序号</th><th>事件</th><th>原因</th></tr>{event_rows}</table><div class="arms">{state_summary}</div><h2>共同时间轴状态</h2><p>每臂样本年龄单位为毫秒。</p><div class="scroll"><table><thead><tr><th>时间 s</th><th colspan="6">左臂</th><th colspan="6">右臂</th></tr><tr><th></th><th>序号</th><th>目标</th><th>IK</th><th>原因</th><th>离线目标</th><th>年龄 ms</th><th>序号</th><th>目标</th><th>IK</th><th>原因</th><th>离线目标</th><th>年龄 ms</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div></html>'''


def render_strict_playback(summary, timeline, samples):
    """Render only accepted joint references; never show rejected candidate joints."""
    viewer_samples = {}
    for side, rows in samples.items():
        converted = []
        velocity = np.zeros(7)
        previous_q = None
        for row in rows:
            q = np.asarray(row['q_safe_rad'])
            next_velocity = (q-previous_q)/row['dt_s'] if previous_q is not None else np.zeros(7)
            acceleration = (next_velocity-velocity)/row['dt_s']
            position_error = np.linalg.norm(np.asarray(row['actual_world'])[:3, 3]-np.asarray(row['desired_world'])[:3, 3])
            rotation_error = Rotation.from_matrix(np.asarray(row['desired_world'])[:3,:3].T@np.asarray(row['actual_world'])[:3,:3]).magnitude()
            converted.append(dict(row, q=row['q_safe_rad'], velocity=next_velocity.tolist(), acceleration=acceleration.tolist(),
                position_error_m=float(position_error), rotation_error_rad=float(rotation_error),
                success=row['command_emitted'], all_control_checks_passed=row['command_emitted'] and all(v is True for v in row['checks'].values())))
            previous_q, velocity = q, next_velocity
        viewer_samples[side] = converted
    data = json.dumps(dict(strict=True, summary=summary, samples=viewer_samples, timeline=timeline), allow_nan=False).replace('<', '\\u003c')
    directory = Path(__file__).parent/'visualization'
    template = (directory/'dual_viewer.html').read_text()
    title = f'{Path(summary["source"]).stem} · A3 双臂严格回放'
    template = template.replace('test · V5 · 双臂桌面回放', html.escape(title))
    template = template.replace('左右手各自驱动对应机械臂的离线几何模型', '左右手各自驱动对应机械臂的严格离线模型')
    template = template.replace('本预览忽略 clutch，展示完整动作；速度与加速度检查失败的几何解也会显示。尚未验证双臂碰撞、桌面碰撞或安全接管，不可用于控制。扳机值仅供观察，夹爪开合没有建模。',
        '本回放执行接管与运动约束，机械臂只显示已接受关节目标的 FK；拒绝的候选关节不显示。双臂默认共享右手接管键，一臂故障时两臂保持。FK 是离线目标模型，尚未验证碰撞与真机响应。扳机仅供观察，夹爪开合没有建模。')
    template = template.replace('位移和旋转 1 倍', f'平移比例 {summary["arms"]["right"]["parameters"]["position_scale"]}，旋转 1 倍')
    template = template.replace('双臂默认共享右手接管键', f'接管模式 {summary["engage_mode"]}，按键来源 {summary["engage_side"]}')
    template = template.replace('双手镜像相对位移 / m', '双臂目标镜像相对位移差 / m')
    template = template.replace('下一检查异常', '下一故障')
    template = template.replace('<option value="symmetry">', '<option value="symmetry">')
    template = template.replace('<option value="q">', '<option value="q" selected>')
    template = template.replace('<h2>轨迹与对称性检查</h2>', '<h2>严格回放摘要</h2>')
    template = template.replace('虚线为目标，实线为 FK', '虚线为状态机目标，实线为已接受目标的 FK')
    template = template.replace('几何解或实际安装位置', '安全关节目标或实际安装位置')
    template = template.replace(' ／ <a href="../test-20261020-world-frame-v4-measured-start-20261021/report.html">V4 右臂对照</a>', ' ／ <a href="report.html">接管事件与严格验收报告</a>')
    template = template.replace('<input id="mirror"',
        '<input id="show_hold" type="checkbox">显示停止与未接管机械臂</label><label><input id="mirror"')
    if summary.get('tracking_mode') == 'dexarena_velocity':
        template = template.replace('<option value="q" selected>',
            '<option value="q_desired_rad">期望关节任务步 / rad</option><option value="joint_lag">期望与受限参考之差 / rad</option><option value="q" selected>')
        template = template.replace('A3 双臂严格回放', 'DexArena 风格双臂追赶诊断')
        template = template.replace('严格离线模型', '离线微分 IK 模型')
        template = template.replace('本回放执行接管与运动约束，机械臂只显示已接受关节目标的 FK；拒绝的候选关节不显示。',
                                    '本回放采用 DexArena 任务步进，禁用关节居中；接管首帧保持原关节，随后生成速度/加速度受限参考。跟踪滞后保留，不将参考 FK 当作测量反馈；停止时参考减速，停稳后允许重新接管。')
        template = template.replace('通过 IK 的 FK', '微分 IK 参考 FK').replace('已接受关节目标', '离线关节参考')
        template = template.replace('严格回放摘要', '追赶诊断摘要').replace('下一故障', '下一约束超限')
        template = template.replace('尚未验证碰撞与真机响应。', '未模拟 PD 动力学与实测状态反馈，尚未验证碰撞与真机响应。')
        template = template.replace('位置残差 / mm', '目标跟踪差 / mm').replace('姿态残差 / deg', '姿态跟踪差 / deg')
    template = template.replace('未插值、平滑或人工同步。', '未插值或人工同步；目标已整形，源手柄位姿未平滑。')
    template = template.replace('同一 PC 接收时间轴；', '全程显示手柄；机械臂只在接管点击区间出现。手柄采用首次接管的固定显示映射，不代表物理标定位置。同一 PC 接收时间轴；')
    mapping = summary['arms']['right']['parameters'].get('motion_mapping', 'legacy-local')
    mapping_note = ('共同桌面轴映射：两手 +X 前 / +Y 左 / +Z 上，平移不受握柄初始姿态影响；'
                    '旋转绕共同空间轴。Quest 跟踪轴与桌面轴的偏航暂按 0 处理，尚未物理标定。'
                    if mapping == 'tabletop-relative' else
                    '旧局部锚定映射：接管时的手柄与 TCP 朝向会旋转位移方向，两臂可能不同；仅保留作问题对照。')
    template = template.replace('<div class="bar"><button id="prev">',
                                f'<p><strong>{mapping_note}</strong></p><div class="bar"><button id="prev">')
    return template.replace('/*REPORT_DATA*/', data).replace('/*VIEWER_JS*/', (directory/'dual_viewer.js').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--initial-snapshot', required=True, type=Path)
    parser.add_argument('--side', choices=('both','left','right'), default='both')
    parser.add_argument('--engage-mode', choices=('shared', 'independent'), default=None,
                        help='Both arms default to shared; single arm uses its own button')
    parser.add_argument('--engage-side', choices=('left', 'right'), default=None,
                        help='Shared button source, defaults to right')
    parser.add_argument('--mount-calibration', type=Path)
    parser.add_argument('--position-scale', type=float, default=1.0)
    parser.add_argument('--pose-shaper-bandwidth', type=float, default=25.0,
                        help='Critically damped end-effector reference shaper bandwidth in rad/s')
    parser.add_argument('--tracking-mode', choices=('strict', 'dexarena_velocity'), default='strict',
                        help='strict constraint rejection or DexArena-style offline uniform velocity-limited IK catch-up')
    parser.add_argument('--motion-mapping', choices=('legacy-local', 'tabletop-relative'), default='legacy-local',
                        help='Legacy grip-relative anchoring, or shared tabletop translation and spatial rotation; offline opt-in')
    parser.add_argument('--offline-acceleration-limit', type=float, default=None,
                        help='Diagnostic override in rad/s^2; affects offline IK only, never the controller')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.side != 'both' and (args.engage_mode == 'shared' or args.engage_side is not None):
        parser.error('Shared engagement parameters require --side both; single arm uses its own button')
    if args.engage_mode == 'independent' and args.engage_side is not None:
        parser.error('--engage-side applies only to shared engagement')
    if args.side == 'both':
        result = replay_bilateral(args.recording,args.initial_snapshot,args.output,args.mount_calibration,args.position_scale,
                                  args.engage_mode or 'shared', args.engage_side or 'right',
                                  args.offline_acceleration_limit, args.pose_shaper_bandwidth, args.tracking_mode, args.motion_mapping)
    else:
        result = replay(args.recording,args.initial_snapshot,args.output,args.side,args.mount_calibration,
                        args.position_scale,args.offline_acceleration_limit,args.pose_shaper_bandwidth,args.tracking_mode,args.motion_mapping)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
