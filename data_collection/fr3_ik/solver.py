"""Stateful offline IK with latched faults and explicit reanchoring.

No ROS node, publisher, socket or hardware backend. Wall-clock timeout is an
optional post-return rejection, NOT a hard deadline or a real-time guarantee.
"""
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from data_collection.move_to_target_ee import solve_fr3_ik, ARM_POSITION_LOWER_RAD, ARM_POSITION_UPPER_RAD
from .constraints import Limits, check_candidate, residual
from .dexarena_tracking import differential_step
from .joint_reference import BoundedJointReference
from .interfaces import (ConstraintChecks, FailureReason, IKResult, JointTarget, JointVector,
                         ResultState)


@dataclass(frozen=True)
class DiagnosticIKResult:
    """Offline-only rate-limited target; deliberately not an IKResult command."""
    request: object
    state: ResultState
    reason: FailureReason
    detail: str
    command: JointTarget | None
    last_safe_target: JointTarget
    solve_time_s: float
    position_residual_m: float | None
    rotation_residual_rad: float | None
    checks: ConstraintChecks
    success: bool
    control_eligible: bool = False


def matrix_from_request(request):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(request.target.pose.quaternion_xyzw).as_matrix()
    matrix[:3, 3] = request.target.pose.position
    return matrix


class ContinuousIK:
    def __init__(self, kinematics, limits=None, max_evaluations=80, timeout_s=None,
                 tracking_mode='strict'):
        self.kinematics = kinematics
        self.limits = limits or Limits.from_controller()
        if not np.array_equal(self.limits.lower, ARM_POSITION_LOWER_RAD) or not np.array_equal(self.limits.upper, ARM_POSITION_UPPER_RAD):
            raise ValueError('Controller and existing IK bounds differ')
        if max_evaluations < 1 or (timeout_s is not None and (not np.isfinite(timeout_s) or timeout_s <= 0)):
            raise ValueError('Invalid solver budget')
        self.max_evaluations = max_evaluations
        self.timeout_s = timeout_s
        if tracking_mode not in {'strict', 'dexarena_velocity'}:
            raise ValueError('Unknown IK tracking mode')
        self.tracking_mode = tracking_mode
        self.last = None
        self.fault = None
        self.diagnostics = {}

    def advance_stop(self, stamp, dt):
        """Offline bounded braking, separate from accepting any new IK target."""
        q, info = self.reference.advance(self.reference.position, dt, stop=True)
        self.last = JointTarget(JointVector(tuple(q)), stamp)
        self.velocity = self.reference.velocity.copy()
        self.diagnostics.update(info)
        return self.last

    def reset(self, anchor, connection_id):
        q = np.asarray(anchor.joints.positions_rad)
        if np.any(q < self.limits.lower) or np.any(q > self.limits.upper):
            raise ValueError('Anchor outside controller envelope')
        if type(connection_id) is not int or connection_id < 0:
            raise ValueError('Invalid connection id')
        self.last = anchor
        self.last_stamp = anchor.stamp
        self.connection_id = connection_id
        self.last_sequence = 0
        self.velocity = np.zeros(7)
        self.last_requested_pose = self.kinematics.fk(q)
        self.fault = None
        self.needs_reanchor = False
        self.diagnostics = {}
        self.reference = BoundedJointReference(self.limits, q)
        self.first_follow = True
        self.stationary_target = self.last_requested_pose.copy()
        self.stationary_q = q.copy()

    def solve(self, target_pose, previous_joint_target, measured_joint_state=None):
        if self.last is None:
            raise ValueError('Explicit reset(anchor, connection_id) required before solve')
        request = target_pose
        started = time.perf_counter()
        self.diagnostics = {'candidate_q': None, 'candidate_velocity': None, 'candidate_acceleration': None,
                            'evaluations': 0, 'ik_attempted': False}

        def result(state, reason, detail, checks=ConstraintChecks(), errors=(None, None), command=None):
            if self.tracking_mode == 'dexarena_velocity' and state == ResultState.FOLLOWING:
                return DiagnosticIKResult(request, state, reason, detail, command, self.last,
                                          time.perf_counter() - started, *errors, checks, True)
            return IKResult(request, state, reason, detail, command, self.last,
                            time.perf_counter() - started, *errors, checks)

        def fail(reason, detail, checks=ConstraintChecks(), errors=(None, None), reanchor=False):
            if self.tracking_mode == 'dexarena_velocity':
                self.reference.position = np.asarray(self.last.joints.positions_rad).copy()
                self.reference.velocity = self.velocity.copy()
            self.fault = reason
            return result(ResultState.REANCHOR_REQUIRED if reanchor else ResultState.FAULT,
                          reason, detail, checks, errors)

        if previous_joint_target != self.last:
            return fail(FailureReason.INVALID_INPUT, 'Previous target does not match solver reference')
        if request.target.source_connection_id != self.connection_id:
            return fail(FailureReason.CONNECTION_CHANGED, 'Connection changed; reset and reanchor required', reanchor=True)
        if self.fault is not None:
            return result(ResultState.REANCHOR_REQUIRED, self.fault, 'Fault latched; explicit reset required')
        try:
            dt = request.target_stamp.elapsed_since(self.last_stamp)
        except ValueError:
            return fail(FailureReason.INVALID_INPUT, 'Clock domain/session changed')
        if dt <= 0 or request.target.source_sequence <= self.last_sequence:
            return fail(FailureReason.INVALID_INPUT, 'Non-increasing timestamp or sequence')
        if dt > self.limits.input_gap_s:
            return fail(FailureReason.STALE_INPUT, 'Input gap exceeded; do not catch up')
        if measured_joint_state is not None:
            try:
                age = request.target_stamp.elapsed_since(measured_joint_state.stamp)
            except ValueError:
                return fail(FailureReason.INVALID_INPUT, 'Measured state requires explicit clock conversion')
            if not measured_joint_state.valid or age < 0 or age > self.limits.input_gap_s:
                return fail(FailureReason.STALE_ROBOT_STATE, 'Invalid or stale measured feedback')
        self.last_stamp = request.target_stamp
        self.last_sequence = request.target.source_sequence
        if not request.target.valid:
            if request.target.state in {'WAIT_INPUT', 'READY', 'MANUAL_HOLD'}:
                self.needs_reanchor = True
                if self.tracking_mode == 'strict':
                    self.velocity = np.zeros(7)
                return result(ResultState.HOLD, FailureReason.NONE, request.target.hold_reason or request.target.state)
            return fail(FailureReason.INVALID_INPUT, request.target.hold_reason or 'Upstream fault')
        if self.needs_reanchor:
            return fail(FailureReason.REANCHOR_REQUIRED, 'Reanchor after manual hold before following', reanchor=True)
        desired = matrix_from_request(request)
        jump_m, jump_rad = residual(desired, self.last_requested_pose)
        if np.linalg.norm(desired[:3, 3]) > 1.5:
            return fail(FailureReason.UNREACHABLE, 'Outside conservative 1.5 m base-radius precheck')
        if jump_m > self.limits.source_jump_m or jump_rad > self.limits.source_jump_rad:
            return fail(FailureReason.STEP_LIMIT, 'End-effector target discontinuity')
        self.diagnostics['ik_attempted'] = True
        try:
            if self.tracking_mode == 'dexarena_velocity':
                # Centering is disabled for this execution-reference policy:
                # holding a hand pose must not drive a redundant joint posture.
                # Keep the original default step available for V2 comparison.
                changed = max(residual(desired, self.stationary_target)) > 1e-10
                if changed:
                    self.stationary_target = desired.copy()
                    self.stationary_q = None
                if self.first_follow:
                    q = np.asarray(self.last.joints.positions_rad).copy()
                    tracking = dict(takeover_anchor=True)
                elif self.stationary_q is not None:
                    q = self.stationary_q.copy()
                    tracking = dict(stationary_latched=True)
                else:
                    q, tracking = differential_step(self.kinematics,
                        np.asarray(self.last.joints.positions_rad), desired, dt,
                        self.limits.lower, self.limits.upper, null_gain=0.0, limit_margin=0.0,
                        limit_velocity=False)
                    if max(residual(self.kinematics.fk(self.reference.position), desired)) < 1e-8 and np.max(np.abs(self.velocity)) < 1e-7:
                        self.stationary_q = self.reference.position.copy()
                        q = self.stationary_q.copy()
                self.diagnostics['desired_joint_rad'] = q.tolist()
                q, reference_info = self.reference.advance(q, dt)
                tracking.update(reference_info, null_gain=0.0)
                self.diagnostics.update(tracking)
            else:
                candidate = solve_fr3_ik(np.asarray(self.last.joints.positions_rad), desired,
                                     self.kinematics.tool, kinematics=self.kinematics,
                                     try_alternative_seeds=False, max_function_evaluations=self.max_evaluations)
                q = candidate.q
                self.diagnostics['evaluations'] = candidate.function_evaluations
        except RuntimeError as exc:
            return fail(FailureReason.UNREACHABLE, 'No verified solution within budget: ' + str(exc))
        except (ValueError, np.linalg.LinAlgError) as exc:
            return fail(FailureReason.SOLVER_ERROR, str(exc))
        self.diagnostics['candidate_q'] = q.tolist()
        # Fresh full-model FK, separate from the optimizer's arm model workspace.
        _, achieved = self.kinematics.geometry(q)
        errors = residual(achieved, desired)
        previous_q = np.asarray(self.last.joints.positions_rad)
        checks, reason, velocity, acceleration = check_candidate(q, previous_q,
                                                                self.velocity, dt, *errors, self.limits)
        if self.tracking_mode == 'dexarena_velocity':
            # Lag is an expected result of the differential servo. Strict A3
            # checks stay truthful diagnostics; they are not acceptance gates
            # for this separate, offline-only result type.
            for passed, failure in ((checks.joint_limits, FailureReason.JOINT_LIMIT),
                                    (checks.step, FailureReason.STEP_LIMIT),
                                    (checks.velocity, FailureReason.VELOCITY_LIMIT),
                                    (checks.acceleration, FailureReason.ACCELERATION_LIMIT)):
                if not passed:
                    reason = failure
                    break
            else:
                reason = FailureReason.NONE
        self.diagnostics.update(candidate_velocity=velocity.tolist(), candidate_acceleration=acceleration.tolist())
        if self.timeout_s is not None and time.perf_counter() - started > self.timeout_s:
            return fail(FailureReason.SOLVE_TIMEOUT, 'Post-return wall-clock deadline exceeded', checks, errors)
        if reason != FailureReason.NONE:
            return fail(reason, 'Candidate rejected; last target preserved', checks, errors)
        self.last = JointTarget(JointVector(tuple(q)), request.target_stamp)
        self.velocity = velocity
        self.last_requested_pose = desired
        self.first_follow = False
        detail = 'Offline DexArena differential target' if self.tracking_mode == 'dexarena_velocity' else 'Checked IK target'
        return result(ResultState.FOLLOWING, FailureReason.NONE, detail, checks, errors, self.last)

    def reject_input(self, request, detail):
        """Transport validation failed before an invalid pose could form IKRequest."""
        if self.last is None:
            raise ValueError('reset required')
        self.fault = FailureReason.INVALID_INPUT
        self.diagnostics = {'candidate_q': None, 'candidate_velocity': None, 'candidate_acceleration': None,
                            'evaluations': 0, 'ik_attempted': False}
        return IKResult(request, ResultState.FAULT, self.fault, detail, None, self.last, 0)
