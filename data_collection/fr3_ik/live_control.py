"""Safety-bounded left-arm Quest control core with explicit Y-button return.

This module contains no ROS publisher and opens no robot connection.  The ROS
wrapper may publish only strict :class:`IKResult` values returned here.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import time

import numpy as np
from scipy.spatial.transform import Rotation

from data_collection.meta_quest.models import TargetPose
from data_collection.meta_quest.receiver import ReceivedPose
from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.state_machine import DebouncedButtonEdge

from .a3_replay import PoseReferenceShaper
from .constraints import Limits, check_candidate, residual
from .interfaces import (
    ClockDomain,
    ConstraintChecks,
    FailureReason,
    IKRequest,
    IKResult,
    JointTarget,
    JointVector,
    ResultState,
    Timestamp,
)
from .joint_reference import BoundedJointReference
from .preview_mapping import converted_matrix
from .solver import ContinuousIK


@dataclass(frozen=True)
class ControlUpdate:
    state: str
    reason: str
    result: IKResult | None = None


class LeftLiveControl:
    """Convert left-controller poses into checked targets for one left FR3."""

    def __init__(
        self,
        kinematics,
        *,
        clock_id: str,
        position_scale: float = 0.03,
        max_tcp_offset_m: float = 0.005,
        pose_shaper_bandwidth: float = 5.0,
        measured_timeout_s: float = 0.15,
        input_timeout_s: float = 1.0,
        relaxed_motion_gates: bool = False,
        max_source_translation_step_m: float = 0.15,
        max_source_rotation_step_rad: float = 0.75,
        world_from_base: np.ndarray | None = None,
    ) -> None:
        if not clock_id.strip():
            raise ValueError("clock_id is required")
        if not 0.0 < position_scale <= 1.0:
            raise ValueError("position_scale must be in (0, 1]")
        if not 0.0 <= max_tcp_offset_m <= 0.30:
            raise ValueError("max_tcp_offset_m must be in [0, 0.30]; 0 disables the radial cap")
        self.kin = kinematics
        self.clock_id = clock_id
        self.position_scale = float(position_scale)
        self.max_tcp_offset_m = float(max_tcp_offset_m)
        self.measured_timeout_s = float(measured_timeout_s)
        self.input_timeout_s = float(input_timeout_s)
        self.limits = Limits.from_controller()
        if relaxed_motion_gates:
            # The 1 kHz hardware JointReferenceGenerator remains the motion
            # limiter.  Live IK may submit any finite in-envelope target
            # without duplicating its velocity/acceleration gates here.
            self.limits = replace(
                self.limits,
                velocity=(1000.0,) * 7,
                acceleration=(100000.0,) * 7,
                max_step_rad=6.0,
                input_gap_s=self.input_timeout_s,
                source_jump_m=float(max_source_translation_step_m),
                source_jump_rad=float(max_source_rotation_step_rad),
            )
        self.relaxed_motion_gates = bool(relaxed_motion_gates)
        self.max_source_translation_step_m = float(max_source_translation_step_m)
        self.max_source_rotation_step_rad = float(max_source_rotation_step_rad)
        if world_from_base is None:
            world_from_base = np.eye(4)
        world_from_base = np.asarray(world_from_base, dtype=float)
        if (
            world_from_base.shape != (4, 4)
            or not np.isfinite(world_from_base).all()
            or not np.allclose(world_from_base[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9)
            or not np.allclose(
                world_from_base[:3, :3].T @ world_from_base[:3, :3],
                np.eye(3),
                atol=1e-6,
            )
            or not np.isclose(np.linalg.det(world_from_base[:3, :3]), 1.0, atol=1e-6)
        ):
            raise ValueError("world_from_base must be a finite rigid 4x4 transform")
        self.world_from_base = world_from_base.copy()
        self.base_from_world_rotation = self.world_from_base[:3, :3].T
        self.translation_frame = (
            "base_axes" if np.allclose(self.world_from_base, np.eye(4), atol=1e-12)
            else "tabletop_world_calibrated"
        )
        self.solver = ContinuousIK(self.kin, limits=self.limits, max_evaluations=120)
        self.shaper = PoseReferenceShaper(pose_shaper_bandwidth)
        self.button = DebouncedButtonEdge(0.15)
        self.button_initialized = False
        self.state = "WAITING"
        self.reason = "waiting for fresh measured state and left controller"
        self.measured_q: np.ndarray | None = None
        self.measured_dq: np.ndarray | None = None
        self.measured_stamp: float | None = None
        self.connection_id: int | None = None
        self.last_pose_stamp: float | None = None
        self.last_source: np.ndarray | None = None
        self.healthy_samples = 0
        self.home_q: np.ndarray | None = None
        self.home_pose: np.ndarray | None = None
        self.source_anchor: np.ndarray | None = None
        self.return_reference: BoundedJointReference | None = None
        self.return_last_stamp: float | None = None
        self.last_command: JointTarget | None = None

    def update_measured(self, q, dq, received_monotonic_s: float) -> None:
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)
        if q.shape != (7,) or dq.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(dq).all():
            self._fault("invalid measured joint state")
            return
        if np.any(q < self.limits.lower) or np.any(q > self.limits.upper):
            self._fault("measured joints outside controller envelope")
            return
        self.measured_q = q.copy()
        self.measured_dq = dq.copy()
        self.measured_stamp = float(received_monotonic_s)

    def _stamp(self, seconds: float) -> Timestamp:
        return Timestamp(float(seconds), ClockDomain.HOST_MONOTONIC, self.clock_id)

    @staticmethod
    def _rigid(matrix: np.ndarray) -> RigidPose:
        return RigidPose(
            tuple(np.asarray(matrix[:3, 3], dtype=float)),
            tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()),
        )

    def _request(self, received: ReceivedPose, pose: np.ndarray) -> IKRequest:
        stamp = self._stamp(received.received_monotonic_s)
        target = TargetPose(
            self._rigid(pose),
            "left_fr3_link0",
            received.sequence,
            received.connection_id,
            stamp.seconds,
            "FOLLOWING",
            True,
            None,
        )
        return IKRequest(target, stamp, stamp, "left_fr3_hand_tcp", "left")

    def _joint_result(
        self,
        received: ReceivedPose,
        q: np.ndarray,
        previous_q: np.ndarray,
        previous_velocity: np.ndarray,
        dt: float,
        detail: str,
    ) -> IKResult:
        pose = self.kin.fk(q)
        position_error, rotation_error = residual(self.kin.fk(q), pose)
        checks, reason, velocity, _ = check_candidate(
            q,
            previous_q,
            previous_velocity,
            dt,
            position_error,
            rotation_error,
            self.limits,
        )
        request = self._request(received, pose)
        if reason != FailureReason.NONE:
            self._fault(f"return target rejected: {reason.value}")
            assert self.last_command is not None
            return IKResult(
                request,
                ResultState.FAULT,
                reason,
                self.reason,
                None,
                self.last_command,
                0.0,
                position_error,
                rotation_error,
                checks,
                self._stamp(time.monotonic()),
            )
        command = JointTarget(JointVector(tuple(q)), request.target_stamp)
        self.last_command = command
        return IKResult(
            request,
            ResultState.FOLLOWING,
            FailureReason.NONE,
            detail,
            command,
            command,
            0.0,
            position_error,
            rotation_error,
            checks,
            self._stamp(time.monotonic()),
        )

    def _fault(self, reason: str) -> None:
        self.state = "FAULT"
        self.reason = reason
        self.button.require_release()

    def _fresh_measured(self, now: float) -> bool:
        # Pose and joint-state callbacks are independent.  A joint sample may
        # legitimately be received just after the pose currently being
        # processed, so accept bounded skew in either direction.
        return (
            self.measured_q is not None
            and self.measured_dq is not None
            and self.measured_stamp is not None
            and abs(now - self.measured_stamp) <= self.measured_timeout_s
        )

    def _engage(self, received: ReceivedPose, source: np.ndarray) -> IKResult | None:
        now = received.received_monotonic_s
        if not self._fresh_measured(now):
            self.reason = "Y rejected: measured state is not fresh"
            return None
        assert self.measured_q is not None and self.measured_dq is not None
        if float(np.max(np.abs(self.measured_dq))) > 0.05:
            self.reason = "Y rejected: robot is not stationary"
            return None
        self.home_q = self.measured_q.copy()
        self.home_pose = self.kin.fk(self.home_q)
        self.source_anchor = source.copy()
        anchor = JointTarget(JointVector(tuple(self.home_q)), self._stamp(now))
        self.solver.reset(anchor, received.connection_id)
        self.solver.last_sequence = received.sequence
        self.shaper.reset(self.home_pose[:3, 3], Rotation.from_matrix(self.home_pose[:3, :3]))
        self.last_command = anchor
        self.state = "FOLLOWING"
        self.reason = "Y engaged; current measured pose is the return anchor"
        request = self._request(received, self.home_pose)
        return IKResult(
            request,
            ResultState.FOLLOWING,
            FailureReason.NONE,
            "Measured-state takeover anchor",
            anchor,
            anchor,
            0.0,
            0.0,
            0.0,
            ConstraintChecks(True, True, True, True, True),
            self._stamp(time.monotonic()),
        )

    def _start_return(self, received: ReceivedPose) -> IKResult:
        assert self.home_q is not None and self.last_command is not None
        current = np.asarray(self.last_command.joints.positions_rad, dtype=float)
        self.return_reference = BoundedJointReference(self.limits, current)
        self.return_reference.velocity = self.solver.velocity.copy()
        self.return_last_stamp = self.last_command.stamp.seconds
        self.state = "RETURNING"
        self.reason = "second Y accepted; returning to takeover anchor"
        return self._return(received)

    def _recover_return(self, received: ReceivedPose) -> IKResult | None:
        """Re-anchor a stopped episode at measured q, then return on Y."""

        now = received.received_monotonic_s
        if not self._fresh_measured(now):
            self.reason = "Y return rejected: measured state is not fresh"
            return None
        if self.home_q is None or self.measured_q is None or self.measured_dq is None:
            self.reason = "Y return rejected: takeover anchor is unavailable"
            return None
        current = self.measured_q.copy()
        anchor = JointTarget(JointVector(tuple(current)), self._stamp(now))
        self.last_command = anchor
        self.return_reference = BoundedJointReference(self.limits, current)
        self.return_reference.velocity = self.measured_dq.copy()
        self.return_last_stamp = now
        self.state = "RETURNING"
        self.reason = "Y accepted after stop; returning from measured state"
        request = self._request(received, self.kin.fk(current))
        return IKResult(
            request,
            ResultState.FOLLOWING,
            FailureReason.NONE,
            "Measured-state recovery anchor before bounded return",
            anchor,
            anchor,
            0.0,
            0.0,
            0.0,
            ConstraintChecks(True, True, True, True, True),
            self._stamp(time.monotonic()),
        )

    def _return(self, received: ReceivedPose) -> IKResult:
        assert self.home_q is not None
        assert self.return_reference is not None
        assert self.return_last_stamp is not None
        assert self.last_command is not None
        dt = received.received_monotonic_s - self.return_last_stamp
        if not 0.0 < dt <= self.input_timeout_s:
            self._fault("return input timing invalid")
            request = self._request(received, self.kin.fk(np.asarray(self.last_command.joints.positions_rad)))
            return IKResult(
                request,
                ResultState.FAULT,
                FailureReason.STALE_INPUT,
                self.reason,
                None,
                self.last_command,
                0.0,
                computed_stamp=self._stamp(time.monotonic()),
            )
        previous_q = self.return_reference.position.copy()
        previous_velocity = self.return_reference.velocity.copy()
        q, _ = self.return_reference.advance(self.home_q, dt)
        self.return_last_stamp = received.received_monotonic_s
        result = self._joint_result(
            received,
            q,
            previous_q,
            previous_velocity,
            dt,
            "Bounded return to Y takeover anchor",
        )
        if (
            self.state != "FAULT"
            and np.max(np.abs(q - self.home_q)) <= 1e-3
            and np.max(np.abs(self.return_reference.velocity)) <= 0.01
            and self.measured_q is not None
            and self.measured_dq is not None
            and np.max(np.abs(self.measured_q - self.home_q)) <= 0.02
            and np.max(np.abs(self.measured_dq)) <= 0.05
        ):
            self.state = "READY"
            self.reason = "return complete; release Y before next takeover"
            self.button.require_release()
        return result

    def process(self, received: ReceivedPose) -> ControlUpdate:
        sample = received.sample
        if sample.side != "left" or sample.source != "controller":
            return ControlUpdate(self.state, self.reason)
        if not sample.tracked:
            self._fault("left controller tracking invalid")
            return ControlUpdate(self.state, self.reason)
        source = converted_matrix(sample)
        now = received.received_monotonic_s
        if self.connection_id is None:
            self.connection_id = received.connection_id
            self.button.reset(bool(sample.clutch))
            self.button_initialized = True
        elif received.connection_id != self.connection_id:
            self._fault("Quest connection changed; restart required")
            return ControlUpdate(self.state, self.reason)
        if self.last_pose_stamp is not None:
            dt = now - self.last_pose_stamp
            if not 0.0 < dt <= self.input_timeout_s:
                self._fault("Quest input gap; restart required")
                return ControlUpdate(self.state, self.reason)
            if self.state == "FOLLOWING" and self.last_source is not None:
                step = float(np.linalg.norm(source[:3, 3] - self.last_source[:3, 3]))
                angle = float(Rotation.from_matrix(source[:3, :3] @ self.last_source[:3, :3].T).magnitude())
                if (step > self.max_source_translation_step_m
                        or angle > self.max_source_rotation_step_rad):
                    self._fault("Quest pose discontinuity; restart required")
                    return ControlUpdate(self.state, self.reason)
        self.last_pose_stamp = now
        self.last_source = source.copy()
        edge = self.button.update(bool(sample.clutch), now)

        if self.state == "FAULT":
            result = self._recover_return(received) if edge else None
            return ControlUpdate(self.state, self.reason, result)

        if self.state == "WAITING":
            self.healthy_samples += 1
            if self.healthy_samples >= 3 and self._fresh_measured(now):
                self.state = "READY"
                self.reason = "ready; press left Y to take over"
            return ControlUpdate(self.state, self.reason)
        if self.state == "READY":
            result = self._engage(received, source) if edge else None
            return ControlUpdate(self.state, self.reason, result)
        if self.state == "RETURNING":
            result = self._return(received)
            return ControlUpdate(self.state, self.reason, result)
        if edge:
            result = self._start_return(received)
            return ControlUpdate(self.state, self.reason, result)

        assert self.state == "FOLLOWING"
        assert self.home_pose is not None and self.source_anchor is not None
        target = self.home_pose.copy()
        delta_world = source[:3, 3] - self.source_anchor[:3, 3]
        delta_base = self.base_from_world_rotation @ delta_world
        target[:3, 3] = self.home_pose[:3, 3] + self.position_scale * delta_base
        if (
            self.max_tcp_offset_m > 0.0
            and float(np.linalg.norm(target[:3, 3] - self.home_pose[:3, 3]))
            > self.max_tcp_offset_m
        ):
            limit_mm = self.max_tcp_offset_m * 1000.0
            self._fault(f"{limit_mm:g} mm TCP workspace exceeded; restart required")
            return ControlUpdate(self.state, self.reason)
        assert self.last_command is not None
        dt = now - self.last_command.stamp.seconds
        shaped = self.shaper.update(target, dt)
        request = self._request(received, shaped)
        # The hardware-side adapter independently checks fresh measured robot
        # state immediately before publishing.  Repeating that asynchronous
        # timing gate here caused false stale-state faults under normal ROS
        # callback scheduling, while adding no protection at the output gate.
        result = self.solver.solve(request, self.solver.last, None)
        result = replace(result, computed_stamp=self._stamp(time.monotonic()))
        if result.success:
            self.last_command = result.command
            self.reason = "following left controller; second Y returns home"
        else:
            self._fault(f"IK stopped: {result.reason.value}")
            # Do not latch the hardware adapter for a source-side IK stop.  Its
            # watchdog stops new motion, and a subsequent Y can re-anchor at
            # measured q and execute the bounded return above.
            return ControlUpdate(self.state, self.reason)
        return ControlUpdate(self.state, self.reason, result)

    def tick(self, now: float) -> ControlUpdate:
        if self.state in {"FOLLOWING", "RETURNING"}:
            if self.last_pose_stamp is None or now - self.last_pose_stamp > self.input_timeout_s:
                self._fault("Quest stream stale; controller watchdog will stop")
            elif not self._fresh_measured(now):
                self._fault("measured robot state stale; controller watchdog will stop")
        return ControlUpdate(self.state, self.reason)
