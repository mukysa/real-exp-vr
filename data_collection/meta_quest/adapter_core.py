"""ROS-independent A/B adapter and final command gate.

The core consumes only the strict contract from ``fr3_ik.interfaces``.  It
never publishes and it never converts a diagnostic reference into a command.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from data_collection.fr3_ik.interfaces import (
    IKResult,
    JOINT_NAMES,
    JointTarget,
    MeasuredJointState,
    ResultState,
    Timestamp,
)


POSITION_LOWER = (-2.6937, -1.7337, -2.8507, -2.9921, -2.7565, 0.5945, -2.9659)
POSITION_UPPER = (2.6937, 1.7337, 2.8507, -0.2018, 2.7565, 4.4669, 2.9659)
MAX_VELOCITY = (0.60, 0.60, 0.60, 0.60, 0.75, 0.75, 0.75)


class AdapterState(str, Enum):
    WAIT_ROBOT = "WAIT_ROBOT"
    RUNNING = "RUNNING"
    HOLD = "HOLD"
    STOP = "STOP"

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True, slots=True)
class GateDecision:
    state: AdapterState
    would_publish: bool
    reject_reason: str | None
    joint_target: JointTarget | None = None

    @property
    def publish(self) -> bool:
        return self.would_publish

    @property
    def reason(self) -> str:
        return self.reject_reason or "accepted"

    @property
    def positions(self) -> tuple[float, ...] | None:
        return None if self.joint_target is None else self.joint_target.joints.positions_rad


class AdapterCore:
    """Stateful final gate shared by offline validation and the ROS wrapper."""

    def __init__(
        self,
        *,
        side: str,
        robot_state_timeout_s: float = 0.25,
        input_timeout_s: float = 0.25,
        ik_timeout_s: float = 0.15,
        max_joint_step_rad: float = 0.10,
        velocity_scale: float = 1.0,
        initial_match_tolerance_rad: float = 1e-4,
        expected_sequence_step: int | None = 2,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        for name, value in (
            ("robot_state_timeout_s", robot_state_timeout_s),
            ("input_timeout_s", input_timeout_s),
            ("ik_timeout_s", ik_timeout_s),
            ("max_joint_step_rad", max_joint_step_rad),
            ("velocity_scale", velocity_scale),
            ("initial_match_tolerance_rad", initial_match_tolerance_rad),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if expected_sequence_step is not None and expected_sequence_step < 1:
            raise ValueError("expected_sequence_step must be positive or None")
        self.side = side
        self.robot_state_timeout_s = robot_state_timeout_s
        self.input_timeout_s = input_timeout_s
        self.ik_timeout_s = ik_timeout_s
        self.max_joint_step_rad = max_joint_step_rad
        self.velocity_scale = velocity_scale
        self.initial_match_tolerance_rad = initial_match_tolerance_rad
        self.expected_sequence_step = expected_sequence_step
        self.previous_target: JointTarget | None = None
        self.connection_id: int | None = None
        self.source_sequence: int | None = None
        self.fault_latched = False
        self.reanchor_required = True

    def reset_for_reanchor(self) -> None:
        """Explicit operator/state-machine recovery; the next target must equal measured q."""
        self.previous_target = None
        self.source_sequence = None
        self.fault_latched = False
        self.reanchor_required = True

    @staticmethod
    def _age(now: Timestamp, earlier: Timestamp) -> float:
        return now.elapsed_since(earlier)

    def _reject(self, state: AdapterState, reason: str, *, latch: bool = False) -> GateDecision:
        if latch:
            self.fault_latched = True
            self.previous_target = None
            self.reanchor_required = True
        return GateDecision(state, False, reason)

    def assess(
        self,
        result: IKResult,
        measured_state: MeasuredJointState | None,
        now: Timestamp,
        *,
        quest_connected: bool = True,
        gello_conflict: bool = False,
    ) -> GateDecision:
        if not isinstance(result, IKResult):
            return self._reject(AdapterState.STOP, "ik_contract_type", latch=True)
        if gello_conflict:
            return self._reject(AdapterState.STOP, "command_source_conflict", latch=True)
        if not quest_connected:
            return self._reject(AdapterState.STOP, "quest_disconnected", latch=True)
        if measured_state is None:
            return self._reject(AdapterState.WAIT_ROBOT, "robot_state_missing")
        if not measured_state.valid:
            return self._reject(AdapterState.STOP, "robot_state_invalid", latch=True)
        if result.request.side != self.side:
            return self._reject(AdapterState.STOP, "side_mismatch", latch=True)
        try:
            robot_age = self._age(now, measured_state.stamp)
            input_age = self._age(now, result.request.source_stamp)
            ik_age = self._age(now, result.computed_stamp)
        except ValueError:
            return self._reject(AdapterState.STOP, "clock_domain_mismatch", latch=True)
        if robot_age < 0 or robot_age > self.robot_state_timeout_s:
            return self._reject(AdapterState.STOP, "robot_state_stale", latch=True)
        if input_age < 0 or input_age > self.input_timeout_s:
            return self._reject(AdapterState.STOP, "input_stale", latch=True)
        if ik_age < 0 or ik_age > self.ik_timeout_s:
            return self._reject(AdapterState.STOP, "ik_result_stale", latch=True)

        # A source-side recovery is emitted only after the operator presses Y.
        # It must be a fresh, fully checked FOLLOWING result whose command
        # matches measured q.  That exact measured-state anchor is the only
        # message allowed to clear a previously latched adapter fault.
        if self.fault_latched:
            recovery = (
                result.success
                and result.state == ResultState.FOLLOWING
                and result.command is not None
                and result.checks.all_passed
                and result.detail == "Measured-state recovery anchor before bounded return"
                and max(abs(value - actual) for value, actual in zip(
                    result.command.joints.positions_rad,
                    measured_state.joints.positions_rad,
                )) <= self.initial_match_tolerance_rad
            )
            if not recovery:
                return self._reject(AdapterState.STOP, "fault_latched_reanchor_required")
            self.reset_for_reanchor()

        connection_id = result.request.target.source_connection_id
        sequence = result.request.target.source_sequence
        if self.connection_id is None:
            self.connection_id = connection_id
        elif connection_id != self.connection_id:
            self.connection_id = connection_id
            self.source_sequence = None
            self.reanchor_required = True
            return self._reject(AdapterState.HOLD, "connection_changed_reanchor_required")
        if self.source_sequence is not None:
            delta = sequence - self.source_sequence
            if delta <= 0:
                return self._reject(AdapterState.STOP, "source_sequence_not_increasing", latch=True)
            if self.expected_sequence_step is not None and delta != self.expected_sequence_step:
                return self._reject(AdapterState.STOP, "source_sequence_gap", latch=True)
        self.source_sequence = sequence

        if not result.success:
            state = AdapterState.HOLD if result.state == ResultState.HOLD else AdapterState.STOP
            return self._reject(state, f"ik_{result.state.value.lower()}:{result.reason.value}",
                                latch=state == AdapterState.STOP)
        if result.state != ResultState.FOLLOWING or result.command is None:
            return self._reject(AdapterState.STOP, "invalid_following_contract", latch=True)
        if not result.checks.all_passed:
            return self._reject(AdapterState.STOP, "ik_constraints_failed", latch=True)

        positions = result.command.joints.positions_rad
        if result.command.joints.names != JOINT_NAMES:
            return self._reject(AdapterState.STOP, "joint_order_mismatch", latch=True)
        if any(value < lower or value > upper
               for value, lower, upper in zip(positions, POSITION_LOWER, POSITION_UPPER)):
            return self._reject(AdapterState.STOP, "joint_position_limit", latch=True)

        measured = measured_state.joints.positions_rad
        if self.reanchor_required:
            if max(abs(value - actual) for value, actual in zip(positions, measured)) > self.initial_match_tolerance_rad:
                return self._reject(AdapterState.HOLD, "initial_target_must_equal_measured")
            self.reanchor_required = False
        reference = measured if self.previous_target is None else self.previous_target.joints.positions_rad
        delta = tuple(abs(value - old) for value, old in zip(positions, reference))
        if max(delta) > self.max_joint_step_rad:
            return self._reject(AdapterState.STOP, "joint_step_limit", latch=True)
        if self.previous_target is not None:
            try:
                elapsed = result.command.stamp.elapsed_since(self.previous_target.stamp)
            except ValueError:
                return self._reject(AdapterState.STOP, "target_clock_changed", latch=True)
            if elapsed <= 0:
                return self._reject(AdapterState.STOP, "target_time_not_increasing", latch=True)
            if any(step / elapsed > limit * self.velocity_scale
                   for step, limit in zip(delta, MAX_VELOCITY)):
                return self._reject(AdapterState.STOP, "joint_velocity_limit", latch=True)

        self.previous_target = result.command
        return GateDecision(AdapterState.RUNNING, True, None, result.command)
