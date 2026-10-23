"""Pure-Python validation and watchdog policy for stage-4 joint commands."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from .models import IKResult


COMMAND_JOINT_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
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
    publish: bool
    reason: str
    positions: tuple[float, ...] | None = None


class JointCommandGate:
    """Reject stale or unsafe IK results before they reach a ROS command topic.

    HOLD and STOP both stop publishing, so the controller's own watchdog makes
    a bounded stop.  HOLD is a source-level pause that can resume with a fresh,
    re-anchored result.  STOP denotes a transport, state, or command-source
    fault and clears the previous target before recovery.
    """

    def __init__(
        self,
        *,
        robot_state_timeout_s: float = 0.25,
        input_timeout_s: float = 0.25,
        ik_timeout_s: float = 0.15,
        max_joint_step_rad: float = 0.10,
        velocity_scale: float = 1.0,
    ) -> None:
        for name, value in (
            ("robot_state_timeout_s", robot_state_timeout_s),
            ("input_timeout_s", input_timeout_s),
            ("ik_timeout_s", ik_timeout_s),
            ("max_joint_step_rad", max_joint_step_rad),
            ("velocity_scale", velocity_scale),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        self.robot_state_timeout_s = robot_state_timeout_s
        self.input_timeout_s = input_timeout_s
        self.ik_timeout_s = ik_timeout_s
        self.max_joint_step_rad = max_joint_step_rad
        self.velocity_scale = velocity_scale
        self.robot_positions: tuple[float, ...] | None = None
        self.robot_state_time_s: float | None = None
        self.previous_positions: tuple[float, ...] | None = None
        self.previous_target_time_s: float | None = None
        self.connection_id: int | None = None
        self.source_sequence: int | None = None

    @staticmethod
    def _canonical_name(name: str) -> str:
        return name.removeprefix("right_").removeprefix("left_")

    def observe_robot_state(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        now_s: float,
    ) -> bool:
        if not math.isfinite(now_s):
            return False
        by_name = {
            self._canonical_name(name): float(position)
            for name, position in zip(joint_names, positions)
            if math.isfinite(float(position))
        }
        try:
            ordered = tuple(by_name[name] for name in COMMAND_JOINT_NAMES)
        except KeyError:
            return False
        self.robot_positions = ordered
        self.robot_state_time_s = now_s
        return True

    def _stop(self, reason: str) -> GateDecision:
        self.previous_positions = None
        self.previous_target_time_s = None
        return GateDecision(AdapterState.STOP, False, reason)

    def reanchor_command_history(self) -> None:
        """Make the next fresh target compare against the latest robot state."""
        self.previous_positions = None
        self.previous_target_time_s = None

    def assess(
        self,
        result: IKResult,
        now_s: float,
        *,
        command_source_conflict: bool = False,
    ) -> GateDecision:
        if command_source_conflict:
            return self._stop("command_source_conflict")
        if self.robot_positions is None or self.robot_state_time_s is None:
            return GateDecision(AdapterState.WAIT_ROBOT, False, "robot_state_missing")
        if now_s - self.robot_state_time_s > self.robot_state_timeout_s:
            return self._stop("robot_state_stale")
        if now_s < result.target_timestamp_s or now_s - result.target_timestamp_s > self.input_timeout_s:
            return self._stop("input_stale")
        if now_s < result.result_timestamp_s or now_s - result.result_timestamp_s > self.ik_timeout_s:
            return self._stop("ik_result_stale")
        if tuple(result.joint_names) != COMMAND_JOINT_NAMES:
            return self._stop("joint_order_mismatch")
        if self.connection_id is not None and result.source_connection_id != self.connection_id:
            self.connection_id = result.source_connection_id
            self.source_sequence = result.source_sequence
            self.previous_positions = None
            self.previous_target_time_s = None
            return GateDecision(AdapterState.HOLD, False, "connection_changed_reanchor_required")
        if (
            self.source_sequence is not None
            and result.source_connection_id == self.connection_id
            and result.source_sequence <= self.source_sequence
        ):
            return self._stop("source_sequence_not_increasing")
        self.connection_id = result.source_connection_id
        self.source_sequence = result.source_sequence
        if not result.success:
            return GateDecision(AdapterState.HOLD, False, f"ik_failure:{result.failure_reason}")
        if result.control_state != "FOLLOWING":
            return GateDecision(AdapterState.HOLD, False, f"control_state:{result.control_state}")
        if not result.joint_limits_ok:
            return self._stop("ik_joint_limits_failed")
        if not result.motion_limits_ok:
            return self._stop("ik_motion_limits_failed")
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(result.positions, POSITION_LOWER, POSITION_UPPER)
        ):
            return self._stop("joint_position_limit")

        reference = self.previous_positions or self.robot_positions
        delta = tuple(abs(value - old) for value, old in zip(result.positions, reference))
        if max(delta) > self.max_joint_step_rad:
            return self._stop("joint_step_limit")
        if self.previous_target_time_s is not None:
            elapsed = result.target_timestamp_s - self.previous_target_time_s
            if elapsed <= 0.0:
                return self._stop("target_time_not_increasing")
            if any(
                step / elapsed > limit * self.velocity_scale
                for step, limit in zip(delta, MAX_VELOCITY)
            ):
                return self._stop("joint_velocity_limit")

        self.previous_positions = tuple(result.positions)
        self.previous_target_time_s = result.target_timestamp_s
        return GateDecision(AdapterState.RUNNING, True, "accepted", tuple(result.positions))
