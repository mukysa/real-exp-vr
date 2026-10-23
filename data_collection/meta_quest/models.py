"""Stable data contracts shared by the Quest stage-2 processing layers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .calibration import RigidPose


@dataclass(frozen=True, slots=True)
class TargetPose:
    """One processed end-effector target in a declared control frame.

    Stage 2 only previews this value.  Publishing it to ROS or commanding a
    robot belongs to a later, explicitly enabled backend.
    """

    pose: RigidPose
    frame_id: str
    source_sequence: int
    source_connection_id: int
    generated_monotonic_s: float
    state: str
    valid: bool
    hold_reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.pose, RigidPose):
            raise TypeError("pose must be a RigidPose")
        if not self.frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if self.source_sequence < 1:
            raise ValueError("source_sequence must be positive")
        if self.source_connection_id < 0:
            raise ValueError("source_connection_id must not be negative")
        if not math.isfinite(self.generated_monotonic_s):
            raise ValueError("generated_monotonic_s must be finite")
        if not self.state.strip():
            raise ValueError("state must not be empty")
        if self.valid and self.hold_reason is not None:
            raise ValueError("a valid target cannot have a hold reason")


@dataclass(frozen=True, slots=True)
class IKResult:
    """Checked stage-3 result consumed by the stage-4 ROS adapter.

    A failed result still carries the previous safe seven-joint target.  This
    keeps failure handling explicit and prevents an empty array from being
    mistaken for a valid command.
    """

    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    success: bool
    failure_reason: str | None
    solve_time_s: float
    position_residual_m: float
    rotation_residual_rad: float
    joint_limits_ok: bool
    motion_limits_ok: bool
    target_timestamp_s: float
    result_timestamp_s: float
    source_connection_id: int
    source_sequence: int
    control_state: str

    def __post_init__(self) -> None:
        if len(self.joint_names) != 7 or len(set(self.joint_names)) != 7:
            raise ValueError("joint_names must contain seven unique names")
        if len(self.positions) != 7 or not all(math.isfinite(value) for value in self.positions):
            raise ValueError("positions must contain seven finite values")
        for field_name in (
            "solve_time_s",
            "position_residual_m",
            "rotation_residual_rad",
            "target_timestamp_s",
            "result_timestamp_s",
        ):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and non-negative")
        if self.result_timestamp_s < self.target_timestamp_s:
            raise ValueError("result_timestamp_s must not precede target_timestamp_s")
        if self.source_connection_id < 0:
            raise ValueError("source_connection_id must not be negative")
        if self.source_sequence < 1:
            raise ValueError("source_sequence must be positive")
        if not self.control_state.strip():
            raise ValueError("control_state must not be empty")
        if self.success:
            if self.failure_reason is not None:
                raise ValueError("a successful IK result cannot have a failure reason")
            if not self.joint_limits_ok or not self.motion_limits_ok:
                raise ValueError("a successful IK result must pass joint and motion limits")
        elif not self.failure_reason:
            raise ValueError("a failed IK result must include a failure reason")
