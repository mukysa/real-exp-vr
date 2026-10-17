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
