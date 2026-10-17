"""Pure geometry and limit checks for robot-disconnected Quest previews."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .calibration import Quaternion, RigidPose


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    """Provisional preview limits, not validated FR3 controller limits."""

    stale_timeout_s: float = 0.2
    max_source_translation_step_m: float = 0.15
    max_source_rotation_step_rad: float = 0.75
    max_target_linear_speed_m_s: float = 3.0
    max_target_angular_speed_rad_s: float = 30.0
    workspace_radius_m: float = 0.75
    workspace_rotation_rad: float = math.pi

    def __post_init__(self) -> None:
        for name in (
            "stale_timeout_s",
            "max_source_translation_step_m",
            "max_source_rotation_step_rad",
            "max_target_linear_speed_m_s",
            "max_target_angular_speed_rad_s",
            "workspace_radius_m",
            "workspace_rotation_rad",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")


def position_distance(first: RigidPose, second: RigidPose) -> float:
    return math.dist(first.position, second.position)


def quaternion_angle(first: Quaternion, second: Quaternion) -> float:
    """Return the shortest rotation angle between normalized xyzw quaternions."""

    dot = abs(sum(left * right for left, right in zip(first, second, strict=True)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def align_quaternion_hemisphere(pose: RigidPose, reference: RigidPose) -> RigidPose:
    """Choose the equivalent quaternion sign closest to the previous target."""

    dot = sum(
        current * previous
        for current, previous in zip(pose.quaternion_xyzw, reference.quaternion_xyzw, strict=True)
    )
    if dot >= 0.0:
        return pose
    return RigidPose(pose.position, tuple(-value for value in pose.quaternion_xyzw))


def source_discontinuity(
    previous: RigidPose,
    current: RigidPose,
    limits: SafetyLimits,
) -> str | None:
    if position_distance(previous, current) > limits.max_source_translation_step_m:
        return "source_translation_jump"
    if quaternion_angle(previous.quaternion_xyzw, current.quaternion_xyzw) > limits.max_source_rotation_step_rad:
        return "source_rotation_jump"
    return None


def target_violation(
    *,
    workspace_anchor: RigidPose,
    previous_target: RigidPose,
    current_target: RigidPose,
    elapsed_s: float,
    limits: SafetyLimits,
) -> str | None:
    if not math.isfinite(elapsed_s) or elapsed_s <= 0.0:
        return "source_time_not_increasing"
    if position_distance(workspace_anchor, current_target) > limits.workspace_radius_m:
        return "workspace_translation_limit"
    if (
        quaternion_angle(workspace_anchor.quaternion_xyzw, current_target.quaternion_xyzw)
        > limits.workspace_rotation_rad
    ):
        return "workspace_rotation_limit"
    linear_speed = position_distance(previous_target, current_target) / elapsed_s
    if linear_speed > limits.max_target_linear_speed_m_s:
        return "target_linear_speed_limit"
    angular_speed = (
        quaternion_angle(previous_target.quaternion_xyzw, current_target.quaternion_xyzw)
        / elapsed_s
    )
    if angular_speed > limits.max_target_angular_speed_rad_s:
        return "target_angular_speed_limit"
    return None
