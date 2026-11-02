"""A fixed startup origin shared by the headset and both controllers."""

from __future__ import annotations

import math

from .calibration import RigidPose, compose, inverse

POSE_REFERENCES = ("standing", "headset-start-position", "headset-start")
IDENTITY = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def pose_dict(pose: RigidPose) -> dict[str, list[float]]:
    return {"position_m": list(pose.position),
            "quaternion_xyzw": list(pose.quaternion_xyzw)}


class TrackingStartReference:
    """Capture once after a stable interval; never follow later head motion.

    Position-only keeps the existing standing/world axis alignment used by
    robot mount calibration. Full-pose mode deliberately rotates those axes.
    """

    def __init__(self, mode: str, *, stable_s: float = 0.5) -> None:
        if mode not in POSE_REFERENCES:
            raise ValueError("unknown pose reference")
        if not math.isfinite(stable_s) or stable_s < 0.0:
            raise ValueError("reference stable interval must be finite and nonnegative")
        self.mode = mode
        self.stable_s = stable_s
        self.headset_start: RigidPose | None = None
        self.captured_monotonic_ns: int | None = None
        self._candidate: RigidPose | None = None
        self._candidate_ns: int | None = None
        self._reference_from_standing = IDENTITY

    @property
    def ready(self) -> bool:
        return self.mode == "standing" or self.headset_start is not None

    @property
    def reference_from_standing(self) -> RigidPose:
        if not self.ready:
            raise RuntimeError("headset startup reference has not been captured")
        return self._reference_from_standing

    def observe(self, headset_pose: RigidPose | None, monotonic_ns: int) -> bool:
        """Return True only on capture, requiring a stable 2 cm / 5 deg window."""
        if self.ready:
            return False
        if headset_pose is None:
            self._candidate = None
            self._candidate_ns = None
            return False
        if self._candidate is not None:
            distance = math.dist(headset_pose.position, self._candidate.position)
            dot = min(1.0, abs(sum(a * b for a, b in zip(
                headset_pose.quaternion_xyzw, self._candidate.quaternion_xyzw))))
            angle = 2.0 * math.acos(dot)
            if distance > 0.02 or angle > math.radians(5.0):
                self._candidate = None
        if self._candidate is None:
            self._candidate = headset_pose
            self._candidate_ns = monotonic_ns
        assert self._candidate_ns is not None
        if (monotonic_ns - self._candidate_ns) / 1e9 < self.stable_s:
            return False
        self.headset_start = headset_pose
        self.captured_monotonic_ns = monotonic_ns
        standing_from_reference = (headset_pose if self.mode == "headset-start"
                                   else RigidPose(headset_pose.position, IDENTITY.quaternion_xyzw))
        self._reference_from_standing = inverse(standing_from_reference)
        return True

    def apply(self, standing_pose: RigidPose) -> RigidPose:
        return compose(self.reference_from_standing, standing_pose)
