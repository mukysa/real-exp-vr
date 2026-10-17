from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.models import TargetPose


def _target(**overrides) -> TargetPose:
    values = {
        "pose": RigidPose((0.4, -0.2, 0.7), (0.0, 0.0, 0.0, 2.0)),
        "frame_id": "right_teleop_control",
        "source_sequence": 12,
        "source_connection_id": 3,
        "generated_monotonic_s": 123.5,
        "state": "FOLLOWING",
        "valid": True,
    }
    values.update(overrides)
    return TargetPose(**values)


def test_target_pose_normalizes_pose_and_is_immutable() -> None:
    target = _target()

    assert target.pose.quaternion_xyzw == (0.0, 0.0, 0.0, 1.0)
    with pytest.raises(FrozenInstanceError):
        target.valid = False


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"frame_id": "  "}, "frame_id"),
        ({"source_sequence": 0}, "source_sequence"),
        ({"source_connection_id": -1}, "source_connection_id"),
        ({"generated_monotonic_s": float("nan")}, "generated_monotonic_s"),
        ({"state": ""}, "state"),
        ({"hold_reason": "stale"}, "valid target"),
    ],
)
def test_target_pose_rejects_invalid_metadata(overrides, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _target(**overrides)


def test_invalid_target_can_explain_why_it_is_held() -> None:
    target = _target(valid=False, state="HOLD", hold_reason="stale")

    assert target.hold_reason == "stale"
