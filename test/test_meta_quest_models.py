from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.models import IKResult, TargetPose


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


def _ik_result(**overrides) -> IKResult:
    values = {
        "joint_names": tuple(f"fr3_joint{index}" for index in range(1, 8)),
        "positions": (0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7),
        "success": True,
        "failure_reason": None,
        "solve_time_s": 0.005,
        "position_residual_m": 1e-6,
        "rotation_residual_rad": 2e-6,
        "joint_limits_ok": True,
        "motion_limits_ok": True,
        "target_timestamp_s": 10.0,
        "result_timestamp_s": 10.005,
        "source_connection_id": 2,
        "source_sequence": 3,
        "control_state": "FOLLOWING",
    }
    values.update(overrides)
    return IKResult(**values)


def test_ik_result_freezes_stage3_stage4_contract() -> None:
    result = _ik_result()

    assert result.success is True
    assert result.joint_names[0] == "fr3_joint1"


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"joint_names": ("fr3_joint1",)}, "joint_names"),
        ({"positions": (0.0,) * 6}, "positions"),
        ({"positions": (0.0,) * 6 + (float("nan"),)}, "positions"),
        ({"source_sequence": 0}, "source_sequence"),
        ({"result_timestamp_s": 9.0}, "must not precede"),
        ({"failure_reason": "bad"}, "successful"),
        ({"joint_limits_ok": False}, "must pass"),
        ({"success": False, "failure_reason": None}, "failure reason"),
    ],
)
def test_ik_result_rejects_ambiguous_or_unsafe_contract(overrides, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _ik_result(**overrides)
