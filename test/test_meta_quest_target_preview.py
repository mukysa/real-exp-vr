from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose
from data_collection.meta_quest.replay import PoseRecording, load_recording
from data_collection.meta_quest.target_preview import (
    TargetPreviewGenerator,
    generate_targets,
    sha256_file,
)


REAL_CAPTURE_SHA256 = "2d84d7eb95cebb133998711684fe98038b862a47b72939a10eadf9beadbf7a66"


def _received(
    sequence: int,
    *,
    connection_id: int = 1,
    position=(0.0, 0.0, 0.0),
    clutch=False,
) -> ReceivedPose:
    return ReceivedPose(
        PoseSample(
            side="right",
            source="controller",
            position=position,
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            tracked=True,
            grasp=0.0,
            clutch=clutch,
            record=False,
            layout="controller_compact_controls_v1",
        ),
        received_monotonic_s=float(sequence),
        sequence=sequence,
        connection_id=connection_id,
    )


def _recording(*poses: ReceivedPose) -> PoseRecording:
    return PoseRecording(
        metadata={"schema_version": 1},
        poses=poses,
        summary={"subscriber_dropped_records": 0},
    )


def test_first_pose_anchor_has_exactly_zero_target_jump() -> None:
    target_anchor = RigidPose((0.4, -0.2, 0.7), (0.0, 0.0, 0.0, 1.0))
    recording = _recording(
        _received(1, position=(0.2, 1.0, -0.4)),
        _received(2, position=(0.3, 1.0, -0.4)),
    )

    result = generate_targets(recording, TargetPreviewGenerator(target_anchor))

    assert result.anchor_sequences == (1,)
    assert result.targets[0].valid is True
    assert result.targets[0].pose == target_anchor
    # Unity +x maps to robot -y.  Identity anchors keep that relative direction.
    assert result.targets[1].pose.position == pytest.approx((0.4, -0.3, 0.7))


def test_button_anchor_requires_release_then_rising_edge() -> None:
    target_anchor = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    recording = _recording(
        _received(1, clutch=True),
        _received(2, clutch=False),
        _received(3, clutch=True),
    )

    result = generate_targets(
        recording,
        TargetPreviewGenerator(target_anchor, anchor="clutch"),
    )

    assert [target.valid for target in result.targets] == [False, False, True]
    assert result.anchor_sequences == (3,)
    assert result.targets[2].pose == target_anchor


def test_repeated_button_anchor_preserves_last_target_without_jump() -> None:
    target_anchor = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    recording = _recording(
        _received(1, position=(0.0, 0.0, 0.0), clutch=False),
        _received(2, position=(0.0, 0.0, 0.0), clutch=True),
        _received(3, position=(0.1, 0.0, 0.0), clutch=False),
        _received(4, position=(0.2, 0.0, 0.0), clutch=True),
    )

    result = generate_targets(
        recording,
        TargetPreviewGenerator(target_anchor, anchor="clutch"),
    )

    assert result.anchor_sequences == (2, 4)
    assert result.targets[3].pose == result.targets[2].pose


def test_connection_change_holds_instead_of_continuing_old_mapping() -> None:
    target_anchor = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    recording = _recording(
        _received(1),
        _received(2, position=(0.0, 0.0, 0.1)),
        _received(3, connection_id=2, position=(9.0, 9.0, 9.0)),
    )

    result = generate_targets(recording, TargetPreviewGenerator(target_anchor))

    assert [target.valid for target in result.targets] == [True, True, False]
    assert result.targets[-1].state == "HOLD"
    assert result.targets[-1].hold_reason == "reconnect_requires_button_anchor"
    assert result.targets[-1].pose == result.targets[-2].pose


def test_real_capture_is_immutable_lossless_test_dataset() -> None:
    path = Path(__file__).resolve().parents[1] / "recordings/quest_raw/stage2-sixdof-safety3.jsonl"
    if not path.exists():
        pytest.skip("operator-owned stage2 capture is not installed")
    before = sha256_file(path)
    assert before == REAL_CAPTURE_SHA256

    recording = load_recording(path)
    result = generate_targets(
        recording,
        TargetPreviewGenerator(
            RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            side="right",
        ),
    )

    assert len(recording.poses) == 3945
    assert result.selected_pose_records == 3902
    assert result.ignored_pose_records == 43
    assert result.valid_targets == 3476
    assert result.held_targets == 426
    assert result.anchor_sequences == (1,)
    assert sha256_file(path) == before
    assert hashlib.sha256(path.read_bytes()).hexdigest() == REAL_CAPTURE_SHA256
