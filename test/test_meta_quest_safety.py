from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose
from data_collection.meta_quest.replay import load_recording
from data_collection.meta_quest.safety import (
    SafetyLimits,
    align_quaternion_hemisphere,
    quaternion_angle,
)
from data_collection.meta_quest.safety_replay import replay_safety
from data_collection.meta_quest.state_machine import (
    TeleoperationState,
    TeleoperationStateMachine,
)
from data_collection.meta_quest.target_preview import sha256_file


REAL_CAPTURE_SHA256 = "2d84d7eb95cebb133998711684fe98038b862a47b72939a10eadf9beadbf7a66"
TARGET_ANCHOR = RigidPose((0.4, -0.2, 0.7), (0.0, 0.0, 0.0, 1.0))


def _received(
    sequence: int,
    timestamp_s: float,
    *,
    connection_id: int = 1,
    position=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    clutch=False,
    tracked=True,
) -> ReceivedPose:
    return ReceivedPose(
        PoseSample(
            side="right",
            source="controller",
            position=position,
            quaternion_xyzw=quaternion,
            tracked=tracked,
            grasp=0.0,
            clutch=clutch,
            record=False,
            layout="controller_compact_controls_v1",
        ),
        timestamp_s,
        sequence,
        connection_id,
    )


def _ready_machine(**kwargs) -> tuple[TeleoperationStateMachine, int, float]:
    machine = TeleoperationStateMachine(TARGET_ANCHOR, min_ready_samples=2, **kwargs)
    assert machine.update(_received(1, 1.00, clutch=False)).state == "WAIT_INPUT"
    assert machine.update(_received(2, 1.02, clutch=False)).state == "READY"
    return machine, 3, 1.04


def test_toggle_follow_hold_and_reanchor_without_jump() -> None:
    machine, sequence, timestamp = _ready_machine()
    engaged = machine.update(_received(sequence, timestamp, clutch=True))
    moved = machine.update(
        _received(sequence + 1, timestamp + 0.02, clutch=True, position=(0.02, 0.0, 0.0))
    )
    released = machine.update(
        _received(sequence + 2, timestamp + 0.08, clutch=False, position=(0.03, 0.0, 0.0))
    )
    held = machine.update(
        _received(sequence + 3, timestamp + 0.18, clutch=True, position=(0.04, 0.0, 0.0))
    )
    machine.update(
        _received(sequence + 4, timestamp + 0.20, clutch=False, position=(0.20, 0.0, 0.0))
    )
    resumed = machine.update(
        _received(sequence + 5, timestamp + 0.38, clutch=True, position=(0.20, 0.0, 0.0))
    )

    assert engaged.valid is True
    assert engaged.pose == TARGET_ANCHOR
    assert moved.valid is True
    assert released.valid is True
    assert held.valid is False and held.state == "MANUAL_HOLD"
    assert resumed.valid is True and resumed.state == "FOLLOWING"
    assert resumed.pose == held.pose
    assert machine.anchor_sequences == [3, 8]


def test_button_held_at_start_requires_release_before_engage() -> None:
    machine = TeleoperationStateMachine(TARGET_ANCHOR, min_ready_samples=2)

    first = machine.update(_received(1, 1.00, clutch=True))
    ready = machine.update(_received(2, 1.02, clutch=True))
    still_ready = machine.update(_received(3, 1.04, clutch=True))
    machine.update(_received(4, 1.06, clutch=False))
    engaged = machine.update(_received(5, 1.25, clutch=True))

    assert first.valid is False
    assert ready.state == "READY"
    assert still_ready.valid is False
    assert engaged.valid is True


def test_stale_tick_fault_requires_fresh_samples_release_and_new_edge() -> None:
    machine, sequence, timestamp = _ready_machine()
    machine.update(_received(sequence, timestamp, clutch=True))

    stale = machine.tick(timestamp + machine.limits.stale_timeout_s + 0.01)
    assert stale is not None
    assert stale.state == "FAULT_HOLD"
    assert stale.hold_reason == "stale_timeout"

    first_fresh = machine.update(_received(4, timestamp + 0.23, clutch=False))
    machine.update(_received(5, timestamp + 0.25, clutch=False))
    recovered = machine.update(_received(6, timestamp + 0.27, clutch=True))
    assert first_fresh.valid is False
    assert recovered.valid is True
    assert recovered.pose == stale.pose


def test_reconnect_holds_and_reanchors_to_last_safe_target() -> None:
    machine, sequence, timestamp = _ready_machine()
    machine.update(_received(sequence, timestamp, clutch=True))
    last_safe = machine.update(
        _received(4, timestamp + 0.02, clutch=True, position=(0.02, 0.0, 0.0))
    )

    disconnected = machine.update(
        _received(5, timestamp + 0.04, connection_id=2, clutch=True, position=(5.0, 5.0, 5.0))
    )
    machine.update(
        _received(6, timestamp + 0.06, connection_id=2, clutch=False, position=(1.0, 1.0, 1.0))
    )
    recovered = machine.update(
        _received(7, timestamp + 0.25, connection_id=2, clutch=True, position=(1.0, 1.0, 1.0))
    )

    assert disconnected.hold_reason == "connection_changed"
    assert disconnected.pose == last_safe.pose
    assert recovered.valid is True
    assert recovered.pose == last_safe.pose


@pytest.mark.parametrize(
    ("limits", "position", "timestamp", "reason"),
    [
        (SafetyLimits(max_source_translation_step_m=0.05), (0.20, 0.0, 0.0), 1.06, "source_translation_jump"),
        (SafetyLimits(max_target_linear_speed_m_s=0.5), (0.04, 0.0, 0.0), 1.06, "target_linear_speed_limit"),
        (SafetyLimits(workspace_radius_m=0.02), (0.04, 0.0, 0.0), 1.20, "workspace_translation_limit"),
    ],
)
def test_motion_faults_hold_last_safe_target(limits, position, timestamp, reason) -> None:
    machine, sequence, start = _ready_machine(limits=limits)
    engaged = machine.update(_received(sequence, start, clutch=True))
    fault = machine.update(_received(sequence + 1, timestamp, clutch=True, position=position))

    assert fault.valid is False
    assert fault.state == "FAULT_HOLD"
    assert fault.hold_reason == reason
    assert fault.pose == engaged.pose


def test_tracking_invalid_is_not_treated_as_manual_hold() -> None:
    machine, sequence, timestamp = _ready_machine()
    machine.update(_received(sequence, timestamp, clutch=True))

    target = machine.update(_received(sequence + 1, timestamp + 0.02, tracked=False))

    assert target.state == "FAULT_HOLD"
    assert target.hold_reason == "tracking_invalid"


def test_quaternion_hemisphere_alignment_removes_sign_flip() -> None:
    reference = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    equivalent = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, -1.0))

    aligned = align_quaternion_hemisphere(equivalent, reference)

    assert aligned.quaternion_xyzw == reference.quaternion_xyzw
    assert quaternion_angle(aligned.quaternion_xyzw, reference.quaternion_xyzw) == 0.0


def test_fault_injection_uses_copies_without_mutating_capture() -> None:
    path = Path(__file__).resolve().parents[1] / "recordings/quest_raw/stage2-sixdof-safety3.jsonl"
    if not path.exists():
        pytest.skip("operator-owned stage2 capture is not installed")
    before = sha256_file(path)
    assert before == REAL_CAPTURE_SHA256
    recording = load_recording(path)
    right = [pose for pose in recording.poses if pose.sample.side == "right"]
    injected = list(right[:20])
    injected[10] = replace(
        injected[10],
        connection_id=injected[9].connection_id + 100,
    )

    machine = TeleoperationStateMachine(TARGET_ANCHOR, min_ready_samples=2)
    targets = [machine.update(pose) for pose in injected]

    assert any(
        target is not None and target.hold_reason == "connection_changed"
        for target in targets
    )
    assert sha256_file(path) == before


def test_real_capture_replays_expected_holds_and_remains_unchanged() -> None:
    path = Path(__file__).resolve().parents[1] / "recordings/quest_raw/stage2-sixdof-safety3.jsonl"
    if not path.exists():
        pytest.skip("operator-owned stage2 capture is not installed")
    before = sha256_file(path)
    assert before == REAL_CAPTURE_SHA256
    recording = load_recording(path)
    machine = TeleoperationStateMachine(
        RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        side="right",
    )

    result = replay_safety(recording, machine)

    assert result.selected_pose_records == 3902
    assert result.ignored_pose_records == 43
    assert machine.anchor_sequences == [1648, 2362, 3050]
    fault_reasons = {
        target.hold_reason
        for target in result.targets
        if target.state == TeleoperationState.FAULT_HOLD
    }
    assert {"stale_gap", "connection_changed"} <= fault_reasons
    assert any(target.state == TeleoperationState.FOLLOWING for target in result.targets)
    assert any(target.state == TeleoperationState.MANUAL_HOLD for target in result.targets)
    assert sha256_file(path) == before
