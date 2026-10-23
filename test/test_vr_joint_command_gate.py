from __future__ import annotations

import pytest

from data_collection.meta_quest.joint_command_gate import (
    AdapterState,
    COMMAND_JOINT_NAMES,
    JointCommandGate,
)
from data_collection.meta_quest.models import IKResult


Q0 = (0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7)


def result(sequence: int = 1, timestamp: float = 1.0, positions=Q0, **overrides) -> IKResult:
    values = {
        "joint_names": COMMAND_JOINT_NAMES,
        "positions": tuple(positions),
        "success": True,
        "failure_reason": None,
        "solve_time_s": 0.001,
        "position_residual_m": 0.0,
        "rotation_residual_rad": 0.0,
        "joint_limits_ok": True,
        "motion_limits_ok": True,
        "target_timestamp_s": timestamp,
        "result_timestamp_s": timestamp,
        "source_connection_id": 1,
        "source_sequence": sequence,
        "control_state": "FOLLOWING",
    }
    values.update(overrides)
    return IKResult(**values)


def ready_gate() -> JointCommandGate:
    gate = JointCommandGate(max_joint_step_rad=0.1)
    assert gate.observe_robot_state(
        tuple(f"right_{name}" for name in COMMAND_JOINT_NAMES), Q0, 1.0
    )
    return gate


def test_valid_target_is_published_in_frozen_joint_order() -> None:
    decision = ready_gate().assess(result(), 1.01)

    assert decision.state == AdapterState.RUNNING
    assert decision.publish is True
    assert decision.positions == Q0


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"now_s": 1.30}, "robot_state_stale"),
        ({"target_timestamp_s": 0.5, "result_timestamp_s": 0.5}, "input_stale"),
        ({"joint_names": tuple(reversed(COMMAND_JOINT_NAMES))}, "joint_order_mismatch"),
        ({"positions": Q0[:-1] + (4.0,)}, "joint_position_limit"),
        ({"positions": (0.2,) + Q0[1:]}, "joint_step_limit"),
    ],
)
def test_fault_injection_rejects_with_explicit_reason(change, reason: str) -> None:
    gate = ready_gate()
    result_changes = dict(change)
    now_s = result_changes.pop("now_s", 1.01)
    decision = gate.assess(result(**result_changes), now_s)

    assert decision.state == AdapterState.STOP
    assert decision.publish is False
    assert decision.reason == reason


def test_ik_failure_holds_without_refreshing_controller_command() -> None:
    gate = ready_gate()
    failed = result(success=False, failure_reason="unreachable", joint_limits_ok=False)

    decision = gate.assess(failed, 1.01)

    assert decision.state == AdapterState.HOLD
    assert decision.publish is False
    assert decision.reason == "ik_failure:unreachable"


def test_command_source_conflict_stops_and_clears_previous_target() -> None:
    gate = ready_gate()
    assert gate.assess(result(), 1.01).publish

    stopped = gate.assess(result(2, 1.05), 1.05, command_source_conflict=True)

    assert stopped.reason == "command_source_conflict"
    assert gate.previous_positions is None


def test_connection_change_requires_one_reanchor_cycle() -> None:
    gate = ready_gate()
    assert gate.assess(result(), 1.01).publish
    gate.observe_robot_state(COMMAND_JOINT_NAMES, Q0, 1.04)

    changed = gate.assess(result(2, 1.04, source_connection_id=2), 1.04)
    resumed = gate.assess(result(3, 1.08, source_connection_id=2), 1.08)

    assert changed.reason == "connection_changed_reanchor_required"
    assert changed.publish is False
    assert resumed.publish is True


def test_waits_for_a_complete_robot_state() -> None:
    gate = JointCommandGate()

    decision = gate.assess(result(), 1.01)

    assert decision.state == AdapterState.WAIT_ROBOT
    assert decision.reason == "robot_state_missing"


def test_rejects_stale_ik_result_separately_from_stale_input() -> None:
    gate = ready_gate()
    old_result = result(target_timestamp_s=1.0, result_timestamp_s=1.0)

    decision = gate.assess(old_result, 1.20)

    assert decision.reason == "ik_result_stale"


def test_manual_hold_does_not_republish_the_previous_target() -> None:
    gate = ready_gate()
    held = result(control_state="MANUAL_HOLD")

    decision = gate.assess(held, 1.01)

    assert decision.state == AdapterState.HOLD
    assert decision.publish is False
    assert decision.reason == "control_state:MANUAL_HOLD"


def test_rejects_non_increasing_sequence() -> None:
    gate = ready_gate()
    assert gate.assess(result(), 1.01).publish
    gate.observe_robot_state(COMMAND_JOINT_NAMES, Q0, 1.04)

    decision = gate.assess(result(sequence=1, timestamp=1.04), 1.04)

    assert decision.reason == "source_sequence_not_increasing"


def test_rejects_velocity_limit_even_when_each_step_is_small() -> None:
    gate = ready_gate()
    assert gate.assess(result(), 1.01).publish
    gate.observe_robot_state(COMMAND_JOINT_NAMES, Q0, 1.001)
    fast = (0.01,) + Q0[1:]

    decision = gate.assess(result(2, 1.001, positions=fast), 1.001)

    assert decision.reason == "joint_velocity_limit"


def test_reanchor_compares_recovery_target_with_latest_robot_state() -> None:
    gate = ready_gate()
    assert gate.assess(result(), 1.01).publish
    moved = (0.01,) + Q0[1:]
    gate.observe_robot_state(COMMAND_JOINT_NAMES, moved, 1.05)

    gate.reanchor_command_history()
    recovered = gate.assess(result(2, 1.05, positions=moved), 1.05)

    assert recovered.publish is True
    assert recovered.positions == moved
