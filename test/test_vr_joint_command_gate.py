from __future__ import annotations

from dataclasses import replace

import pytest

from data_collection.fr3_ik.examples import contract_examples
from data_collection.fr3_ik.interfaces import (
    ClockDomain,
    JointTarget,
    MeasuredJointState,
    Timestamp,
)
from data_collection.meta_quest.adapter_core import AdapterCore, AdapterState


Q0 = (0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7)
CLOCK_ID = "offline-gate-test"


def stamp(seconds: float) -> Timestamp:
    return Timestamp(seconds, ClockDomain.REPLAY, CLOCK_ID)


def following(sequence: int = 1, timestamp: float = 1.0, positions=Q0,
              connection_id: int = 1, side: str = "right"):
    base = contract_examples()["following"]
    target = replace(
        base.request.target,
        frame_id=f"{side}_fr3_link0",
        source_sequence=sequence,
        source_connection_id=connection_id,
        generated_monotonic_s=timestamp,
    )
    request = replace(
        base.request,
        target=target,
        source_stamp=stamp(timestamp),
        target_stamp=stamp(timestamp),
        tool_frame=f"{side}_fr3_hand_tcp",
        side=side,
    )
    command = JointTarget(replace(base.command.joints, positions_rad=tuple(positions)), stamp(timestamp))
    return replace(base, request=request, command=command, last_safe_target=command,
                   computed_stamp=stamp(timestamp))


def measured(timestamp: float = 1.0, positions=Q0) -> MeasuredJointState:
    joints = replace(contract_examples()["following"].command.joints,
                     positions_rad=tuple(positions))
    return MeasuredJointState(joints, stamp(timestamp), "/right/joint_states", True)


def ready_gate(**kwargs) -> AdapterCore:
    return AdapterCore(side="right", **kwargs)


def test_valid_target_is_accepted_in_frozen_joint_order() -> None:
    decision = ready_gate().assess(following(), measured(), stamp(1.01))
    assert decision.state == AdapterState.RUNNING
    assert decision.would_publish is True
    assert decision.positions == Q0
    assert decision.reject_reason is None


@pytest.mark.parametrize(
    ("case", "reason"),
    [
        ("robot_stale", "robot_state_stale"),
        ("input_stale", "input_stale"),
        ("side", "side_mismatch"),
        ("limit", "joint_position_limit"),
        ("step", "initial_target_must_equal_measured"),
        ("quest", "quest_disconnected"),
        ("gello", "command_source_conflict"),
    ],
)
def test_faults_reject_with_explicit_reason(case: str, reason: str) -> None:
    result = following()
    state = measured()
    now = stamp(1.01)
    kwargs = {}
    if case == "robot_stale":
        state = measured(0.5)
    elif case == "input_stale":
        result = following(timestamp=0.5)
    elif case == "side":
        result = following(side="left")
    elif case == "limit":
        result = following(positions=Q0[:-1] + (4.0,))
    elif case == "step":
        result = following(positions=(0.01,) + Q0[1:])
    elif case == "quest":
        kwargs["quest_connected"] = False
    elif case == "gello":
        kwargs["gello_conflict"] = True
    decision = ready_gate().assess(result, state, now, **kwargs)
    assert decision.would_publish is False
    assert decision.reject_reason == reason


def test_hold_and_fault_results_never_publish() -> None:
    for name, expected_state in (("normal_hold", AdapterState.HOLD),
                                 ("unreachable", AdapterState.STOP),
                                 ("reconnect", AdapterState.STOP)):
        result = contract_examples()[name]
        same_clock_request = replace(result.request, source_stamp=result.request.target_stamp)
        result = replace(result, request=same_clock_request,
                         computed_stamp=result.request.target_stamp)
        result_stamp = result.request.target_stamp
        robot_state = replace(measured(result_stamp.seconds), stamp=result_stamp)
        decision = ready_gate().assess(result, robot_state, result_stamp)
        assert decision.state == expected_state
        assert decision.would_publish is False


def test_connection_change_requires_explicit_reanchor() -> None:
    gate = ready_gate()
    assert gate.assess(following(), measured(), stamp(1.01)).would_publish
    changed = gate.assess(following(3, 1.05, connection_id=2), measured(1.05), stamp(1.05))
    assert changed.reject_reason == "connection_changed_reanchor_required"
    gate.reset_for_reanchor()
    resumed = gate.assess(following(5, 1.10, connection_id=2), measured(1.10), stamp(1.10))
    assert resumed.would_publish


def test_rejects_duplicate_gap_and_clock_mismatch() -> None:
    for second, expected in ((following(1, 1.05), "source_sequence_not_increasing"),
                             (following(5, 1.05), "source_sequence_gap")):
        gate = ready_gate()
        assert gate.assess(following(), measured(), stamp(1.01)).would_publish
        decision = gate.assess(second, measured(1.05), stamp(1.05))
        assert decision.reject_reason == expected
    gate = ready_gate()
    wrong_clock = Timestamp(1.01, ClockDomain.REPLAY, "another-session")
    assert gate.assess(following(), measured(), wrong_clock).reject_reason == "clock_domain_mismatch"


def test_velocity_limit_and_fault_latch_require_reanchor() -> None:
    gate = ready_gate()
    assert gate.assess(following(), measured(), stamp(1.01)).would_publish
    fast = (0.04,) + Q0[1:]
    rejected = gate.assess(following(3, 1.05, fast), measured(1.05), stamp(1.05))
    assert rejected.reject_reason == "joint_velocity_limit"
    still_rejected = gate.assess(following(5, 1.10), measured(1.10), stamp(1.10))
    assert still_rejected.reject_reason == "fault_latched_reanchor_required"
    gate.reset_for_reanchor()
    assert gate.assess(following(7, 1.15), measured(1.15), stamp(1.15)).would_publish
