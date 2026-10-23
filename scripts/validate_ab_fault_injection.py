#!/usr/bin/env python3
"""No-ROS fault-injection acceptance check for the shared AdapterCore."""

from dataclasses import replace
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from data_collection.fr3_ik.examples import contract_examples
from data_collection.fr3_ik.interfaces import (
    ClockDomain,
    FailureReason,
    JointTarget,
    MeasuredJointState,
    ResultState,
    Timestamp,
)
from data_collection.meta_quest.adapter_core import AdapterCore


Q0 = (0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7)
CLOCK_ID = "ab-fault-injection"


def stamp(seconds: float, clock_id: str = CLOCK_ID) -> Timestamp:
    return Timestamp(seconds, ClockDomain.REPLAY, clock_id)


def following(sequence=1, seconds=1.0, positions=Q0, connection=1, side="right"):
    base = contract_examples()["following"]
    target = replace(base.request.target, frame_id=f"{side}_fr3_link0",
                     source_sequence=sequence, source_connection_id=connection,
                     generated_monotonic_s=seconds)
    request = replace(base.request, target=target, source_stamp=stamp(seconds),
                      target_stamp=stamp(seconds), side=side,
                      tool_frame=f"{side}_fr3_hand_tcp")
    command = JointTarget(replace(base.command.joints, positions_rad=tuple(positions)), stamp(seconds))
    return replace(base, request=request, command=command, last_safe_target=command,
                   computed_stamp=stamp(seconds))


def failed(reason: FailureReason, sequence=1, seconds=1.0):
    base = following(sequence, seconds)
    return replace(base, state=ResultState.FAULT, reason=reason,
                   detail=f"injected {reason.value}", command=None,
                   last_safe_target=JointTarget(base.last_safe_target.joints, stamp(seconds)),
                   position_residual_m=None, rotation_residual_rad=None,
                   checks=replace(base.checks, residual=False))


def measured(seconds=1.0, positions=Q0, valid=True):
    joints = replace(contract_examples()["following"].command.joints,
                     positions_rad=tuple(positions))
    return MeasuredJointState(joints, stamp(seconds), "/right/joint_states", valid)


def run_suite() -> dict:
    records = []

    def record(name, decision=None, *, contract_rejected=False):
        would_publish = False if contract_rejected else decision.would_publish
        reason = "contract_validation" if contract_rejected else decision.reason
        records.append({"case": name, "would_publish": would_publish, "reject_reason": reason})

    record("quest_data_stale", AdapterCore(side="right").assess(
        following(seconds=0.5), measured(), stamp(1.0)))
    record("robot_state_stale", AdapterCore(side="right").assess(
        following(), measured(0.5), stamp(1.0)))
    record("tracking_lost", AdapterCore(side="right").assess(
        failed(FailureReason.INVALID_INPUT), measured(), stamp(1.0)))
    record("quest_disconnected", AdapterCore(side="right").assess(
        following(), measured(), stamp(1.0), quest_connected=False))
    record("target_pose_jump", AdapterCore(side="right").assess(
        failed(FailureReason.STEP_LIMIT), measured(), stamp(1.0)))
    record("ik_residual_exceeded", AdapterCore(side="right").assess(
        failed(FailureReason.RESIDUAL), measured(), stamp(1.0)))
    record("joint_range", AdapterCore(side="right").assess(
        following(positions=Q0[:-1] + (4.0,)), measured(), stamp(1.0)))
    record("gello_conflict", AdapterCore(side="right").assess(
        following(), measured(), stamp(1.0), gello_conflict=True))
    record("wrong_arm", AdapterCore(side="right").assess(
        following(side="left"), measured(), stamp(1.0)))
    record("invalid_measured", AdapterCore(side="right").assess(
        following(), measured(valid=False), stamp(1.0)))

    for name, second in (
        ("sequence_duplicate", following(1, 1.05)),
        ("sequence_backwards", following(1, 1.05)),
        ("sequence_gap", following(5, 1.05)),
    ):
        gate = AdapterCore(side="right")
        first_sequence = 1 if name != "sequence_backwards" else 3
        gate.assess(following(first_sequence), measured(), stamp(1.0))
        record(name, gate.assess(second, measured(1.05), stamp(1.05)))

    gate = AdapterCore(side="right")
    gate.assess(following(), measured(), stamp(1.0))
    record("joint_step", gate.assess(
        following(3, 1.2, (0.2,) + Q0[1:]), measured(1.2), stamp(1.2)))
    gate = AdapterCore(side="right")
    gate.assess(following(), measured(), stamp(1.0))
    record("joint_velocity", gate.assess(
        following(3, 1.05, (0.04,) + Q0[1:]), measured(1.05), stamp(1.05)))

    gate = AdapterCore(side="right")
    gate.assess(following(), measured(), stamp(1.0))
    gate.assess(following(3, 1.05), measured(1.05), stamp(1.05), gello_conflict=True)
    record("fault_latched_normal_input", gate.assess(
        following(5, 1.10), measured(1.10), stamp(1.10)))

    try:
        following(positions=Q0[:-1] + (float("nan"),))
    except ValueError:
        record("nan", contract_rejected=True)
    try:
        following(positions=Q0[:-1] + (float("inf"),))
    except ValueError:
        record("inf", contract_rejected=True)

    failures = [item for item in records if item["would_publish"] or not item["reject_reason"]]
    return {
        "schema": "fr3_ab_fault_injection_v1",
        "case_count": len(records),
        "all_rejected": not failures,
        "failures": failures,
        "cases": records,
        "hardware_command_published": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = run_suite()
    text = json.dumps(result, indent=2, allow_nan=False, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    if not result["all_rejected"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
