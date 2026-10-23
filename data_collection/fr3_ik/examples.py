"""Synthetic contract examples, NOT solved trajectories or robot commands."""

from dataclasses import asdict, replace
import json

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.models import TargetPose
from .interfaces import (
    BASE_FRAME, ClockDomain, ConstraintChecks, FailureReason, IKRequest,
    IKResult, JointTarget, JointVector, ResultState, Timestamp,
)


def contract_examples() -> dict[str, IKResult]:
    q = JointVector((0, -0.4, 0, -1.8, 0, 1.5, 0.7))
    previous = JointTarget(q, Timestamp(0, ClockDomain.REPLAY, "synthetic-contract-v1"))
    stamp = Timestamp(0.02, ClockDomain.REPLAY, "synthetic-contract-v1")
    target = TargetPose(RigidPose((0.425887, 0, 0.602186), (1, 0, 0, 0)),
                        BASE_FRAME, 2, 1, stamp.seconds, "FOLLOWING", True)
    request = IKRequest(target, Timestamp(100, ClockDomain.HOST_MONOTONIC, "example-host-session"), stamp)
    command = JointTarget(q, stamp)
    following = IKResult(request, ResultState.FOLLOWING, FailureReason.NONE,
                         "Synthetic stub: illustrative metrics, no IK performed", command, command,
                         0.001, 0.0001, 0.0001, ConstraintChecks(True, True, True, True, True))
    hold_request = replace(request, target=replace(target, state="MANUAL_HOLD", valid=False,
                                                  hold_reason="manual_hold"))
    hold = IKResult(hold_request, ResultState.HOLD, FailureReason.NONE,
                    "Normal hold; do not advance reference", None, previous, 0)
    failure = IKResult(request, ResultState.FAULT, FailureReason.UNREACHABLE,
                       "Synthetic unreachable result; require explicit recovery", None, previous,
                       0.004, 0.2, 0.1, ConstraintChecks(residual=False))
    reconnect_request = replace(request, target=replace(target, source_connection_id=2,
                                state="FAULT_HOLD", valid=False, hold_reason="connection_changed"))
    reconnect = IKResult(reconnect_request, ResultState.REANCHOR_REQUIRED,
                         FailureReason.CONNECTION_CHANGED,
                         "Discard pending targets; no automatic cross-connection recovery",
                         None, previous, 0)
    return {"following": following, "normal_hold": hold, "unreachable": failure,
            "reconnect": reconnect}


def main():
    print(json.dumps({name: {**asdict(result), "success": result.success}
                      for name, result in contract_examples().items()}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
