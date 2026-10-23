"""A/B contract draft v1; Python 3.10, standard library only.

These types validate structure, not robot safety. B must independently check
freshness, command ownership and controller state before publishing.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.models import TargetPose


CONTRACT_VERSION = 1
JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
HARDWARE_JOINT_NAMES = tuple(f"right_fr3_joint{i}" for i in range(1, 8))
BASE_FRAME = "right_fr3_link0"
TCP_FRAME = "right_fr3_hand_tcp"


class ClockDomain(str, Enum):
    HOST_MONOTONIC = "host_monotonic"
    REPLAY = "replay"
    ROS = "ros"


class ResultState(str, Enum):
    FOLLOWING = "FOLLOWING"
    HOLD = "HOLD"
    FAULT = "FAULT"
    REANCHOR_REQUIRED = "REANCHOR_REQUIRED"


class FailureReason(str, Enum):
    NONE = "none"
    INVALID_INPUT = "invalid_input"
    UNREACHABLE = "unreachable"
    RESIDUAL = "residual_exceeded"
    JOINT_LIMIT = "joint_limit"
    STEP_LIMIT = "step_limit"
    VELOCITY_LIMIT = "velocity_limit"
    ACCELERATION_LIMIT = "acceleration_limit"
    SOLVE_TIMEOUT = "solve_timeout"
    STALE_INPUT = "stale_input"
    STALE_ROBOT_STATE = "stale_robot_state"
    CONNECTION_CHANGED = "connection_changed"
    REANCHOR_REQUIRED = "reanchor_required"
    SOLVER_ERROR = "solver_error"


def require(condition, message):
    if not condition:
        raise ValueError(message)


@dataclass(frozen=True)
class Timestamp:
    seconds: float
    domain: ClockDomain
    clock_id: str  # e.g. host/boot id, replay SHA-256, or ROS clock session.

    def __post_init__(self):
        require(math.isfinite(self.seconds) and self.seconds >= 0, "Invalid timestamp")
        require(isinstance(self.domain, ClockDomain), "Explicit ClockDomain required")
        require(bool(self.clock_id.strip()), "clock_id is required")

    def elapsed_since(self, earlier: "Timestamp") -> float:
        require((self.domain, self.clock_id) == (earlier.domain, earlier.clock_id),
                "Cannot compare different clock domains or sessions")
        return self.seconds - earlier.seconds


@dataclass(frozen=True)
class JointVector:
    positions_rad: tuple[float, ...]
    names: tuple[str, ...] = JOINT_NAMES

    def __post_init__(self):
        positions = tuple(float(x) for x in self.positions_rad)
        require(len(positions) == 7 and all(math.isfinite(x) for x in positions),
                "Expected seven finite joint positions in radians")
        require(tuple(self.names) == JOINT_NAMES, "Expected GELLO command joint order")
        object.__setattr__(self, "positions_rad", positions)
        object.__setattr__(self, "names", tuple(self.names))


@dataclass(frozen=True)
class JointTarget:
    """A-validated reference (or explicit initial FK anchor), NOT controller accepted feedback."""

    joints: JointVector
    stamp: Timestamp

    def __post_init__(self):
        require(isinstance(self.joints, JointVector) and isinstance(self.stamp, Timestamp),
                "Typed joints and timestamp required")


@dataclass(frozen=True)
class MeasuredJointState:
    """Actual feedback only; use None for offline absence, never substitute a target."""

    joints: JointVector
    stamp: Timestamp
    source: str  # Concrete topic or snapshot identifier.
    valid: bool

    def __post_init__(self):
        require(isinstance(self.joints, JointVector) and isinstance(self.stamp, Timestamp),
                "Typed measured joints and timestamp required")
        require(bool(self.source.strip()), "Measured state source required")
        require(type(self.valid) is bool, "valid must be bool")


@dataclass(frozen=True)
class IKRequest:
    target: TargetPose  # Reuse stage-2 pose/state contract without importing its state machine.
    source_stamp: Timestamp  # Original receiver arrival timestamp, not the device poll stamp.
    target_stamp: Timestamp  # Generation time in live mode; logical replay time offline.
    tool_frame: str = TCP_FRAME
    side: str = "right"

    def __post_init__(self):
        require(isinstance(self.target, TargetPose), "Processed TargetPose required")
        require(isinstance(self.source_stamp, Timestamp) and isinstance(self.target_stamp, Timestamp),
                "Explicit source and target clocks required")
        require(self.side in {"left", "right"}, "Expected left or right FR3 arm")
        require(self.target.frame_id == f"{self.side}_fr3_link0", "Target must use the selected FR3 base frame")
        require(self.tool_frame == f"{self.side}_fr3_hand_tcp", "Draft v1 controls the nominal Franka hand TCP")
        require(self.target.generated_monotonic_s == self.target_stamp.seconds,
                "Target generation time must match its explicit timestamp")
        require(self.target.state in {"WAIT_INPUT", "READY", "FOLLOWING", "MANUAL_HOLD", "FAULT_HOLD"},
                "Preview/raw targets are not control inputs")
        require(not self.target.valid or self.target.state == "FOLLOWING",
                "Only FOLLOWING may carry a valid advancing target")


@dataclass(frozen=True)
class ConstraintChecks:
    """None means not evaluated, never pass by default."""

    residual: bool | None = None
    joint_limits: bool | None = None
    step: bool | None = None
    velocity: bool | None = None
    acceleration: bool | None = None

    def __post_init__(self):
        require(all(x is None or type(x) is bool for x in self.__dict__.values()),
                "Checks must be bool or None")

    @property
    def all_passed(self) -> bool:
        return all(x is True for x in self.__dict__.values())


@dataclass(frozen=True)
class IKResult:
    request: IKRequest  # Preserve sequence, connection, clocks and upstream reason intact.
    state: ResultState
    reason: FailureReason
    detail: str
    command: JointTarget | None
    last_safe_target: JointTarget
    solve_time_s: float
    position_residual_m: float | None = None
    rotation_residual_rad: float | None = None
    checks: ConstraintChecks = ConstraintChecks()
    computed_stamp: Timestamp | None = None

    def __post_init__(self):
        require(isinstance(self.request, IKRequest) and isinstance(self.last_safe_target, JointTarget)
                and isinstance(self.checks, ConstraintChecks), "Typed request, last safe target and checks required")
        require(isinstance(self.computed_stamp, Timestamp), "Explicit computation timestamp required")
        require((self.computed_stamp.domain, self.computed_stamp.clock_id) ==
                (self.request.target_stamp.domain, self.request.target_stamp.clock_id),
                "Computation and target timestamps must share a clock")
        require(self.computed_stamp.seconds >= self.request.target_stamp.seconds,
                "Computation timestamp cannot precede target generation")
        require(self.command is None or isinstance(self.command, JointTarget), "Invalid command type")
        require(isinstance(self.state, ResultState) and isinstance(self.reason, FailureReason),
                "Explicit result state/reason required")
        require(bool(self.detail.strip()), "Result detail required")
        require(math.isfinite(self.solve_time_s) and self.solve_time_s >= 0, "Invalid solve time")
        for value in (self.position_residual_m, self.rotation_residual_rad):
            require(value is None or (math.isfinite(value) and value >= 0), "Invalid residual")
        if self.state == ResultState.FOLLOWING:
            require(self.reason == FailureReason.NONE and self.command is not None,
                    "FOLLOWING requires a command and no failure")
            require(self.request.target.valid and self.request.target.state == "FOLLOWING",
                    "Cannot advance an invalid or held input")
            require(self.checks.all_passed, "All constraints must pass before emitting a command")
            require(self.position_residual_m is not None and self.rotation_residual_rad is not None,
                    "Successful IK requires FK residuals")
            require(self.command.stamp == self.request.target_stamp, "Command timestamp mismatch")
            require(self.last_safe_target == self.command, "Successful target must become last safe")
        else:
            require(self.command is None, "HOLD/failure must not expose a new command")
            if self.state == ResultState.HOLD:
                require(self.reason == FailureReason.NONE, "Normal HOLD is not an IK fault")
                require(not self.request.target.valid and self.request.target.state in
                        {"WAIT_INPUT", "READY", "MANUAL_HOLD"}, "Normal HOLD requires held input")
            else:
                require(self.reason != FailureReason.NONE, "Fault/reanchor requires a reason")

    @property
    def success(self) -> bool:
        """True only for a new checked target; not a permission to publish to a robot."""
        return self.state == ResultState.FOLLOWING

    def command_for_adapter(self) -> JointTarget:
        if not self.success or self.command is None:
            raise ValueError("No new command: adapter must handle HOLD/STOP/reanchor explicitly")
        return self.command


def _timestamp_to_dict(stamp: Timestamp) -> dict:
    return {"seconds": stamp.seconds, "domain": stamp.domain.value, "clock_id": stamp.clock_id}


def _timestamp_from_dict(payload: dict) -> Timestamp:
    return Timestamp(float(payload["seconds"]), ClockDomain(payload["domain"]), str(payload["clock_id"]))


def ik_result_to_dict(result: IKResult) -> dict:
    """Serialize the sole A/B IK contract without losing clock or state semantics."""
    target = result.request.target
    def joint_target(value: JointTarget | None):
        if value is None:
            return None
        return {
            "joint_names": list(value.joints.names),
            "positions_rad": list(value.joints.positions_rad),
            "stamp": _timestamp_to_dict(value.stamp),
        }
    return {
        "schema": "fr3_ik_result_v1",
        "request": {
            "side": result.request.side,
            "tool_frame": result.request.tool_frame,
            "source_stamp": _timestamp_to_dict(result.request.source_stamp),
            "target_stamp": _timestamp_to_dict(result.request.target_stamp),
            "target": {
                "position": list(target.pose.position),
                "quaternion_xyzw": list(target.pose.quaternion_xyzw),
                "frame_id": target.frame_id,
                "source_sequence": target.source_sequence,
                "source_connection_id": target.source_connection_id,
                "generated_monotonic_s": target.generated_monotonic_s,
                "state": target.state,
                "valid": target.valid,
                "hold_reason": target.hold_reason,
            },
        },
        "state": result.state.value,
        "reason": result.reason.value,
        "detail": result.detail,
        "command": joint_target(result.command),
        "last_safe_target": joint_target(result.last_safe_target),
        "solve_time_s": result.solve_time_s,
        "position_residual_m": result.position_residual_m,
        "rotation_residual_rad": result.rotation_residual_rad,
        "checks": {
            "residual": result.checks.residual,
            "joint_limits": result.checks.joint_limits,
            "step": result.checks.step,
            "velocity": result.checks.velocity,
            "acceleration": result.checks.acceleration,
        },
        "computed_stamp": _timestamp_to_dict(result.computed_stamp),
    }


def ik_result_from_dict(payload: dict) -> IKResult:
    """Validate and deserialize an IK result received by the ROS adapter."""
    require(payload.get("schema") == "fr3_ik_result_v1", "Unsupported IK result schema")
    request_payload = payload["request"]
    target_payload = request_payload["target"]
    target = TargetPose(
        RigidPose(tuple(target_payload["position"]), tuple(target_payload["quaternion_xyzw"])),
        str(target_payload["frame_id"]),
        int(target_payload["source_sequence"]),
        int(target_payload["source_connection_id"]),
        float(target_payload["generated_monotonic_s"]),
        str(target_payload["state"]),
        bool(target_payload["valid"]),
        target_payload.get("hold_reason"),
    )
    request = IKRequest(
        target,
        _timestamp_from_dict(request_payload["source_stamp"]),
        _timestamp_from_dict(request_payload["target_stamp"]),
        str(request_payload["tool_frame"]),
        str(request_payload["side"]),
    )
    def joint_target(value: dict | None) -> JointTarget | None:
        if value is None:
            return None
        return JointTarget(
            JointVector(tuple(value["positions_rad"]), tuple(value["joint_names"])),
            _timestamp_from_dict(value["stamp"]),
        )
    return IKResult(
        request=request,
        state=ResultState(payload["state"]),
        reason=FailureReason(payload["reason"]),
        detail=str(payload["detail"]),
        command=joint_target(payload.get("command")),
        last_safe_target=joint_target(payload["last_safe_target"]),
        solve_time_s=float(payload["solve_time_s"]),
        position_residual_m=(None if payload.get("position_residual_m") is None
                             else float(payload["position_residual_m"])),
        rotation_residual_rad=(None if payload.get("rotation_residual_rad") is None
                               else float(payload["rotation_residual_rad"])),
        checks=ConstraintChecks(**payload["checks"]),
        computed_stamp=_timestamp_from_dict(payload["computed_stamp"]),
    )


class IKSolver(Protocol):
    def solve(self, target_pose: IKRequest, previous_joint_target: JointTarget,
              measured_joint_state: MeasuredJointState | None) -> IKResult:
        """Pure A-line solve. A2 implements this; missing measured feedback is explicit."""
        ...
