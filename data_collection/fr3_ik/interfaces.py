"""A/B contract draft v1; Python 3.10, standard library only.

These types validate structure, not robot safety. B must independently check
freshness, command ownership and controller state before publishing.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol

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

    def __post_init__(self):
        require(isinstance(self.request, IKRequest) and isinstance(self.last_safe_target, JointTarget)
                and isinstance(self.checks, ConstraintChecks), "Typed request, last safe target and checks required")
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


class IKSolver(Protocol):
    def solve(self, target_pose: IKRequest, previous_joint_target: JointTarget,
              measured_joint_state: MeasuredJointState | None) -> IKResult:
        """Pure A-line solve. A2 implements this; missing measured feedback is explicit."""
        ...
