"""Offline-testable Quest clutch, hold, recovery, and safety state machine."""

from __future__ import annotations

from collections import Counter
from enum import StrEnum
import math

from .calibration import PoseCalibration, RigidPose
from .coordinates import IDENTITY_POSE, unity_to_robot_pose
from .models import TargetPose
from .receiver import ReceivedPose
from .safety import (
    SafetyLimits,
    align_quaternion_hemisphere,
    source_discontinuity,
    target_violation,
)


class TeleoperationState(StrEnum):
    WAIT_INPUT = "WAIT_INPUT"
    READY = "READY"
    FOLLOWING = "FOLLOWING"
    MANUAL_HOLD = "MANUAL_HOLD"
    FAULT_HOLD = "FAULT_HOLD"


class DebouncedButtonEdge:
    """Require an observed release and debounce accepted rising edges."""

    def __init__(self, debounce_s: float) -> None:
        if not math.isfinite(debounce_s) or debounce_s < 0.0:
            raise ValueError("debounce_s must be non-negative and finite")
        self.debounce_s = debounce_s
        self._previous = False
        self._release_observed = False
        self._last_edge_time: float | None = None

    def reset(self, pressed: bool) -> None:
        self._previous = bool(pressed)
        self._release_observed = not pressed

    def require_release(self) -> None:
        self._previous = True
        self._release_observed = False

    def update(self, pressed: bool, timestamp_s: float) -> bool:
        pressed = bool(pressed)
        if not pressed:
            self._release_observed = True
        interval_ok = (
            self._last_edge_time is None
            or timestamp_s - self._last_edge_time >= self.debounce_s
        )
        rising = pressed and not self._previous and self._release_observed and interval_ok
        if rising:
            self._last_edge_time = timestamp_s
            self._release_observed = False
        self._previous = pressed
        return rising


class TeleoperationStateMachine:
    """Generate safe preview targets without importing or commanding a robot."""

    def __init__(
        self,
        target_anchor: RigidPose,
        *,
        side: str = "right",
        engage_button: str = "clutch",
        frame_id: str = "right_teleop_preview",
        position_scale: float = 1.0,
        controller_to_tool: RigidPose = IDENTITY_POSE,
        limits: SafetyLimits | None = None,
        min_ready_samples: int = 3,
        button_debounce_s: float = 0.15,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if engage_button not in {"clutch", "record"}:
            raise ValueError("engage_button must be clutch or record")
        if not frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(position_scale) or position_scale <= 0.0:
            raise ValueError("position_scale must be positive and finite")
        if min_ready_samples < 1:
            raise ValueError("min_ready_samples must be positive")
        self.target_anchor = target_anchor
        self.side = side
        self.engage_button = engage_button
        self.frame_id = frame_id
        self.position_scale = position_scale
        self.controller_to_tool = controller_to_tool
        self.limits = limits or SafetyLimits()
        self.min_ready_samples = min_ready_samples
        self.button = DebouncedButtonEdge(button_debounce_s)
        self.state = TeleoperationState.WAIT_INPUT
        self.state_counts: Counter[str] = Counter()
        self.hold_reason_counts: Counter[str] = Counter()
        self.anchor_sequences: list[int] = []
        self._calibration: PoseCalibration | None = None
        self._connection_id: int | None = None
        self._last_receive_time: float | None = None
        self._last_source_pose: RigidPose | None = None
        self._last_safe_target = target_anchor
        self._last_target_time: float | None = None
        self._last_received: ReceivedPose | None = None
        self._healthy_samples = 0
        self._fault_reason: str | None = None

    def _button_pressed(self, received: ReceivedPose) -> bool:
        value = getattr(received.sample, self.engage_button)
        return bool(value)

    def _target(
        self,
        received: ReceivedPose,
        *,
        valid: bool,
        state: TeleoperationState,
        hold_reason: str | None = None,
        timestamp_s: float | None = None,
    ) -> TargetPose:
        target = TargetPose(
            pose=self._last_safe_target,
            frame_id=self.frame_id,
            source_sequence=received.sequence,
            source_connection_id=received.connection_id,
            generated_monotonic_s=(
                received.received_monotonic_s if timestamp_s is None else timestamp_s
            ),
            state=state.value,
            valid=valid,
            hold_reason=hold_reason,
        )
        self.state_counts[state.value] += 1
        if hold_reason is not None:
            self.hold_reason_counts[hold_reason] += 1
        return target

    def _hold(self, received: ReceivedPose, reason: str) -> TargetPose:
        return self._target(
            received,
            valid=False,
            state=self.state,
            hold_reason=reason,
        )

    def _enter_fault(self, reason: str) -> None:
        self.state = TeleoperationState.FAULT_HOLD
        self._fault_reason = reason
        self._calibration = None
        self._healthy_samples = 0
        self.button.require_release()

    def _start_following(self, received: ReceivedPose, source_pose: RigidPose) -> TargetPose:
        self._calibration = PoseCalibration.from_anchor(source_pose, self._last_safe_target)
        self.state = TeleoperationState.FOLLOWING
        self._fault_reason = None
        self._healthy_samples = 0
        self._last_target_time = received.received_monotonic_s
        self.anchor_sequences.append(received.sequence)
        return self._target(received, valid=True, state=self.state)

    def update(self, received: ReceivedPose) -> TargetPose | None:
        sample = received.sample
        if sample.source != "controller" or sample.side != self.side:
            return None
        source_pose = unity_to_robot_pose(
            RigidPose(sample.position, sample.quaternion_xyzw),
            position_scale=self.position_scale,
            controller_to_tool=self.controller_to_tool,
        )
        pressed = self._button_pressed(received)
        if self._connection_id is None:
            self._connection_id = received.connection_id
            self.button.reset(pressed)
        elif received.connection_id != self._connection_id:
            self._connection_id = received.connection_id
            self._last_receive_time = received.received_monotonic_s
            self._last_source_pose = source_pose
            self._last_received = received
            self._enter_fault("connection_changed")
            return self._hold(received, "connection_changed")

        if not sample.tracked:
            self._last_receive_time = received.received_monotonic_s
            self._last_source_pose = None
            self._last_received = received
            self._enter_fault("tracking_invalid")
            return self._hold(received, "tracking_invalid")

        elapsed_source = None
        if self._last_receive_time is not None:
            elapsed_source = received.received_monotonic_s - self._last_receive_time
            if elapsed_source <= 0.0:
                self._last_received = received
                self._enter_fault("source_time_not_increasing")
                return self._hold(received, "source_time_not_increasing")
            if elapsed_source > self.limits.stale_timeout_s:
                self._last_receive_time = received.received_monotonic_s
                self._last_source_pose = source_pose
                self._last_received = received
                self._enter_fault("stale_gap")
                return self._hold(received, "stale_gap")

        if self.state == TeleoperationState.FOLLOWING and self._last_source_pose is not None:
            reason = source_discontinuity(self._last_source_pose, source_pose, self.limits)
            if reason is not None:
                self._last_receive_time = received.received_monotonic_s
                self._last_source_pose = source_pose
                self._last_received = received
                self._enter_fault(reason)
                return self._hold(received, reason)

        self._last_receive_time = received.received_monotonic_s
        self._last_source_pose = source_pose
        self._last_received = received
        edge = self.button.update(pressed, received.received_monotonic_s)

        if self.state == TeleoperationState.WAIT_INPUT:
            self._healthy_samples += 1
            if self._healthy_samples >= self.min_ready_samples:
                self.state = TeleoperationState.READY
                return self._hold(received, "ready_for_engage")
            return self._hold(received, "warming_up")

        if self.state == TeleoperationState.READY:
            if edge:
                return self._start_following(received, source_pose)
            return self._hold(received, "ready_for_engage")

        if self.state == TeleoperationState.MANUAL_HOLD:
            if edge:
                return self._start_following(received, source_pose)
            return self._hold(received, "manual_hold")

        if self.state == TeleoperationState.FAULT_HOLD:
            self._healthy_samples += 1
            if edge and self._healthy_samples >= self.min_ready_samples:
                return self._start_following(received, source_pose)
            return self._hold(received, self._fault_reason or "fault_hold")

        if edge:
            self.state = TeleoperationState.MANUAL_HOLD
            self._calibration = None
            return self._hold(received, "manual_hold")

        assert self._calibration is not None
        mapped = align_quaternion_hemisphere(
            self._calibration.apply(source_pose),
            self._last_safe_target,
        )
        assert self._last_target_time is not None
        violation = target_violation(
            workspace_anchor=self.target_anchor,
            previous_target=self._last_safe_target,
            current_target=mapped,
            elapsed_s=received.received_monotonic_s - self._last_target_time,
            limits=self.limits,
        )
        if violation is not None:
            self._enter_fault(violation)
            return self._hold(received, violation)
        self._last_safe_target = mapped
        self._last_target_time = received.received_monotonic_s
        return self._target(received, valid=True, state=self.state)

    def tick(self, now_monotonic_s: float) -> TargetPose | None:
        """Enter fault hold when a live stream stops producing packets."""

        if (
            self.state != TeleoperationState.FOLLOWING
            or self._last_receive_time is None
            or self._last_received is None
            or now_monotonic_s - self._last_receive_time <= self.limits.stale_timeout_s
        ):
            return None
        self._enter_fault("stale_timeout")
        return self._target(
            self._last_received,
            valid=False,
            state=self.state,
            hold_reason="stale_timeout",
            timestamp_s=now_monotonic_s,
        )
