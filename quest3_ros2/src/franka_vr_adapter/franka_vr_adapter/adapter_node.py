"""ROS 2 boundary between checked Quest 3 IK results and the FR3 controller."""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Sequence, TextIO

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def _add_repository_root_to_path() -> Path:
    """Find shared pure-Python contracts from a source or symlink install."""
    for parent in Path(__file__).resolve().parents:
        gate_file = parent / "data_collection" / "meta_quest" / "joint_command_gate.py"
        if gate_file.is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return parent
    raise ImportError(
        "Could not locate data_collection/meta_quest/joint_command_gate.py. "
        "Build franka_vr_adapter with --symlink-install from the DexArena checkout."
    )


_add_repository_root_to_path()

from data_collection.meta_quest.joint_command_gate import (  # noqa: E402
    AdapterState,
    COMMAND_JOINT_NAMES,
    JointCommandGate,
    POSITION_LOWER,
    POSITION_UPPER,
)
from data_collection.meta_quest.models import IKResult  # noqa: E402


FAULT_MODES = frozenset(
    {
        "none",
        "stop_input",
        "robot_state_stale",
        "stale_input",
        "wrong_joint_order",
        "joint_limit",
        "nan",
        "ik_failure",
        "command_source_conflict",
    }
)


class JsonlTraceWriter:
    """Line-buffered trace for requested, accepted, measured, and status data."""

    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser() if path else None
        self._file: TextIO | None = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("a", encoding="utf-8", buffering=1)

    def write(self, event: str, **fields: Any) -> None:
        if self._file is None:
            return
        payload = {"event": event, "monotonic_time_s": time.monotonic(), **fields}
        self._file.write(
            json.dumps(payload, allow_nan=False, separators=(",", ":"), sort_keys=True)
            + "\n"
        )

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


class SyntheticTargetSource:
    """Generate a small, bounded test trajectory anchored at measured joints."""

    def __init__(self, amplitude_rad: float, period_s: float) -> None:
        if not math.isfinite(amplitude_rad) or amplitude_rad <= 0.0:
            raise ValueError("synthetic_amplitude_rad must be positive and finite")
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("synthetic_period_s must be positive and finite")
        self.amplitude_rad = amplitude_rad
        self.period_s = period_s
        self.anchor: tuple[float, ...] | None = None
        self.started_s: float | None = None
        self.sequence = 0

    def set_anchor(self, positions: Sequence[float], now_s: float) -> None:
        if self.anchor is not None:
            return
        values = tuple(float(value) for value in positions)
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            raise ValueError("synthetic anchor must contain seven finite joints")
        self.anchor = values
        self.started_s = now_s

    def reanchor(self, positions: Sequence[float], now_s: float) -> None:
        """Restart the trajectory at measured state without reusing sequence IDs."""
        self.anchor = None
        self.started_s = None
        self.set_anchor(positions, now_s)

    def make_result(self, now_s: float) -> IKResult | None:
        if self.anchor is None or self.started_s is None:
            return None
        self.sequence += 1
        if self.sequence == 1:
            positions = self.anchor
        else:
            phase = 2.0 * math.pi * (now_s - self.started_s) / self.period_s
            unit_offset = 0.5 * (1.0 - math.cos(phase))
            room_up = POSITION_UPPER[0] - self.anchor[0]
            room_down = self.anchor[0] - POSITION_LOWER[0]
            direction = 1.0 if room_up >= room_down else -1.0
            safe_amplitude = min(self.amplitude_rad, max(room_up, room_down) * 0.5)
            first_joint = self.anchor[0] + direction * safe_amplitude * unit_offset
            positions = (first_joint,) + self.anchor[1:]
        return IKResult(
            joint_names=COMMAND_JOINT_NAMES,
            positions=positions,
            success=True,
            failure_reason=None,
            solve_time_s=0.0,
            position_residual_m=0.0,
            rotation_residual_rad=0.0,
            joint_limits_ok=True,
            motion_limits_ok=True,
            target_timestamp_s=now_s,
            result_timestamp_s=now_s,
            source_connection_id=0,
            source_sequence=self.sequence,
            control_state="FOLLOWING",
        )


class VrAdapterNode(Node):
    """Publish only joint targets accepted by the shared pure-Python gate."""

    def __init__(self) -> None:
        super().__init__("franka_vr_adapter")
        self._declare_parameters()

        self.enable_synthetic_input = bool(
            self.get_parameter("enable_synthetic_input").value
        )
        self.robot_state_settle_s = float(
            self.get_parameter("robot_state_settle_s").value
        )
        self.robot_state_settle_tolerance_rad = float(
            self.get_parameter("robot_state_settle_tolerance_rad").value
        )
        if self.robot_state_settle_s <= 0.0:
            raise ValueError("robot_state_settle_s must be positive")
        if self.robot_state_settle_tolerance_rad <= 0.0:
            raise ValueError("robot_state_settle_tolerance_rad must be positive")
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        if command_rate_hz <= 0.0 or status_rate_hz <= 0.0:
            raise ValueError("command_rate_hz and status_rate_hz must be positive")

        self.gate = JointCommandGate(
            robot_state_timeout_s=float(
                self.get_parameter("robot_state_timeout_s").value
            ),
            input_timeout_s=float(self.get_parameter("input_timeout_s").value),
            ik_timeout_s=float(self.get_parameter("ik_timeout_s").value),
            max_joint_step_rad=float(
                self.get_parameter("max_joint_step_rad").value
            ),
            velocity_scale=float(self.get_parameter("velocity_scale").value),
        )
        self.synthetic_source = SyntheticTargetSource(
            amplitude_rad=float(self.get_parameter("synthetic_amplitude_rad").value),
            period_s=float(self.get_parameter("synthetic_period_s").value),
        )
        self.command_source_conflict = bool(
            self.get_parameter("command_source_conflict").value
        )
        self.enable_fault_injection = bool(
            self.get_parameter("enable_fault_injection").value
        )
        self.fault_mode = str(self.get_parameter("fault_mode").value)
        if self.fault_mode not in FAULT_MODES:
            raise ValueError(f"unsupported fault_mode: {self.fault_mode}")
        if self.fault_mode != "none" and not self.enable_fault_injection:
            raise ValueError("non-default fault_mode requires enable_fault_injection=true")
        self._reanchor_requested = False
        self._gello_conflict_active = False
        self._settle_candidate: tuple[float, ...] | None = None
        self._robot_state_stable_since_s: float | None = None
        self.trace = JsonlTraceWriter(str(self.get_parameter("trace_path").value))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        measured_topic = str(self.get_parameter("measured_topic").value)
        accepted_topic = str(self.get_parameter("accepted_topic").value)
        raw_topic = str(self.get_parameter("raw_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.raw_publisher = self.create_publisher(JointState, raw_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.measured_subscription = self.create_subscription(
            JointState, measured_topic, self._on_measured_state, 10
        )
        self.accepted_subscription = self.create_subscription(
            JointState, accepted_topic, self._on_accepted_target, 10
        )

        self.raw_publish_count = 0
        self.accepted_count = 0
        self.have_robot_state = False
        self.last_state = AdapterState.WAIT_ROBOT
        self.last_reason = "robot_state_missing"

        self.status_timer = self.create_timer(1.0 / status_rate_hz, self._publish_status)
        self.command_timer = None
        if self.enable_synthetic_input:
            self.command_timer = self.create_timer(
                1.0 / command_rate_hz, self._publish_synthetic_target
            )
            self.get_logger().warning(
                "Synthetic input is ENABLED; commands still require a fresh robot state "
                "and JointCommandGate approval."
            )
        else:
            self.get_logger().info(
                "Synthetic input is disabled. The adapter will publish status only."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("measured_topic", "/right/joint_states")
        self.declare_parameter("accepted_topic", "/right/vr/accepted_joint_states")
        self.declare_parameter("raw_topic", "/right/vr/raw_joint_states")
        self.declare_parameter("status_topic", "/right/vr/status")
        self.declare_parameter("gello_topic", "/right/gello/raw_joint_states")
        self.declare_parameter("enable_synthetic_input", False)
        self.declare_parameter("command_source_conflict", False)
        self.declare_parameter("enable_fault_injection", False)
        self.declare_parameter("fault_mode", "none")
        self.declare_parameter("trace_path", "")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("robot_state_timeout_s", 0.25)
        self.declare_parameter("robot_state_settle_s", 0.5)
        self.declare_parameter("robot_state_settle_tolerance_rad", 0.01)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("ik_timeout_s", 0.15)
        self.declare_parameter("max_joint_step_rad", 0.10)
        self.declare_parameter("velocity_scale", 1.0)
        self.declare_parameter("synthetic_amplitude_rad", 0.02)
        self.declare_parameter("synthetic_period_s", 8.0)

    def _on_measured_state(self, message: JointState) -> None:
        now_s = time.monotonic()
        self.trace.write(
            "measured",
            joint_names=list(message.name),
            positions=list(message.position),
        )
        if self.enable_fault_injection and self.fault_mode == "robot_state_stale":
            return
        observed = self.gate.observe_robot_state(message.name, message.position, now_s)
        if not observed:
            self.have_robot_state = False
            self.last_state = AdapterState.STOP
            self.last_reason = "invalid_robot_state"
            return
        self.have_robot_state = True
        robot_positions = self.gate.robot_positions or ()
        if self.synthetic_source.anchor is None:
            state_changed = (
                self._settle_candidate is None
                or max(
                    abs(value - previous)
                    for value, previous in zip(robot_positions, self._settle_candidate)
                )
                > self.robot_state_settle_tolerance_rad
            )
            if state_changed:
                self._settle_candidate = robot_positions
                self._robot_state_stable_since_s = now_s
            elif (
                self._robot_state_stable_since_s is not None
                and now_s - self._robot_state_stable_since_s >= self.robot_state_settle_s
            ):
                self.synthetic_source.set_anchor(robot_positions, now_s)
                self.trace.write("initial_anchor", positions=list(robot_positions))
        if not self.enable_synthetic_input:
            self.last_state = AdapterState.HOLD
            self.last_reason = "synthetic_input_disabled"

    def _on_accepted_target(self, message: JointState) -> None:
        self.accepted_count += 1
        self.trace.write(
            "accepted",
            joint_names=list(message.name),
            positions=list(message.position),
        )

    def _on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        requested_mode = self.fault_mode
        for parameter in parameters:
            if parameter.name == "fault_mode":
                requested_mode = str(parameter.value)
        if requested_mode not in FAULT_MODES:
            return SetParametersResult(
                successful=False,
                reason=f"fault_mode must be one of {sorted(FAULT_MODES)}",
            )
        if requested_mode != "none" and not self.enable_fault_injection:
            return SetParametersResult(
                successful=False,
                reason="fault injection is disabled for this launch",
            )
        if requested_mode != self.fault_mode:
            previous_mode = self.fault_mode
            self.fault_mode = requested_mode
            if requested_mode == "none":
                self._reanchor_requested = True
            self.trace.write(
                "fault_mode",
                previous_mode=previous_mode,
                fault_mode=requested_mode,
            )
        return SetParametersResult(successful=True)

    def _gello_publisher_present(self) -> bool:
        return self.count_publishers(str(self.get_parameter("gello_topic").value)) > 0

    def _apply_fault(self, result: IKResult, now_s: float) -> IKResult:
        if not self.enable_fault_injection:
            return result
        if self.fault_mode == "stale_input":
            stale_s = now_s - self.gate.input_timeout_s - 0.05
            return replace(
                result,
                target_timestamp_s=stale_s,
                result_timestamp_s=stale_s,
            )
        if self.fault_mode == "wrong_joint_order":
            return replace(result, joint_names=tuple(reversed(COMMAND_JOINT_NAMES)))
        if self.fault_mode == "joint_limit":
            unsafe = (POSITION_UPPER[0] + 0.1,) + result.positions[1:]
            return replace(result, positions=unsafe)
        if self.fault_mode == "ik_failure":
            return replace(
                result,
                success=False,
                failure_reason="injected_failure",
                joint_limits_ok=False,
                motion_limits_ok=False,
            )
        return result

    def _publish_synthetic_target(self) -> None:
        now_s = time.monotonic()
        graph_conflict = self._gello_publisher_present()
        if self._gello_conflict_active and not graph_conflict:
            self._reanchor_requested = True
        self._gello_conflict_active = graph_conflict

        if self._reanchor_requested and self.gate.robot_positions is not None:
            self.synthetic_source.reanchor(self.gate.robot_positions, now_s)
            self.gate.reanchor_command_history()
            self._reanchor_requested = False
            self.trace.write(
                "reanchor",
                positions=list(self.gate.robot_positions),
            )

        if self.enable_fault_injection and self.fault_mode == "stop_input":
            self.last_state = AdapterState.HOLD
            self.last_reason = "synthetic_input_stopped"
            self.trace.write(
                "decision",
                state=str(self.last_state),
                reason=self.last_reason,
                publish=False,
                fault_mode=self.fault_mode,
            )
            return
        if self.enable_fault_injection and self.fault_mode == "nan":
            self.last_state = AdapterState.STOP
            self.last_reason = "ik_result_invalid:positions_non_finite"
            self.gate.reanchor_command_history()
            self.trace.write(
                "requested_invalid",
                reason=self.last_reason,
                fault_mode=self.fault_mode,
            )
            return

        result = self.synthetic_source.make_result(now_s)
        if result is None:
            self.last_state = AdapterState.WAIT_ROBOT
            self.last_reason = (
                "robot_state_settling" if self.have_robot_state else "robot_state_missing"
            )
            return
        result = self._apply_fault(result, now_s)
        self.trace.write(
            "requested",
            joint_names=list(result.joint_names),
            positions=list(result.positions),
            source_sequence=result.source_sequence,
            success=result.success,
            fault_mode=self.fault_mode,
        )
        decision = self.gate.assess(
            result,
            now_s,
            command_source_conflict=(
                self.command_source_conflict
                or graph_conflict
                or (
                    self.enable_fault_injection
                    and self.fault_mode == "command_source_conflict"
                )
            ),
        )
        self.last_state = decision.state
        self.last_reason = decision.reason
        self.trace.write(
            "decision",
            state=str(decision.state),
            reason=decision.reason,
            publish=decision.publish,
            fault_mode=self.fault_mode,
        )
        if not decision.publish or decision.positions is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "fr3_link0"
        message.name = list(COMMAND_JOINT_NAMES)
        message.position = list(decision.positions)
        self.raw_publisher.publish(message)
        self.raw_publish_count += 1
        self.trace.write(
            "raw",
            joint_names=list(message.name),
            positions=list(message.position),
            raw_publish_count=self.raw_publish_count,
        )

    def _publish_status(self) -> None:
        if self._gello_publisher_present():
            state = AdapterState.STOP
            reason = "command_source_conflict"
        elif not self.have_robot_state:
            state = AdapterState.WAIT_ROBOT
            reason = "robot_state_missing"
        elif not self.enable_synthetic_input:
            state = AdapterState.HOLD
            reason = "synthetic_input_disabled"
        else:
            state = self.last_state
            reason = self.last_reason
        status = String()
        status.data = json.dumps(
            {
                "state": str(state),
                "reason": reason,
                "synthetic_input_enabled": self.enable_synthetic_input,
                "have_robot_state": self.have_robot_state,
                "raw_publish_count": self.raw_publish_count,
                "accepted_count": self.accepted_count,
                "fault_mode": self.fault_mode,
                "monotonic_time_s": time.monotonic(),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.status_publisher.publish(status)
        self.trace.write(
            "status",
            state=str(state),
            reason=reason,
            fault_mode=self.fault_mode,
            raw_publish_count=self.raw_publish_count,
            accepted_count=self.accepted_count,
        )

    def destroy_node(self) -> None:
        self.trace.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VrAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
