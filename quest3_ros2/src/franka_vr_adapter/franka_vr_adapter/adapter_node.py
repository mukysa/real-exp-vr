"""Thin ROS wrapper around the shared, pure-Python AdapterCore."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time
from typing import Any, TextIO

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def _add_repository_root_to_path() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "data_collection/meta_quest/adapter_core.py").is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return parent
    raise ImportError("Build franka_vr_adapter with --symlink-install from the project checkout")


_add_repository_root_to_path()

from data_collection.fr3_ik.interfaces import (  # noqa: E402
    ClockDomain,
    JOINT_NAMES,
    JointVector,
    MeasuredJointState,
    Timestamp,
    ik_result_from_dict,
)
from data_collection.meta_quest.adapter_core import AdapterCore, AdapterState  # noqa: E402


class JsonlTraceWriter:
    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser() if path else None
        self._file: TextIO | None = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("a", encoding="utf-8", buffering=1)

    def write(self, event: str, **fields: Any) -> None:
        if self._file is not None:
            self._file.write(json.dumps(
                {"event": event, "monotonic_time_s": time.monotonic(), **fields},
                allow_nan=False, separators=(",", ":"), sort_keys=True,
            ) + "\n")

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


class VrAdapterNode(Node):
    """Deserialize strict IK results, gate them, and publish accepted targets only."""

    def __init__(self) -> None:
        super().__init__("franka_vr_adapter")
        for name, default in (
            ("side", "right"),
            ("measured_topic", "/right/joint_states"),
            ("accepted_topic", "/right/vr/accepted_joint_states"),
            ("ik_result_topic", "/right/vr/ik_result"),
            ("raw_topic", "/right/vr/raw_joint_states"),
            ("status_topic", "/right/vr/status"),
            ("trace_path", ""),
            ("status_rate_hz", 2.0),
            ("robot_state_timeout_s", 0.25),
            ("input_timeout_s", 0.25),
            ("ik_timeout_s", 0.15),
            ("max_joint_step_rad", 0.10),
            ("velocity_scale", 1.0),
            ("expected_sequence_step", 2),
        ):
            self.declare_parameter(name, default)

        self.side = str(self.get_parameter("side").value)
        def side_topic(parameter_name: str) -> str:
            configured = str(self.get_parameter(parameter_name).value)
            return configured.replace("/right/", f"/{self.side}/", 1)
        self.measured_topic = side_topic("measured_topic")
        self.accepted_topic = side_topic("accepted_topic")
        self.ik_result_topic = side_topic("ik_result_topic")
        self.raw_topic = side_topic("raw_topic")
        self.status_topic = side_topic("status_topic")
        expected_sequence_step = int(self.get_parameter("expected_sequence_step").value)
        self.core = AdapterCore(
            side=self.side,
            robot_state_timeout_s=float(self.get_parameter("robot_state_timeout_s").value),
            input_timeout_s=float(self.get_parameter("input_timeout_s").value),
            ik_timeout_s=float(self.get_parameter("ik_timeout_s").value),
            max_joint_step_rad=float(self.get_parameter("max_joint_step_rad").value),
            velocity_scale=float(self.get_parameter("velocity_scale").value),
            # A non-positive value disables exact-gap checking while retaining
            # the strictly-increasing check.  Live control intentionally emits
            # no IK results while READY, so a new Y takeover can legitimately
            # resume at a later receiver sequence.
            expected_sequence_step=(
                expected_sequence_step if expected_sequence_step > 0 else None
            ),
        )
        self.trace = JsonlTraceWriter(str(self.get_parameter("trace_path").value))
        self.latest_measured_positions: tuple[float, ...] | None = None
        self.latest_measured_received_s: float | None = None
        self.raw_publish_count = 0
        self.accepted_count = 0
        self.last_state = AdapterState.WAIT_ROBOT
        self.last_reason = "robot_state_missing"

        self.raw_publisher = self.create_publisher(
            JointState, self.raw_topic, 10)
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10)
        self.create_subscription(
            JointState, self.measured_topic,
            self._on_measured_state, 10)
        self.create_subscription(
            JointState, self.accepted_topic,
            self._on_accepted_target, 10)
        self.create_subscription(
            String, self.ik_result_topic,
            self._on_ik_result, 10)
        status_rate = float(self.get_parameter("status_rate_hz").value)
        if not math.isfinite(status_rate) or status_rate <= 0:
            raise ValueError("status_rate_hz must be positive and finite")
        self.create_timer(1.0 / status_rate, self._publish_status)

    @staticmethod
    def _canonical(name: str) -> str:
        return name.removeprefix("right_").removeprefix("left_")

    def _on_measured_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            self.last_state = AdapterState.STOP
            self.last_reason = "invalid_robot_state"
            return
        if any(name.startswith(("left_", "right_")) and not name.startswith(f"{self.side}_")
               for name in message.name):
            self.last_state = AdapterState.STOP
            self.last_reason = "measured_side_mismatch"
            return
        values = {self._canonical(name): float(value)
                  for name, value in zip(message.name, message.position)}
        try:
            if len(values) != 7:
                raise ValueError("expected seven unique joint names")
            positions = tuple(values[name] for name in JOINT_NAMES)
        except (KeyError, ValueError):
            self.last_state = AdapterState.STOP
            self.last_reason = "invalid_robot_state"
            return
        if not all(math.isfinite(value) for value in positions):
            self.last_state = AdapterState.STOP
            self.last_reason = "invalid_robot_state"
            return
        self.latest_measured_positions = positions
        self.latest_measured_received_s = time.monotonic()
        self.trace.write("measured", positions=list(positions))

    def _on_accepted_target(self, message: JointState) -> None:
        self.accepted_count += 1
        self.trace.write("accepted", joint_names=list(message.name), positions=list(message.position))

    def _gello_publisher_present(self) -> bool:
        return self.count_publishers(f"/{self.side}/gello/raw_joint_states") > 0

    def _on_ik_result(self, message: String) -> None:
        try:
            result = ik_result_from_dict(json.loads(message.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.last_state = AdapterState.STOP
            self.last_reason = f"ik_contract_invalid:{type(exc).__name__}"
            self.core.fault_latched = True
            self.trace.write("decision", publish=False, reason=self.last_reason)
            return
        if result.computed_stamp.domain != ClockDomain.HOST_MONOTONIC:
            self.last_state = AdapterState.STOP
            self.last_reason = "live_clock_must_be_host_monotonic"
            self.core.fault_latched = True
            return
        now_s = time.monotonic()
        now = Timestamp(now_s, result.computed_stamp.domain, result.computed_stamp.clock_id)
        measured = None
        if self.latest_measured_positions is not None and self.latest_measured_received_s is not None:
            measured = MeasuredJointState(
                JointVector(self.latest_measured_positions),
                Timestamp(self.latest_measured_received_s, now.domain, now.clock_id),
                self.measured_topic,
                True,
            )
        decision = self.core.assess(
            result, measured, now,
            quest_connected=True,
            gello_conflict=self._gello_publisher_present(),
        )
        self.last_state = decision.state
        self.last_reason = decision.reason
        self.trace.write(
            "decision", publish=decision.would_publish, reason=decision.reason,
            sequence=result.request.target.source_sequence,
            target_position_m=list(result.request.target.pose.position),
            command_positions_rad=(
                None if result.command is None
                else list(result.command.joints.positions_rad)
            ),
        )
        if not decision.would_publish or decision.joint_target is None:
            return
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = f"{self.side}_fr3_link0"
        command.name = list(JOINT_NAMES)
        command.position = list(decision.joint_target.joints.positions_rad)
        self.raw_publisher.publish(command)
        self.raw_publish_count += 1
        self.trace.write("raw", positions=list(command.position), raw_publish_count=self.raw_publish_count)

    def _publish_status(self) -> None:
        status = String()
        status.data = json.dumps({
            "state": str(self.last_state),
            "reason": self.last_reason,
            "have_robot_state": self.latest_measured_positions is not None,
            "raw_publish_count": self.raw_publish_count,
            "accepted_count": self.accepted_count,
            "fault_latched": self.core.fault_latched,
            "input": "strict_ik_result_topic",
        }, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(status)

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
