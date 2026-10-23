"""Exercise fake-only adapter faults and write a machine-readable result."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Callable

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String


FAULT_CASES = (
    ("stop_input", "stop_input", "synthetic_input_stopped"),
    ("robot_state_stale", "robot_state_stale", "robot_state_stale"),
    ("stale_input", "stale_input", "input_stale"),
    ("wrong_joint_order", "wrong_joint_order", "joint_order_mismatch"),
    ("joint_limit", "joint_limit", "joint_position_limit"),
    ("nan", "nan", "ik_result_invalid:positions_non_finite"),
    ("ik_failure", "ik_failure", "ik_failure:injected_failure"),
    ("gello_conflict", "none", "command_source_conflict"),
)


class FaultInjectionRunner(Node):
    def __init__(self) -> None:
        super().__init__("vr_fault_injection_runner")
        self.raw: list[tuple[float, tuple[float, ...]]] = []
        self.accepted: list[tuple[float, tuple[float, ...]]] = []
        self.measured: list[tuple[float, tuple[float, ...]]] = []
        self.commanded: list[tuple[float, tuple[float, ...]]] = []
        self.status: list[tuple[float, dict[str, object]]] = []
        self.create_subscription(
            JointState, "/right/vr/raw_joint_states", self._raw_callback, 50
        )
        self.create_subscription(
            JointState,
            "/right/vr/accepted_joint_states",
            self._accepted_callback,
            50,
        )
        self.create_subscription(
            JointState, "/right/joint_states", self._measured_callback, 50
        )
        self.create_subscription(
            JointState,
            "/right/franka/commanded_joint_states",
            self._commanded_callback,
            50,
        )
        self.create_subscription(String, "/right/vr/status", self._status_callback, 20)
        self.parameter_client = self.create_client(
            SetParameters, "/franka_vr_adapter/set_parameters"
        )
        self.gello_publisher = None

    @staticmethod
    def _sample(message: JointState) -> tuple[float, tuple[float, ...]]:
        return time.monotonic(), tuple(float(value) for value in message.position)

    def _raw_callback(self, message: JointState) -> None:
        self.raw.append(self._sample(message))

    def _accepted_callback(self, message: JointState) -> None:
        self.accepted.append(self._sample(message))

    def _measured_callback(self, message: JointState) -> None:
        self.measured.append(self._sample(message))

    def _commanded_callback(self, message: JointState) -> None:
        self.commanded.append(self._sample(message))

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.status.append((time.monotonic(), payload))

    def pump(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, deadline - time.monotonic()))

    def wait_for(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def set_fault_mode(self, mode: str) -> tuple[bool, str]:
        if not self.parameter_client.wait_for_service(timeout_sec=3.0):
            return False, "parameter_service_unavailable"
        request = SetParameters.Request()
        request.parameters = [Parameter("fault_mode", value=mode).to_parameter_msg()]
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None or not response.results:
            return False, "parameter_service_no_response"
        result = response.results[0]
        return bool(result.successful), str(result.reason)

    def latest_status_reason(self) -> str | None:
        if not self.status:
            return None
        return str(self.status[-1][1].get("reason"))

    def start_gello_conflict(self) -> None:
        self.gello_publisher = self.create_publisher(
            JointState, "/right/gello/raw_joint_states", 10
        )

    def stop_gello_conflict(self) -> None:
        if self.gello_publisher is not None:
            self.destroy_publisher(self.gello_publisher)
            self.gello_publisher = None


def _max_span(samples: list[tuple[float, tuple[float, ...]]]) -> float | None:
    valid = [positions for _, positions in samples if len(positions) == 7]
    if len(valid) < 2:
        return None
    return max(
        max(values) - min(values)
        for values in zip(*valid)
    )


def _max_error(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    if len(left) != 7 or len(right) != 7:
        return math.inf
    return max(abs(a - b) for a, b in zip(left, right))


def run(output_path: Path) -> dict[str, object]:
    rclpy.init()
    node = FaultInjectionRunner()
    results: list[dict[str, object]] = []
    try:
        baseline_ready = node.wait_for(
            lambda: len(node.raw) >= 5 and bool(node.measured) and bool(node.status),
            8.0,
        )
        if not baseline_ready:
            raise RuntimeError("baseline topics did not become ready")

        for case_name, parameter_mode, expected_reason in FAULT_CASES:
            if case_name == "gello_conflict":
                changed, parameter_reason = node.set_fault_mode("none")
                node.start_gello_conflict()
            else:
                changed, parameter_reason = node.set_fault_mode(parameter_mode)
            status_seen = node.wait_for(
                lambda: node.latest_status_reason() == expected_reason,
                3.0,
            )
            observed_reason = node.latest_status_reason()

            node.pump(0.20)
            quiet_raw_start = len(node.raw)
            quiet_accepted_start = len(node.accepted)
            settle_start = time.monotonic()
            node.pump(0.90)
            quiet_raw_end = len(node.raw)
            quiet_accepted_end = len(node.accepted)
            final_commanded = [
                sample for sample in node.commanded if sample[0] >= settle_start + 0.65
            ]
            commanded_span = _max_span(final_commanded)

            measured_before_recovery = node.measured[-1][1]
            recovery_raw_index = len(node.raw)
            if case_name == "gello_conflict":
                node.stop_gello_conflict()
                node.pump(0.40)
            else:
                recovered, recovery_reason = node.set_fault_mode("none")
                if not recovered:
                    parameter_reason = f"recovery_failed:{recovery_reason}"
            recovery_seen = node.wait_for(
                lambda: len(node.raw) > recovery_raw_index,
                3.0,
            )
            recovery_error = None
            if recovery_seen:
                recovery_error = _max_error(
                    node.raw[recovery_raw_index][1], measured_before_recovery
                )
            node.pump(0.20)

            raw_quiet = quiet_raw_start == quiet_raw_end
            accepted_quiet = quiet_accepted_start == quiet_accepted_end
            controller_settled = (
                commanded_span is not None and commanded_span < 5e-4
            )
            recovery_reanchored = (
                recovery_error is not None and recovery_error < 0.01
            )
            passed = all(
                (
                    changed,
                    status_seen,
                    raw_quiet,
                    accepted_quiet,
                    controller_settled,
                    recovery_seen,
                    recovery_reanchored,
                )
            )
            results.append(
                {
                    "case": case_name,
                    "expected_reason": expected_reason,
                    "observed_reason": observed_reason,
                    "parameter_change_ok": changed,
                    "parameter_reason": parameter_reason,
                    "status_seen": status_seen,
                    "raw_quiet": raw_quiet,
                    "accepted_quiet": accepted_quiet,
                    "controller_command_span_rad": commanded_span,
                    "controller_settled": controller_settled,
                    "recovery_seen": recovery_seen,
                    "recovery_first_raw_error_rad": recovery_error,
                    "recovery_reanchored": recovery_reanchored,
                    "passed": passed,
                }
            )

        payload: dict[str, object] = {
            "generated_at_unix_s": time.time(),
            "all_passed": all(bool(result["passed"]) for result in results),
            "cases": results,
            "counts": {
                "raw": len(node.raw),
                "accepted": len(node.accepted),
                "measured": len(node.measured),
                "commanded": len(node.commanded),
                "status": len(node.status),
            },
        }
    finally:
        node.stop_gello_conflict()
        node.destroy_node()
        rclpy.try_shutdown()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return payload


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parsed = parser.parse_args(args)
    payload = run(parsed.output)
    print(json.dumps(payload, indent=2, sort_keys=True))
    raise SystemExit(0 if payload["all_passed"] else 1)


if __name__ == "__main__":
    main()
