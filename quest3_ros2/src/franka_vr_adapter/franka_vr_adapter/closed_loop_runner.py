"""Measure the fake Quest command path and the controller timeout on adapter exit."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


EXPECTED_JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
Sample = tuple[float, tuple[str, ...], tuple[float, ...]]


class ClosedLoopRunner(Node):
    def __init__(self, namespace: str = "right") -> None:
        super().__init__(f"{namespace}_vr_closed_loop_runner")
        prefix = f"/{namespace}"
        self.expected_joints = tuple(f"{namespace}_fr3_joint{index}" for index in range(1, 8))
        self.raw: list[Sample] = []
        self.accepted: list[Sample] = []
        self.measured: list[Sample] = []
        self.commanded: list[Sample] = []
        self.create_subscription(
            JointState, f"{prefix}/vr/raw_joint_states", self._raw_callback, 100
        )
        self.create_subscription(
            JointState,
            f"{prefix}/vr/accepted_joint_states",
            self._accepted_callback,
            100,
        )
        self.create_subscription(
            JointState, f"{prefix}/joint_states", self._measured_callback, 100
        )
        self.create_subscription(
            JointState,
            f"{prefix}/franka/commanded_joint_states",
            self._commanded_callback,
            100,
        )

    @staticmethod
    def _sample(message: JointState) -> Sample:
        return (
            time.monotonic(),
            tuple(message.name),
            tuple(float(value) for value in message.position),
        )

    def _raw_callback(self, message: JointState) -> None:
        self.raw.append(self._sample(message))

    def _accepted_callback(self, message: JointState) -> None:
        self.accepted.append(self._sample(message))

    def _measured_callback(self, message: JointState) -> None:
        self.measured.append(self._sample(message))

    def _commanded_callback(self, message: JointState) -> None:
        self.commanded.append(self._sample(message))

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

    def raw_publishers(self) -> int:
        return self.count_publishers("/right/vr/raw_joint_states")


def _positions(samples: list[Sample]) -> list[tuple[float, ...]]:
    return [positions for _, _, positions in samples if len(positions) == 7]


def _all_finite(samples: list[Sample]) -> bool:
    return bool(samples) and all(
        len(positions) == 7
        and all(math.isfinite(value) for value in positions)
        for _, _, positions in samples
    )


def _joint_order_ok(samples: list[Sample]) -> bool:
    return bool(samples) and all(names in (EXPECTED_JOINTS, tuple(f"left_{name}" for name in EXPECTED_JOINTS), tuple(f"right_{name}" for name in EXPECTED_JOINTS)) for _, names, _ in samples)


def _max_span(samples: list[Sample]) -> float | None:
    values = _positions(samples)
    if len(values) < 2:
        return None
    return max(max(axis) - min(axis) for axis in zip(*values))


def _max_step(samples: list[Sample]) -> float | None:
    values = _positions(samples)
    if len(values) < 2:
        return None
    return max(
        max(abs(right - left) for left, right in zip(previous, current))
        for previous, current in zip(values, values[1:])
    )


def _max_gap(samples: list[Sample]) -> float | None:
    if len(samples) < 2:
        return None
    return max(right[0] - left[0] for left, right in zip(samples, samples[1:]))


def _best_sequence_match(left: list[Sample], right: list[Sample]) -> dict[str, object]:
    """Allow one boundary sample to be missed by either subscription."""
    best: tuple[float, int, int, int] | None = None
    for left_skip in range(min(3, len(left))):
        for right_skip in range(min(3, len(right))):
            count = min(len(left) - left_skip, len(right) - right_skip)
            if count <= 0:
                continue
            pair_errors = []
            for index in range(count):
                left_positions = left[left_skip + index][2]
                right_positions = right[right_skip + index][2]
                if len(left_positions) != 7 or len(right_positions) != 7:
                    pair_errors.append(math.inf)
                else:
                    pair_errors.append(
                        max(
                            abs(a - b)
                            for a, b in zip(left_positions, right_positions)
                        )
                    )
            error = max(pair_errors)
            candidate = (error, -count, left_skip, right_skip)
            if best is None or candidate < best:
                best = candidate
    if best is None:
        return {"matched_count": 0, "max_error_rad": None, "coverage": 0.0}
    error, negative_count, left_skip, right_skip = best
    count = -negative_count
    return {
        "matched_count": count,
        "max_error_rad": error,
        "coverage": count / max(len(left), len(right)),
        "left_skip": left_skip,
        "right_skip": right_skip,
    }


def _trace_metrics(path: Path) -> dict[str, object]:
    events = [
        json.loads(line) for line in path.read_text().splitlines() if line.strip()
    ]
    anchors = [event for event in events if event.get("event") == "initial_anchor"]
    raw = [event for event in events if event.get("event") == "raw"]
    accepted = [event for event in events if event.get("event") == "accepted"]
    requested = [event for event in events if event.get("event") == "requested"]
    measured = [event for event in events if event.get("event") == "measured"]

    first_raw_error = None
    if anchors and raw:
        first_raw_error = max(
            abs(float(a) - float(b))
            for a, b in zip(anchors[0]["positions"], raw[0]["positions"])
        )

    def trace_samples(items: list[dict[str, object]]) -> list[Sample]:
        return [
            (
                float(event["monotonic_time_s"]),
                tuple(str(name) for name in event.get("joint_names", EXPECTED_JOINTS)),
                tuple(float(value) for value in event["positions"]),
            )
            for event in items
        ]

    return {
        "counts": {
            "requested": len(requested),
            "raw": len(raw),
            "accepted": len(accepted),
            "measured": len(measured),
        },
        "first_raw_from_anchor_error_rad": first_raw_error,
        "raw_accepted": _best_sequence_match(
            trace_samples(raw), trace_samples(accepted)
        ),
        "requested_raw": _best_sequence_match(
            trace_samples(requested), trace_samples(raw)
        ),
    }


def _settle_time(
    samples: list[Sample], exit_time_s: float, tolerance: float
) -> float | None:
    after = [sample for sample in samples if sample[0] >= exit_time_s]
    if len(after) < 2:
        return None
    for index in range(len(after) - 1):
        tail_span = _max_span(after[index:])
        if tail_span is not None and tail_span <= tolerance:
            return after[index][0] - exit_time_s
    return None


def run(
    output_path: Path,
    trace_path: Path,
    ready_path: Path,
    duration_s: float,
    post_exit_duration_s: float,
    namespace: str = "right",
) -> dict[str, object]:
    rclpy.init()
    node = ClosedLoopRunner(namespace=namespace)
    try:
        ready = node.wait_for(
            lambda: len(node.raw) >= 5
            and len(node.accepted) >= 5
            and len(node.measured) >= 5
            and len(node.commanded) >= 5,
            10.0,
        )
        if not ready:
            raise RuntimeError("required fake-stack topics did not become ready")

        node.raw.clear()
        node.accepted.clear()
        node.measured.clear()
        node.commanded.clear()
        measurement_started_s = time.monotonic()
        node.pump(duration_s)
        pre_exit_counts = {
            "raw": len(node.raw),
            "accepted": len(node.accepted),
            "measured": len(node.measured),
            "commanded": len(node.commanded),
        }
        ready_path.parent.mkdir(parents=True, exist_ok=True)
        ready_path.write_text(
            json.dumps({"ready": True, "monotonic_time_s": time.monotonic()}) + "\n"
        )

        # DDS graph discovery can retain a dead publisher for several seconds.
        # The controller-facing fact we care about is that commands have stopped.
        command_stream_stopped = node.wait_for(
            lambda: bool(node.raw) and time.monotonic() - node.raw[-1][0] >= 0.4,
            20.0,
        )
        adapter_exit_observed_s = node.raw[-1][0] if node.raw else time.monotonic()
        post_exit_start_counts = {
            "raw": len(node.raw),
            "accepted": len(node.accepted),
            "measured": len(node.measured),
            "commanded": len(node.commanded),
        }
        if command_stream_stopped:
            node.pump(post_exit_duration_s)

        post_exit_counts = {
            name: len(samples) - post_exit_start_counts[name]
            for name, samples in (
                ("raw", node.raw),
                ("accepted", node.accepted),
                ("measured", node.measured),
                ("commanded", node.commanded),
            )
        }
        raw_accepted = _best_sequence_match(
            node.raw[: pre_exit_counts["raw"]],
            node.accepted[: pre_exit_counts["accepted"]],
        )
        measured_span = _max_span(node.measured[: pre_exit_counts["measured"]])
        commanded_span = _max_span(node.commanded[: pre_exit_counts["commanded"]])
        settle_time_s = _settle_time(node.commanded, adapter_exit_observed_s, 5e-4)
        trace = _trace_metrics(trace_path)

        checks = {
            "sample_counts_sufficient": min(pre_exit_counts.values()) >= 20,
            "raw_joint_order": _joint_order_ok(node.raw[: pre_exit_counts["raw"]]),
            "accepted_joint_order": _joint_order_ok(
                node.accepted[: pre_exit_counts["accepted"]]
            ),
            "all_streams_finite": all(
                _all_finite(samples[: pre_exit_counts[name]])
                for name, samples in (
                    ("raw", node.raw),
                    ("accepted", node.accepted),
                    ("measured", node.measured),
                    ("commanded", node.commanded),
                )
            ),
            "raw_accepted_match": raw_accepted["coverage"] >= 0.98
            and raw_accepted["max_error_rad"] is not None
            and raw_accepted["max_error_rad"] <= 1e-12,
            "first_command_reanchored": trace["first_raw_from_anchor_error_rad"]
            is not None
            and trace["first_raw_from_anchor_error_rad"] <= 0.005,
            "measured_stream_continuous": _max_gap(
                node.measured[: pre_exit_counts["measured"]]
            )
            is not None
            and _max_gap(node.measured[: pre_exit_counts["measured"]]) <= 0.1,
            "controller_reference_moved": commanded_span is not None
            and commanded_span >= 0.005,
            "adapter_command_stream_stopped": command_stream_stopped,
            "raw_stopped_after_exit": post_exit_counts["raw"] == 0,
            "accepted_stopped_after_exit": post_exit_counts["accepted"] == 0,
            "controller_reference_settled": settle_time_s is not None
            and settle_time_s <= 1.2,
        }
        payload: dict[str, object] = {
            "generated_at_unix_s": time.time(),
            "measurement_duration_s": duration_s,
            "post_exit_duration_s": post_exit_duration_s,
            "all_passed": all(checks.values()),
            "checks": checks,
            "pre_exit_counts": pre_exit_counts,
            "post_exit_counts": post_exit_counts,
            "shutdown_transition_counts": {
                name: post_exit_start_counts[name] - pre_exit_counts[name]
                for name in pre_exit_counts
            },
            "raw_accepted": raw_accepted,
            "trace": trace,
            "stream_metrics": {
                "raw_max_step_rad": _max_step(node.raw[: pre_exit_counts["raw"]]),
                "accepted_max_step_rad": _max_step(
                    node.accepted[: pre_exit_counts["accepted"]]
                ),
                "measured_max_step_rad": _max_step(
                    node.measured[: pre_exit_counts["measured"]]
                ),
                "measured_max_gap_s": _max_gap(
                    node.measured[: pre_exit_counts["measured"]]
                ),
                "measured_span_rad": measured_span,
                "commanded_span_rad": commanded_span,
                "controller_settle_time_after_adapter_exit_s": settle_time_s,
            },
            "dynamic_tracking_validated": bool(
                measured_span and measured_span >= 0.005
            ),
            "dynamic_tracking_limitation": (
                "The ROS mock_generic_system exposes a static measured joint state "
                "under this effort-controller setup. This run validates command "
                "transport, safety gating, topic continuity, and controller timeout, "
                "but cannot validate physical joint tracking dynamics."
            ),
            "measurement_started_monotonic_s": measurement_started_s,
            "adapter_exit_observed_monotonic_s": adapter_exit_observed_s,
            "raw_publisher_count_after_exit": node.raw_publishers(),
        }
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--post-exit-duration", type=float, default=2.0)
    parser.add_argument("--namespace", choices=("left", "right"), default="right")
    args = parser.parse_args()
    payload = run(
        output_path=args.output,
        trace_path=args.trace,
        ready_path=args.ready_file,
        duration_s=args.duration,
        post_exit_duration_s=args.post_exit_duration,
        namespace=args.namespace,
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    raise SystemExit(0 if payload["all_passed"] else 1)


if __name__ == "__main__":
    main()
