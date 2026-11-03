"""One OpenVR stream and selected FR3 strict-IK sources with opt-in hand-arm input."""

from __future__ import annotations

from dataclasses import replace
import json
import re
from pathlib import Path
from queue import Empty
import sys
from threading import Lock
import time
import uuid

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


def _repository_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "data_collection/fr3_ik/live_control.py").is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return parent
    raise ImportError("Build franka_vr_adapter with --symlink-install")


ROOT = _repository_root()

from data_collection.fr3_ik.interfaces import ik_result_to_dict  # noqa: E402
from data_collection.fr3_ik.kinematics import Kinematics  # noqa: E402
from data_collection.fr3_ik.live_control import LeftLiveControl  # noqa: E402
from data_collection.meta_quest.hand_arm_input import HandArmInput
from data_collection.fr3_ik.live_input import PairedQuestInput  # noqa: E402
from data_collection.fr3_ik.mount_calibration import load_tabletop_calibration  # noqa: E402
from data_collection.meta_quest.receiver import PoseReceiver  # noqa: E402
from data_collection.meta_quest.gripper_mapping import trigger_to_open_fraction  # noqa: E402


class MultiLiveIkNode(Node):
    def __init__(self) -> None:
        super().__init__("quest_live_ik")
        defaults = (
            ("mode", "left"),
            ("left_gripper_enabled", False),
            ("hand_arm", False),
            ("trigger_press", 0.75),
            ("trigger_release", 0.25),
            ("pose_host", "127.0.0.1"),
            ("pose_port", 8001),
            ("position_scale", 1.0),
            ("max_tcp_offset_m", 0.0),
            ("pose_shaper_bandwidth", 5.0),
            ("measured_timeout_s", 1.0),
            ("input_timeout_s", 2.0),
            ("mount_calibration", str(ROOT / "calibration/matrix.md")),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in {"left", "right", "duo"}:
            raise ValueError("mode must be left, right or duo")
        self.sides = ("left", "right") if self.mode == "duo" else (self.mode,)
        self.left_gripper_enabled = bool(self.get_parameter("left_gripper_enabled").value)
        self.hand_arm = bool(self.get_parameter("hand_arm").value)
        if self.hand_arm and (self.mode != "left" or self.left_gripper_enabled):
            raise ValueError("hand_arm requires left arm without gripper")
        if self.left_gripper_enabled and self.mode != "left":
            raise ValueError("left gripper control currently requires left mode")
        if self.left_gripper_enabled:
            self.gripper_command_publisher = self.create_publisher(
                Float32, "/left/gripper/gripper_client/target_gripper_width_percent", 10
            )
            self.gripper_record_publisher = self.create_publisher(
                Float32, "/left/vr/gripper_open_fraction", 10
            )
            self.gripper_target = 1.0  # The gripper manager homes open at startup.
            self.last_gripper_command_s = 0.0
            self.last_gripper_record_s = 0.0
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        clock_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        calibrations, _, _ = load_tabletop_calibration(
            Path(str(self.get_parameter("mount_calibration").value))
        )
        self.cores = {
            side: LeftLiveControl(
                Kinematics(side),
                side=side,
                clock_id=clock_id,
                position_scale=float(self.get_parameter("position_scale").value),
                max_tcp_offset_m=float(self.get_parameter("max_tcp_offset_m").value),
                pose_shaper_bandwidth=float(self.get_parameter("pose_shaper_bandwidth").value),
                measured_timeout_s=float(self.get_parameter("measured_timeout_s").value),
                input_timeout_s=self.input_timeout_s,
                relaxed_motion_gates=True,
                max_source_translation_step_m=2.0,
                max_source_rotation_step_rad=np.pi,
                ik_optimizer_tolerance=1e-6,
                world_from_base=calibrations[side],
            )
            for side in self.sides
        }
        self.result_publishers = {
            side: self.create_publisher(String, f"/{side}/vr/ik_result", 10)
            for side in self.sides
        }
        self.status_publishers = {
            side: self.create_publisher(String, f"/{side}/vr/source_status", 10)
            for side in self.sides
        }
        self._measured_lock = Lock()
        self._measured: dict[str, tuple[list[float], list[float], float]] = {}
        self._measured_errors: dict[str, str] = {}
        self._adapter_lock = Lock()
        self.adapter_faults: dict[str, str] = {}
        self._measured_group = MutuallyExclusiveCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        for side in self.sides:
            self.create_subscription(
                JointState, f"/{side}/joint_states",
                lambda message, arm=side: self._on_measured(arm, message),
                10, callback_group=self._measured_group,
            )
            self.create_subscription(
                String, f"/{side}/vr/status",
                lambda message, arm=side: self._on_adapter_status(arm, message),
                10, callback_group=self._measured_group,
            )
        self.receiver = PoseReceiver(
            str(self.get_parameter("pose_host").value),
            int(self.get_parameter("pose_port").value),
        )
        self.subscription = self.receiver.subscribe(256)
        self.receiver.start()
        self.input = (HandArmInput(
            timeout_s=self.input_timeout_s,
            trigger_press=float(self.get_parameter("trigger_press").value),
            trigger_release=float(self.get_parameter("trigger_release").value),
        ) if self.hand_arm else PairedQuestInput(self.mode))
        if self.hand_arm:
            self.record_session = uuid.uuid4().hex
            self.record_sequence = 0
            self.record_status = None
            self.record_status_at = 0.0
            self.input_generation = 0
            self.record_event_publisher = self.create_publisher(String, "/left/vr/record_event", 10)
            self.create_subscription(String, "/left/vr/record_status", self._on_record_status,
                                     10, callback_group=self._control_group)
        self.y_blocked_until_release = False
        self.result_counts = {side: 0 for side in self.sides}
        self.last_status = {side: ("", "") for side in self.sides}
        self.create_timer(0.01, self._drain, callback_group=self._control_group)
        self.create_timer(0.5, self._status, callback_group=self._control_group)
        input_description = ("left pose/right buttons required" if self.hand_arm
                             else "both Touch controllers tracked")
        self.get_logger().info(
            f"Quest {self.mode} source ready: physical {'right B' if self.hand_arm else 'left Y'} toggles takeover/return; "
            f"{input_description}; left gripper enabled={self.left_gripper_enabled}"
        )

    @staticmethod
    def _canonical(name: str) -> str:
        return name.removeprefix("left_").removeprefix("right_")

    def _on_measured(self, side: str, message: JointState) -> None:
        if any(
            name.startswith(("left_", "right_")) and not name.startswith(f"{side}_")
            for name in message.name
        ):
            with self._measured_lock:
                self._measured_errors[side] = "measured side mismatch"
            return
        values = {
            self._canonical(name): (float(position), float(velocity))
            for name, position, velocity in zip(
                message.name,
                message.position,
                message.velocity if len(message.velocity) == len(message.position)
                else [0.0] * len(message.position),
            )
        }
        try:
            q = [values[f"fr3_joint{i}"][0] for i in range(1, 8)]
            dq = [values[f"fr3_joint{i}"][1] for i in range(1, 8)]
        except KeyError:
            with self._measured_lock:
                self._measured_errors[side] = "invalid measured joint names"
            return
        with self._measured_lock:
            self._measured[side] = (q, dq, time.monotonic())
            self._measured_errors.pop(side, None)

    def _sync_measured(self) -> None:
        with self._measured_lock:
            measured = dict(self._measured)
            errors = dict(self._measured_errors)
        for side, core in self.cores.items():
            if side in errors:
                core._fault(errors[side])
            elif side in measured:
                core.update_measured(*measured[side])

    def _on_adapter_status(self, side: str, message: String) -> None:
        if self.mode != "duo" and not getattr(self, "hand_arm", False):
            return
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            status = {"state": "STOP", "reason": "invalid adapter status"}
        if not isinstance(status, dict):
            status = {"state": "STOP", "reason": "invalid adapter status"}
        if status.get("state") == "STOP" or status.get("fault_latched") is True:
            with self._adapter_lock:
                self.adapter_faults.setdefault(side, str(status.get("reason", "adapter stopped")))

    def _couple_faults(self) -> bool:
        if getattr(self, "hand_arm", False):
            with self._adapter_lock:
                faults = dict(self.adapter_faults)
            if faults:
                self.cores["left"]._fault("left adapter stopped; restart required: " + faults["left"])
            return self.cores["left"].state == "FAULT"
        if self.mode != "duo":
            return any(core.state == "FAULT" for core in self.cores.values())
        with self._adapter_lock:
            adapter_faults = dict(self.adapter_faults)
        if adapter_faults:
            fault_side, reason = next(iter(adapter_faults.items()))
            for core in self.cores.values():
                core._fault(f"{fault_side} adapter stopped: {reason}; restart required")
            return True
        fault = next((side for side, core in self.cores.items() if core.state == "FAULT"), None)
        if fault is None:
            return False
        for side, core in self.cores.items():
            if side != fault and core.state != "FAULT":
                core._fault(f"peer {fault} stopped; both arms halted")
        return True

    def _publish_result(self, side: str, result) -> None:
        message = String()
        message.data = json.dumps(ik_result_to_dict(result), allow_nan=False, separators=(",", ":"))
        self.result_publishers[side].publish(message)
        self.result_counts[side] += 1

    def _drain(self) -> None:
        self._sync_measured()
        if self.subscription.dropped_records:
            for core in self.cores.values():
                core._fault("Quest input queue overflow; restart required")
        while True:
            try:
                self.input.offer(self.subscription.get_nowait())
            except Empty:
                break
        frame = self.input.take()
        now = time.monotonic()
        hand_arm = getattr(self, "hand_arm", False)
        if hand_arm:
            self._publish_record_events(now)
            # In the test mode a latched fault is never converted into a
            # recovery return by B. Recording end/discard remains available.
            if self.cores["left"].state == "FAULT":
                frame = None
        control = self.input.latest.get("right" if hand_arm else "left")
        if (
            control is None or now - control.received_monotonic_s > self.input_timeout_s
        ) and any(core.state in {"FOLLOWING", "RETURNING"} for core in self.cores.values()):
            for core in self.cores.values():
                core._fault("physical right buttons stale; restart required" if hand_arm else "physical left Y controller stale; both arms halted")
        if self._couple_faults() and hand_arm:
            frame = None
        if frame is not None:
            y = frame.left_y
            if hand_arm and not self.input.control_fresh(now):
                y = False
            if self.y_blocked_until_release:
                y = False
                if not self.input.left_y:
                    self.y_blocked_until_release = False
            states_before = {side: core.state for side, core in self.cores.items()}
            if self.mode == "duo" and y and len(set(states_before.values())) != 1:
                y = False
                self.y_blocked_until_release = self.input.left_y
                for core in self.cores.values():
                    core.reason = "Y rejected: selected arms are not in the same state"
            # A dual takeover must not energize just one side.  Reject the
            # shared Y edge until both measured states are fresh and still.
            if y and all(core.state == "READY" for core in self.cores.values()):
                if any(
                    not core._fresh_measured(now)
                    or core.measured_dq is None
                    or float(np.max(np.abs(core.measured_dq))) > 0.05
                    for core in self.cores.values()
                ):
                    y = False
                    self.y_blocked_until_release = self.input.left_y
                    for core in self.cores.values():
                        core.reason = "Y rejected: selected arms need fresh stationary state"
            updates = {}
            for side, core in self.cores.items():
                received = frame.poses[side]
                received = replace(received, sample=replace(received.sample, clutch=y))
                updates[side] = core.process(received)
            if hand_arm and frame.left_y:
                event = self.input.takeover_event
                core = self.cores["left"]
                age_ms = 1000 * (now - event.received_monotonic_s) if event else -1
                measured_age_ms = (1000 * (now - core.measured_stamp)
                                   if core.measured_stamp is not None else -1)
                speed = float(np.max(np.abs(core.measured_dq))) if core.measured_dq is not None else None
                reason = re.sub(r"\bY\b", "B", core.reason)
                self.get_logger().info(
                    f"HAND-ARM B frame={event.frame_id if event else None} "
                    f"input_age_ms={age_ms:.1f} gate_passed={y} "
                    f"state={states_before['left']}->{core.state} "
                    f"measured_age_ms={measured_age_ms:.1f} max_dq={speed} reason={reason}")
            if (
                self.mode == "duo" and y
                and all(state == "READY" for state in states_before.values())
                and not all(core.state == "FOLLOWING" for core in self.cores.values())
            ):
                for core in self.cores.values():
                    core._fault("dual takeover was not atomic; both arms halted")
            if not self._couple_faults():
                for side, update in updates.items():
                    if update.result is not None:
                        self._publish_result(side, update.result)
            if self.left_gripper_enabled and self.cores["left"].state == "FOLLOWING":
                try:
                    self.gripper_target = trigger_to_open_fraction(
                        frame.poses["left"].sample.grasp
                    )
                except ValueError:
                    self.cores["left"]._fault("invalid physical left trigger")
        self._sync_measured()
        for core in self.cores.values():
            core.tick(time.monotonic())
        self._couple_faults()
        if self.left_gripper_enabled:
            self._publish_gripper(time.monotonic())

    def _on_record_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        self.record_status, self.record_status_at = status, time.monotonic()
        display = (status.get("recorder_id"), status.get("revision"), status.get("state"),
                   status.get("result"), status.get("detail"))
        if display != getattr(self, "_last_record_display", None):
            self.get_logger().info("RECORDER " + json.dumps(status, ensure_ascii=False))
            self._last_record_display = display

    def _publish_record_events(self, now: float) -> None:
        if self.input_generation != self.input.generation:
            if self.input_generation and self.cores["left"].state in {"FOLLOWING", "RETURNING"}:
                self.cores["left"]._fault("right button stream changed or resumed after timeout; restart required")
            self.input_generation = self.input.generation
            self.record_session = uuid.uuid4().hex
        for event in self.input.pop_events(now):
            status = self.record_status
            label = "A" if event.operation == "toggle" else "trigger"
            trace = (f"HAND-ARM {label} frame={event.frame_id} "
                     f"input_age_ms={1000 * (now - event.received_monotonic_s):.1f}")
            if (not status or now - self.record_status_at > 1.0
                    or status.get("state") not in {"idle", "recording"}):
                self.get_logger().warning(
                    f"{trace} rejected: recorder unavailable or busy; "
                    f"state={status.get('state') if status else 'unavailable'}")
                continue
            self.record_sequence += 1
            payload = {
                "session": self.record_session, "sequence": self.record_sequence,
                "operation": event.operation, "recorder_id": status.get("recorder_id"),
                "revision": status.get("revision"),
                "expires_at": time.time() + max(0.0, 1.0 - (now - event.received_monotonic_s)),
            }
            self.record_event_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
            self.get_logger().info(
                f"{trace} sent: sequence={self.record_sequence} operation={event.operation} "
                f"recorder_state={status.get('state')}")

    def _publish_gripper(self, now: float) -> None:
        core = self.cores["left"]
        if core.state == "FOLLOWING" and now - self.last_gripper_command_s >= 0.1:
            left = self.input.latest.get("left")
            if left is not None and now - left.received_monotonic_s <= self.input_timeout_s:
                self.gripper_command_publisher.publish(Float32(data=self.gripper_target))
                self.last_gripper_command_s = now
        # The recording topic is separate: pre-Y observations need a target,
        # but publishing it to the hardware topic would move the gripper.
        if core.state != "FAULT" and now - self.last_gripper_record_s >= 0.05:
            self.gripper_record_publisher.publish(Float32(data=self.gripper_target))
            self.last_gripper_record_s = now

    def _status(self) -> None:
        now = time.monotonic()
        if self.hand_arm and now - self.record_status_at > 1.0:
            if getattr(self, "_last_record_display", None) != "unavailable":
                self.get_logger().warning("RECORDER unavailable: relay/recorder heartbeat missing; keyboard remains available")
                self._last_record_display = "unavailable"
        for side, core in self.cores.items():
            reason = core.reason
            if self.hand_arm:
                reason = re.sub(r"\bY\b", "B", reason.replace("left Y", "right B"))
            payload = {
                "mode": self.mode,
                "side": side,
                "state": core.state,
                "reason": reason,
                "result_count": self.result_counts[side],
                "button": "physical right B" if self.hand_arm else "physical left Y",
                "hand_arm": self.hand_arm,
                "right_control_fresh": self.input.control_fresh(now) if self.hand_arm else False,
                "generated_unix_s": time.time(),
                "record_session": self.record_session if self.hand_arm else None,
                "left_y_pressed": False if self.hand_arm else self.input.left_y,
                "right_b_pressed": self.input.right_b,
                "tracked_controllers": {
                    hand: hand in self.input.latest
                    and self.input.latest[hand].sample.tracked
                    and now - self.input.latest[hand].received_monotonic_s <= self.input_timeout_s
                    for hand in ("left", "right")
                },
                "hardware_output": "strict IKResult only; adapter and controller remain separate gates",
                "position_scale": core.position_scale,
                "max_tcp_offset_m": core.max_tcp_offset_m if core.max_tcp_offset_m > 0 else None,
                "translation_frame": core.translation_frame,
                "left_gripper_enabled": self.left_gripper_enabled,
            }
            if self.hand_arm:
                payload["recorder"] = (self.record_status if now - self.record_status_at <= 1.0
                                       else {"state": "unavailable", "detail": "recorder heartbeat missing"})
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self.status_publishers[side].publish(message)
            current = (core.state, reason)
            if current != self.last_status[side]:
                self.get_logger().info(f"{side} {current[0]}: {current[1]}")
                self.last_status[side] = current

    def destroy_node(self) -> None:
        self.receiver.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiLiveIkNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
