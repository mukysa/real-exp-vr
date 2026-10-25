"""ROS wrapper for the bounded left-arm Quest live-control core."""

from __future__ import annotations

import json
from pathlib import Path
from queue import Empty
import sys
from threading import Lock
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def _add_repository_root_to_path() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "data_collection/fr3_ik/live_control.py").is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return parent
    raise ImportError("Build franka_vr_adapter with --symlink-install from the project checkout")


_add_repository_root_to_path()

from data_collection.fr3_ik.interfaces import ik_result_to_dict  # noqa: E402
from data_collection.fr3_ik.kinematics import Kinematics  # noqa: E402
from data_collection.fr3_ik.live_control import LeftLiveControl  # noqa: E402
from data_collection.meta_quest.receiver import PoseReceiver  # noqa: E402


class LeftLiveIkNode(Node):
    def __init__(self) -> None:
        super().__init__("left_quest_live_ik")
        for name, default in (
            ("pose_host", "127.0.0.1"),
            ("pose_port", 8001),
            ("position_scale", 0.03),
            ("max_tcp_offset_m", 0.005),
            ("pose_shaper_bandwidth", 5.0),
            ("measured_topic", "/left/joint_states"),
            ("ik_result_topic", "/left/vr/ik_result"),
            ("status_topic", "/left/vr/source_status"),
        ):
            self.declare_parameter(name, default)
        clock_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.core = LeftLiveControl(
            Kinematics("left"),
            clock_id=clock_id,
            position_scale=float(self.get_parameter("position_scale").value),
            max_tcp_offset_m=float(self.get_parameter("max_tcp_offset_m").value),
            pose_shaper_bandwidth=float(self.get_parameter("pose_shaper_bandwidth").value),
        )
        self.result_publisher = self.create_publisher(
            String, str(self.get_parameter("ik_result_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._measured_lock = Lock()
        self._latest_measured: tuple[list[float], list[float], float] | None = None
        self._measured_error: str | None = None
        self._measured_group = MutuallyExclusiveCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            JointState,
            str(self.get_parameter("measured_topic").value),
            self._on_measured,
            10,
            callback_group=self._measured_group,
        )
        self.receiver = PoseReceiver(
            str(self.get_parameter("pose_host").value),
            int(self.get_parameter("pose_port").value),
        )
        self.subscription = self.receiver.subscribe(128)
        self.receiver.start()
        self.result_count = 0
        self.last_status = ("", "")
        self.create_timer(0.01, self._drain, callback_group=self._control_group)
        self.create_timer(0.5, self._status, callback_group=self._control_group)
        workspace = (
            f"{self.core.max_tcp_offset_m * 1000.0:g} mm TCP cap"
            if self.core.max_tcp_offset_m > 0.0
            else "TCP radial cap disabled"
        )
        self.get_logger().info(
            f"Left Quest source ready: Y toggles takeover/return, "
            f"{workspace}, no gripper control"
        )

    @staticmethod
    def _canonical(name: str) -> str:
        return name.removeprefix("left_").removeprefix("right_")

    def _on_measured(self, message: JointState) -> None:
        values = {
            self._canonical(name): (float(position), float(velocity))
            for name, position, velocity in zip(
                message.name,
                message.position,
                message.velocity if len(message.velocity) == len(message.position) else [0.0] * len(message.position),
            )
        }
        try:
            q = [values[f"fr3_joint{i}"][0] for i in range(1, 8)]
            dq = [values[f"fr3_joint{i}"][1] for i in range(1, 8)]
        except KeyError:
            with self._measured_lock:
                self._measured_error = "invalid /left/joint_states names"
            return
        with self._measured_lock:
            self._latest_measured = (q, dq, time.monotonic())
            self._measured_error = None

    def _sync_measured(self) -> None:
        with self._measured_lock:
            measured = self._latest_measured
            error = self._measured_error
        if error is not None:
            self.core._fault(error)
        elif measured is not None:
            self.core.update_measured(*measured)

    def _publish_result(self, result) -> None:
        message = String()
        message.data = json.dumps(ik_result_to_dict(result), allow_nan=False, separators=(",", ":"))
        self.result_publisher.publish(message)
        self.result_count += 1

    def _drain(self) -> None:
        self._sync_measured()
        if self.subscription.dropped_records:
            self.core._fault("Quest input queue overflow; restart required")
        # IK can take longer than the controller's 30 Hz input period while
        # the hand is moving.  Always solve the newest pose so old samples do
        # not accumulate and later trip the adapter's input-age gate.  The
        # receiver still retains its lossless FIFO API for recording users.
        try:
            received = self.subscription.get_latest_nowait()
        except Empty:
            self.core.tick(time.monotonic())
            return
        update = self.core.process(received)
        if update.result is not None:
            self._publish_result(update.result)
        # The measured callback remains live in the second executor thread
        # while a strict IK solve is running.  Pull its newest sample before
        # evaluating the watchdog.
        self._sync_measured()
        self.core.tick(time.monotonic())

    def _status(self) -> None:
        payload = {
            "state": self.core.state,
            "reason": self.core.reason,
            "result_count": self.result_count,
            "hardware_output": "strict IKResult only; adapter and controller remain separate gates",
            "button": "left Y",
            "position_scale": self.core.position_scale,
            "max_tcp_offset_m": (
                self.core.max_tcp_offset_m if self.core.max_tcp_offset_m > 0.0 else None
            ),
            "tcp_workspace_limit_enabled": self.core.max_tcp_offset_m > 0.0,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)
        current = (self.core.state, self.core.reason)
        if current != self.last_status:
            self.get_logger().info(f"{current[0]}: {current[1]}")
            self.last_status = current

    def destroy_node(self) -> None:
        self.receiver.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeftLiveIkNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
