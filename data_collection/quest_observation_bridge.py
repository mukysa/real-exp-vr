"""Quest left-arm bridge with state-only observations before and between Y takes.

This node only subscribes to ROS state/action topics and publishes ZMQ samples.
It never publishes FR3 commands. During READY/WAITING it pairs the measured
joint state with itself, producing a zero action until a real Y takeover.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from franka_lerobot_data_bridge.lerobot_data_bridge import LeRobotDataBridge

from data_collection.quest_action_policy import use_measured_hold


class QuestObservationBridge(LeRobotDataBridge):
    def __init__(self) -> None:
        self._quest_source_state: str | None = None
        self._accepted_action_seen = False
        self._last_action_mode: str | None = None
        super().__init__()
        if self.teleop_source != "quest" or self.arm_mode != "left" or self.deployment_mode:
            raise ValueError("Quest observation bridge requires Quest left collection mode")
        self.create_subscription(
            String, "/left/vr/source_status", self._on_source_status, 10
        )

    def _on_source_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = status.get("state") if isinstance(status, dict) else None
        if state in {"READY", "WAITING", "FOLLOWING", "RETURNING", "FAULT"}:
            self._quest_source_state = state

    def _arm_action_sample(self, arm_name: str):
        accepted = super()._arm_action_sample(arm_name)
        if accepted is not None:
            self._accepted_action_seen = True
        if arm_name == "left" and use_measured_hold(
            self._quest_source_state, self._accepted_action_seen
        ):
            measured = self.latest_robot_arm_samples[arm_name]
            if measured is not None:
                self._log_action_mode("measured_hold")
                return measured
        self._log_action_mode("accepted_target")
        return accepted

    def _log_action_mode(self, mode: str) -> None:
        if mode != self._last_action_mode:
            self._last_action_mode = mode
            self.get_logger().info(
                "Quest bridge action source: " + mode
                + "; measured_hold is observation only and never commands the FR3"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QuestObservationBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
