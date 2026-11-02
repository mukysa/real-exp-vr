"""Hand-arm DDS <-> local recorder JSON relay; independent of cameras."""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import zmq


class QuestRecordRelay(Node):
    def __init__(self, **kwargs):
        super().__init__("quest_record_relay", **kwargs)
        self.declare_parameter("control_port", 5559)
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.ROUTER)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.ROUTER_MANDATORY, 1)
        self.socket.setsockopt(zmq.SNDHWM, 32)
        self.socket.setsockopt(zmq.RCVHWM, 32)
        self.socket.bind(f"tcp://127.0.0.1:{int(self.get_parameter('control_port').value)}")
        self.identity = None
        self.status = None
        self.status_at = 0.0
        self.publisher = self.create_publisher(String, "/left/vr/record_status", 10)
        self.create_subscription(String, "/left/vr/source_status", self._source, 10)
        self.create_subscription(String, "/left/vr/record_event", self._event, 10)
        self.create_timer(0.02, self._poll)
        self.create_timer(0.2, self._heartbeat)

    def _send(self, payload):
        if self.identity is None:
            return
        try:
            self.socket.send_multipart([self.identity, json.dumps(payload, allow_nan=False).encode()], zmq.NOBLOCK)
        except (zmq.ZMQError, ValueError):
            pass

    @staticmethod
    def _decode(message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return None
        return payload if isinstance(payload, dict) else None

    def _source(self, message):
        status = self._decode(message)
        if status and status.get("hand_arm") is True:
            self._send({"kind": "source", "status": status})

    def _event(self, message):
        event = self._decode(message)
        if event and time.monotonic() - self.status_at <= 1.0:
            self._send({"kind": "event", "event": event})

    def _poll(self):
        for _ in range(64):
            try:
                parts = self.socket.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                return
            if len(parts) != 2:
                continue
            identity, data = parts
            try:
                status = json.loads(data)
            except (ValueError, UnicodeError):
                continue
            if not isinstance(status, dict) or status.get("kind") != "status":
                continue
            if (self.identity is not None and self.identity != identity
                    and time.monotonic() - self.status_at <= 1.0):
                # Never let two recorders share the same controller session.
                continue
            self.identity, self.status, self.status_at = identity, status, time.monotonic()
            self.publisher.publish(String(data=json.dumps(status)))

    def _heartbeat(self):
        if self.status is None or time.monotonic() - self.status_at > 1.0:
            self.publisher.publish(String(data=json.dumps({"state": "unavailable", "detail": "recorder heartbeat missing"})))

    def destroy_node(self):
        self.socket.close(0)
        self.zmq_context.term()
        super().destroy_node()


def main():
    rclpy.init()
    node = QuestRecordRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
