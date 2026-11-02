"""Local DDS + ZMQ integration; no cameras, controllers or robot connections."""
import json
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import zmq

from data_collection.meta_quest.record_control import RecorderControl
from data_collection.quest_record_relay import QuestRecordRelay


def test_dds_events_and_status_round_trip_with_no_camera_or_pose_stream():
    context = Context()
    rclpy.init(args=[], context=context)
    relay = QuestRecordRelay(context=context, parameter_overrides=[Parameter("control_port", value=0)])
    peer = Node("hand_arm_test_client", context=context)
    source = peer.create_publisher(String, "/left/vr/source_status", 10)
    requests = peer.create_publisher(String, "/left/vr/record_event", 10)
    statuses = []
    peer.create_subscription(String, "/left/vr/record_status",
                             lambda msg: statuses.append(json.loads(msg.data)), 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(relay)
    executor.add_node(peer)
    transport = zmq.Context()
    control = RecorderControl(transport, relay.socket.getsockopt_string(zmq.LAST_ENDPOINT))
    events = []

    def pump_until(condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            events.extend(control.drain())
        assert condition(), "local control transport did not reach the expected state"

    def request(sequence, operation="toggle"):
        return {"session": "test-session", "sequence": sequence, "operation": operation,
                "recorder_id": control.recorder_id, "revision": control.revision,
                "expires_at": time.time() + 0.9}

    try:
        pump_until(lambda: source.get_subscription_count() > 0 and requests.get_subscription_count() > 0)
        control.publish(force=True)
        pump_until(lambda: any(s.get("recorder_id") == control.recorder_id for s in statuses))
        source.publish(String(data=json.dumps({
            "hand_arm": True, "state": "FOLLOWING", "right_control_fresh": True,
            "generated_unix_s": time.time(), "record_session": "test-session"})))
        pump_until(lambda: control.source is not None)
        assert control.following()

        first = request(1)
        requests.publish(String(data=json.dumps(first)))
        pump_until(lambda: len(events) == 1)
        assert control.validate(events.pop()) is None
        control.begin("s", first)
        control.finish("recording", "completed", "Recording started.")
        pump_until(lambda: any(s.get("state") == "recording" for s in statuses))

        # Repeated DDS delivery cannot apply the same request twice.
        requests.publish(String(data=json.dumps(first)))
        pump_until(lambda: len(events) == 1)
        assert control.validate(events.pop()) is not None

        discard = request(2, "discard")
        requests.publish(String(data=json.dumps(discard)))
        pump_until(lambda: len(events) == 1)
        assert control.validate(events.pop()) is None
        control.begin("d", discard)
        control.finish("idle", "completed", "Current episode discarded.")
        pump_until(lambda: any(s.get("event_id") == ["test-session", 2]
                               and s.get("state") == "idle" for s in statuses))

        # An unavailable recorder never accumulates button presses for replay.
        relay.status_at = time.monotonic() - 2
        relay._heartbeat()
        pump_until(lambda: any(s.get("state") == "unavailable" for s in statuses))
        requests.publish(String(data=json.dumps(request(3))))
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            events.extend(control.drain())
        control.publish(force=True)
        pump_until(lambda: time.monotonic() - relay.status_at < 1)
        assert not events
    finally:
        control.close()
        transport.term()
        executor.shutdown()
        peer.destroy_node()
        relay.destroy_node()
        context.shutdown()
