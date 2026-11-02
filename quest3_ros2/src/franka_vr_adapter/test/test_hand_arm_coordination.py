"""Run the actual node drain and IK core without creating a ROS or robot node."""
from dataclasses import replace
import json
from queue import Queue
from threading import Lock
import time
from types import SimpleNamespace

from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.fr3_ik.live_control import LeftLiveControl
from data_collection.meta_quest.hand_arm_input import HandArmInput
from data_collection.meta_quest.protocol import PoseSample, parse_pose_line
from data_collection.meta_quest.receiver import ReceivedPose
from franka_vr_adapter.multi_live_ik_node import MultiLiveIkNode


Q = (-0.11657, 0.42515, -0.66825, -1.97659, 0.82153, 1.84735, -1.95020)


def rig():
    core = LeftLiveControl(Kinematics("left"), clock_id="test", relaxed_motion_gates=True,
                           measured_timeout_s=1.0, input_timeout_s=2.0)
    queue = Queue()
    queue.dropped_records = 0
    outputs, events = [], []
    logger = SimpleNamespace(info=lambda _: None, warning=lambda _: None)
    harness = SimpleNamespace(
        mode="left", hand_arm=True, left_gripper_enabled=False,
        input_timeout_s=2.0, cores={"left": core}, input=HandArmInput(),
        subscription=queue, _sync_measured=lambda: None,
        y_blocked_until_release=False, _adapter_lock=Lock(), adapter_faults={},
        _publish_result=lambda side, result: outputs.append(result),
        get_logger=lambda: logger, input_generation=0,
        record_session="initial", record_sequence=0,
        record_status={"state": "recording", "recorder_id": "test", "revision": 1},
        record_status_at=time.monotonic(),
        record_event_publisher=SimpleNamespace(publish=lambda msg: events.append(json.loads(msg.data))),
    )
    harness._couple_faults = lambda: MultiLiveIkNode._couple_faults(harness)
    harness._publish_record_events = lambda now: MultiLiveIkNode._publish_record_events(harness, now)
    base = time.monotonic() - 0.8

    def frame(number, *, b=False, a=False, trigger=0.0, left=True, buttons=True):
        timestamp = base + number * 0.04
        core.update_measured(Q, (0.0,) * 7, timestamp)
        if left:
            # All left controls are held throughout: they must have no effect.
            sample = PoseSample("left", "controller", (0.0, 1.0, 0.0),
                                (0.0, 0.0, 0.0, 1.0), True,
                                grasp=1.0, clutch=True, record=True, frame_id=number)
            queue.put(ReceivedPose(sample, timestamp, number * 2, 1))
        if buttons:
            sample = parse_pose_line(f"right buttons|f={number}: {trigger},{int(b)},{int(a)}")
            queue.put(ReceivedPose(sample, timestamp, number * 2 + 1, 1))
        MultiLiveIkNode._drain(harness)

    for number in (1, 2, 3):
        frame(number)
    assert core.state == "READY"
    return harness, core, outputs, events, frame


def test_b_uses_measured_anchor_returns_and_reengages_with_all_left_buttons_held():
    harness, core, outputs, events, frame = rig()
    frame(4, b=True)
    assert core.state == "FOLLOWING"
    assert outputs[-1].command.joints.positions_rad == Q
    frame(5, b=True)
    assert core.state == "FOLLOWING"
    frame(6)
    frame(9, b=True)
    assert core.state == "READY"
    frame(10)
    frame(14, b=True)
    assert core.state == "FOLLOWING"
    assert not events
    assert not harness.left_gripper_enabled


def test_record_events_work_without_left_pose_and_do_not_change_arm_state():
    harness, core, outputs, events, frame = rig()
    frame(4, b=True)
    count = len(outputs)
    frame(5, a=True, left=False)
    assert events[-1]["operation"] == "toggle"
    frame(6, left=False)
    frame(7, a=True, trigger=1.0, left=False)
    assert [event["operation"] for event in events] == ["toggle", "discard"]
    assert core.state == "FOLLOWING" and len(outputs) == count


def test_right_input_loss_latches_fault_and_b_cannot_recover_it():
    harness, core, outputs, events, frame = rig()
    frame(4, b=True)
    harness.input._right = replace(harness.input._right, received_monotonic_s=time.monotonic() - 3)
    harness.input.latest["right"] = harness.input._right
    frame(5, buttons=False)
    assert core.state == "FAULT"
    count = len(outputs)
    frame(6, b=True, a=True)
    frame(7)
    frame(12, b=True, a=True)
    assert core.state == "FAULT" and len(outputs) == count
    assert events[-1]["operation"] == "toggle"
