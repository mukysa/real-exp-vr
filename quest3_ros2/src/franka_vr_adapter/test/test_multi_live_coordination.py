from queue import Empty
from types import SimpleNamespace
from threading import Lock
import json
import time

from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.fr3_ik.live_control import LeftLiveControl
from data_collection.fr3_ik.live_input import PairedQuestInput
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose
from franka_vr_adapter.multi_live_ik_node import MultiLiveIkNode


Q = (-0.11657, 0.42515, -0.66825, -1.97659, 0.82153, 1.84735, -1.95020)


class Subscription:
    dropped_records = 0

    def __init__(self):
        self.pending = []

    def get_nowait(self):
        if not self.pending:
            raise Empty
        return self.pending.pop(0)


def rig():
    sides = ("left", "right")
    cores = {
        side: LeftLiveControl(
            Kinematics(side), side=side, clock_id="test",
            position_scale=1.0, max_tcp_offset_m=0.0,
            measured_timeout_s=1.0, input_timeout_s=2.0,
            relaxed_motion_gates=True,
        ) for side in sides
    }
    base = time.monotonic() - 0.5
    outputs = []
    harness = SimpleNamespace(
        mode="duo", cores=cores, input_timeout_s=2.0, left_gripper_enabled=False,
        y_blocked_until_release=False,
        _adapter_lock=Lock(), adapter_faults={},
        input=PairedQuestInput("duo"), subscription=Subscription(),
        _sync_measured=lambda: None,
        _publish_result=lambda side, result: outputs.append((side, result)),
    )
    harness._couple_faults = lambda: MultiLiveIkNode._couple_faults(harness)

    def frame(number, y=False):
        timestamp = base + number * 0.04
        for side in sides:
            sample = PoseSample(
                side, "controller", (0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0), True,
                clutch=y if side == "left" else False,
                frame_id=number,
            )
            harness.subscription.pending.append(
                ReceivedPose(sample, timestamp, number * 2 + (side == "right"), 1)
            )
        MultiLiveIkNode._drain(harness)

    return harness, cores, outputs, base, frame


def test_left_y_cannot_engage_only_one_of_two_arms():
    harness, cores, outputs, base, frame = rig()
    for number in (1, 2, 3):
        cores["left"].update_measured(Q, (0.0,) * 7, base + number * 0.04)
        frame(number)
    assert cores["left"].state == "READY"
    assert cores["right"].state == "WAITING"
    cores["left"].update_measured(Q, (0.0,) * 7, base + 0.16)
    frame(4, y=True)
    assert cores["left"].state == "READY"
    assert cores["right"].state == "WAITING"
    assert outputs == []
    cores["right"].update_measured(Q, (0.0,) * 7, base + 0.20)
    for core in cores.values():
        core.update_measured(Q, (0.0,) * 7, base + 0.20)
    frame(5, y=True)
    assert {core.state for core in cores.values()} == {"READY"}
    assert outputs == []
    frame(6, y=False)
    for core in cores.values():
        core.update_measured(Q, (0.0,) * 7, base + 0.28)
    frame(9, y=True)
    assert {core.state for core in cores.values()} == {"FOLLOWING"}


def test_left_y_engages_both_when_both_ready():
    harness, cores, outputs, base, frame = rig()
    for number in (1, 2, 3):
        for core in cores.values():
            core.update_measured(Q, (0.0,) * 7, base + number * 0.04)
        frame(number)
    assert {core.state for core in cores.values()} == {"READY"}
    for core in cores.values():
        core.update_measured(Q, (0.0,) * 7, base + 0.16)
    frame(4, y=True)
    assert {core.state for core in cores.values()} == {"FOLLOWING"}
    assert {side for side, _ in outputs} == {"left", "right"}


def test_one_adapter_stop_halts_both_and_requires_restart():
    harness, cores, outputs, base, frame = rig()
    for number in (1, 2, 3):
        for core in cores.values():
            core.update_measured(Q, (0.0,) * 7, base + number * 0.04)
        frame(number)
    for core in cores.values():
        core.update_measured(Q, (0.0,) * 7, base + 0.16)
    frame(4, y=True)
    assert {core.state for core in cores.values()} == {"FOLLOWING"}
    published_before = len(outputs)
    MultiLiveIkNode._on_adapter_status(
        harness,
        "right",
        SimpleNamespace(data=json.dumps({"state": "STOP", "reason": "robot_state_stale"})),
    )
    assert harness.adapter_faults == {"right": "robot_state_stale"}
    frame(5, y=False)
    assert {core.state for core in cores.values()} == {"FAULT"}
    assert len(outputs) == published_before
    for core in cores.values():
        core.update_measured(Q, (0.0,) * 7, base + 0.40)
    frame(10, y=True)
    assert {core.state for core in cores.values()} == {"FAULT"}
    assert len(outputs) == published_before
