from __future__ import annotations

import socket
import time
from queue import Empty

import pytest

from data_collection.meta_quest.receiver import PoseReceiver


RIGHT = "Right controller:, 0.1, 0.2, 0.3, 0, 0, 0, 1, 0.0, 0, 0"
LEFT = "Left controller:, -0.1, 0.2, 0.3, 0, 0, 0, 1, 0.0, 0, 0"


def _wait_for_pose_records(receiver: PoseReceiver, count: int):
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        snapshot = receiver.snapshot()
        if snapshot.pose_records >= count:
            return snapshot
        time.sleep(0.01)
    return receiver.snapshot()


def test_sequence_only_advances_for_valid_pose_records() -> None:
    receiver = PoseReceiver()

    receiver._record_line(RIGHT)
    receiver._record_line("Right controller:, 1, 2, 3")
    receiver._record_line("Fist: Closed")
    receiver._record_line(LEFT)

    snapshot = receiver.snapshot()
    assert snapshot.pose_records == 2
    assert snapshot.rejected_pose_records == 1
    assert snapshot.poses[("controller", "right")].sequence == 1
    assert snapshot.poses[("controller", "left")].sequence == 2
    assert snapshot.poses[("controller", "right")].connection_id == 0


def test_identical_live_packets_still_advance_sequence() -> None:
    receiver = PoseReceiver()

    receiver._record_line(RIGHT)
    first = receiver.snapshot().poses[("controller", "right")]
    receiver._record_line(RIGHT)
    second = receiver.snapshot().poses[("controller", "right")]

    assert second.sequence == first.sequence + 1
    assert second.received_monotonic_s >= first.received_monotonic_s


def test_connection_id_is_stable_within_connection_and_changes_after_reconnect() -> None:
    with PoseReceiver(port=0) as receiver:
        with socket.create_connection(("127.0.0.1", receiver.port)) as connection:
            connection.sendall(f"{RIGHT}\n{LEFT}\n".encode())

        first_snapshot = _wait_for_pose_records(receiver, 2)
        right = first_snapshot.poses[("controller", "right")]
        left = first_snapshot.poses[("controller", "left")]
        assert right.connection_id == left.connection_id
        assert right.connection_id > 0

        with socket.create_connection(("127.0.0.1", receiver.port)) as connection:
            connection.sendall(f"{RIGHT}\n".encode())

        second_snapshot = _wait_for_pose_records(receiver, 3)
        reconnected = second_snapshot.poses[("controller", "right")]
        assert reconnected.connection_id > right.connection_id
        assert reconnected.sequence > left.sequence


def test_subscribers_receive_independent_ordered_streams_without_blocking() -> None:
    receiver = PoseReceiver()
    small = receiver.subscribe(max_queue_size=2)
    complete = receiver.subscribe(max_queue_size=4)

    receiver._record_line(RIGHT)
    receiver._record_line(LEFT)
    receiver._record_line(RIGHT)

    assert [small.get_nowait().sequence, small.get_nowait().sequence] == [1, 2]
    assert small.dropped_records == 1
    assert [complete.get_nowait().sequence for _ in range(3)] == [1, 2, 3]
    assert complete.dropped_records == 0
    assert receiver.snapshot().pose_records == 3

    receiver.unsubscribe(small)
    receiver.unsubscribe(complete)
    receiver._record_line(LEFT)
    with pytest.raises(Empty):
        complete.get_nowait()
