from __future__ import annotations

import json
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest

from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.raw_recorder import PoseRecordingWriter, main as record_main
from data_collection.meta_quest.receiver import ReceivedPose
from data_collection.meta_quest.replay import (
    RecordingFormatError,
    load_recording,
    replay_poses,
)


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_background_recorder_signals_finalize_all_poses(tmp_path, stop_signal):
    path = tmp_path / "signals.jsonl"
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    # Reproduce the ignored SIGINT inherited from a Bash background job.
    command = (
        "import signal,runpy; signal.signal(signal.SIGINT,signal.SIG_IGN); "
        "runpy.run_module('data_collection.meta_quest.raw_recorder',run_name='__main__')"
    )
    process = subprocess.Popen(
        [sys.executable, "-c", command, "--output", str(path),
         "--port", str(port), "--no-keep-awake"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while True:
            try:
                connection = socket.create_connection(("127.0.0.1", port), timeout=0.2)
                break
            except OSError:
                assert process.poll() is None
                assert time.monotonic() < deadline
                time.sleep(0.02)
        with connection:
            connection.sendall(
                b"Right controller:, 0.1, 0.2, 0.3, 0, 0, 0, 1, 0.0, 0, 0\n" * 4
            )
        partial = path.with_name(path.name + ".partial")
        while len(partial.read_text().splitlines()) < 5:
            assert time.monotonic() < deadline
            time.sleep(0.02)
        process.send_signal(stop_signal)
        # A second interrupt during receiver shutdown must not abort saving.
        time.sleep(0.02)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        recording = load_recording(path)
        assert len(recording.poses) == 4
        assert recording.lossless
        assert not partial.exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


def _received(sequence: int, side: str, received_at: float, connection_id: int = 1):
    return ReceivedPose(
        sample=PoseSample(
            side=side,
            source="controller",
            position=(float(sequence), 0.2, 0.3),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            tracked=True,
            grasp=0.25,
            clutch=sequence % 2 == 0,
            record=False,
            layout="controller_compact_controls_v1",
        ),
        received_monotonic_s=received_at,
        sequence=sequence,
        connection_id=connection_id,
    )


def _write_recording(path, poses, dropped: int = 0):
    with PoseRecordingWriter(path, {"project_commit": "test"}) as writer:
        for pose in poses:
            writer.write_pose(pose)
        writer.complete({
            "receiver_pose_records": len(poses) + dropped,
            "rejected_pose_records": 0,
            "subscriber_dropped_records": dropped,
            "accepted_connections": 1,
            "duration_s": 1.0,
        })


def test_recording_round_trip_preserves_order_and_fields(tmp_path) -> None:
    path = tmp_path / "poses.jsonl"
    expected = (
        _received(10, "right", 5.0, 3),
        _received(11, "left", 5.01, 3),
        _received(12, "left", 5.02, 4),
        _received(13, "right", 5.03, 4),
    )

    _write_recording(path, expected)
    recording = load_recording(path)

    assert recording.poses == expected
    assert recording.lossless is True
    assert recording.summary["pose_records"] == 4
    assert path.exists()
    assert not path.with_name(path.name + ".partial").exists()


def test_identical_pose_samples_remain_distinct_records(tmp_path) -> None:
    path = tmp_path / "still.jsonl"
    first = _received(1, "right", 10.0)
    second = ReceivedPose(first.sample, 10.02, 2, 1)

    _write_recording(path, (first, second))

    assert [pose.sequence for pose in load_recording(path).poses] == [1, 2]


def test_replay_scales_original_intervals(tmp_path) -> None:
    path = tmp_path / "timed.jsonl"
    _write_recording(path, (_received(1, "right", 4.0), _received(2, "right", 4.2)))
    sleeps = []

    replayed = list(replay_poses(load_recording(path), speed=2.0, sleep_fn=sleeps.append))

    assert [pose.sequence for pose in replayed] == [1, 2]
    assert sleeps == pytest.approx([0.1])


def test_lossy_recording_is_explicitly_marked(tmp_path) -> None:
    path = tmp_path / "lossy.jsonl"
    _write_recording(path, (_received(1, "right", 1.0),), dropped=2)

    assert load_recording(path).lossless is False


@pytest.mark.parametrize(
    "records, message",
    [
        ([{"record_type": "pose"}], "first record"),
        ([{"record_type": "metadata", "schema_version": 99}], "schema_version"),
        ([{"record_type": "metadata", "schema_version": 1}], "completed summary"),
    ],
)
def test_replay_rejects_incomplete_or_unsupported_files(tmp_path, records, message) -> None:
    path = tmp_path / "bad.jsonl"
    path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")

    with pytest.raises(RecordingFormatError, match=message):
        load_recording(path)


def test_replay_rejects_sequence_regression(tmp_path) -> None:
    path = tmp_path / "bad-sequence.jsonl"
    _write_recording(path, (_received(2, "right", 1.0), _received(1, "left", 1.1)))

    with pytest.raises(RecordingFormatError, match="sequence is not increasing"):
        load_recording(path)


def test_writer_refuses_to_overwrite_completed_capture(tmp_path) -> None:
    path = tmp_path / "existing.jsonl"
    _write_recording(path, (_received(1, "right", 1.0),))

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        with PoseRecordingWriter(path, {}):
            pass


def test_raw_recorder_cli_captures_a_local_tcp_stream(tmp_path) -> None:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    path = tmp_path / "tcp-capture.jsonl"
    errors = []

    def run_recorder() -> None:
        try:
            record_main([
                "--port", str(port),
                "--output", str(path),
                "--duration-s", "0.5",
                "--no-keep-awake",
            ])
        except BaseException as exc:  # surfaced in the test thread below
            errors.append(exc)

    thread = threading.Thread(target=run_recorder)
    thread.start()
    deadline = time.monotonic() + 1.0
    while True:
        try:
            connection = socket.create_connection(("127.0.0.1", port))
            break
        except ConnectionRefusedError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.01)
    with connection:
        pair = (
            b"Right controller:, 0.1, 0.2, 0.3, 0, 0, 0, 1, 0, 0, 0\n"
            b"Left controller:, -0.1, 0.2, 0.3, 0, 0, 0, 1, 0, 0, 0\n"
        )
        connection.sendall(pair * 100)
    thread.join(timeout=2.0)

    assert not thread.is_alive()
    assert errors == []
    recording = load_recording(path)
    assert [pose.sample.side for pose in recording.poses[:4]] == [
        "right", "left", "right", "left"
    ]
    assert [pose.sequence for pose in recording.poses] == list(range(1, 201))
    assert recording.lossless is True
    assert recording.summary["receiver_pose_records"] == 200
