"""Verify that observability is bounded and cannot change call semantics."""

import gc
import json
import os
import signal
import struct
import weakref

import pytest

from data_collection.meta_quest import debug_recording as recording
from data_collection.meta_quest.debug_capture import Collector


def finish(recorder):
    recorder.close()
    recorder._thread.join(timeout=3)
    assert not recorder._thread.is_alive()
    result = []
    for name in ("events.jsonl.1", "events.jsonl"):
        path = recorder.output_dir / name
        if path.exists():
            result.extend(json.loads(line) for line in path.read_text().splitlines())
    return result


def test_call_forwards_kwargs_and_preserves_exception_identity(tmp_path):
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    channel = recording.DebugChannel("test", recorder)
    calls = []
    marker = RuntimeError("same exception")

    def success(value, *, extra):
        calls.append((value, extra))
        return value + extra

    assert channel.call("success", success, 3, extra=4) == 7
    assert calls == [(3, 4)]

    def fail():
        raise marker

    with pytest.raises(RuntimeError) as raised:
        channel.call("failure", fail)
    assert raised.value is marker
    records = finish(recorder)
    assert any(r.get("event") == "call_exception" and r["fields"]["stage"] == "failure" for r in records)
    summary = next(r for r in records if r.get("event") == "summary")
    assert summary["timings"]["test.success"]["count"] == 1


def test_call_preserves_exception_when_its_message_cannot_be_read(tmp_path):
    class BrokenMessage(RuntimeError):
        def __str__(self):
            raise RuntimeError("message conversion failed")

    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    channel = recording.DebugChannel("test", recorder)
    marker = BrokenMessage()

    def fail():
        raise marker

    with pytest.raises(BrokenMessage) as raised:
        channel.call("failure", fail)
    assert raised.value is marker
    records = finish(recorder)
    exception = next(r for r in records if r.get("event") == "call_exception")
    assert exception["fields"]["detail"] == "<BrokenMessage: message unavailable>"


def test_manifest_budget_also_covers_long_environment_and_arguments(tmp_path, monkeypatch):
    for index in range(64):
        monkeypatch.setenv(f"VK_TEST_{index}", "图" * 4096)
    monkeypatch.setattr(recording.sys, "argv", ["参" * 4096 for _ in range(64)])
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    finish(recorder)
    path = recorder.output_dir / "manifest.json"
    assert path.stat().st_size <= 65536
    manifest = json.loads(path.read_text())
    assert manifest["pid"] == os.getpid()
    assert manifest["snapshot_truncated"]


def test_debug_root_symlink_cannot_redirect_writes(tmp_path, monkeypatch):
    outside = tmp_path / "original-workspace"
    outside.mkdir()
    debug_root = tmp_path / "vr_debug"
    debug_root.symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(recording, "DEBUG_ROOT", debug_root)
    with pytest.raises(OSError):
        recording._prepare_run()
    assert not list(outside.iterdir())


def test_samples_detach_objects_and_retain_latest_with_valid_json(tmp_path):
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    channel = recording.DebugChannel("test", recorder)

    class NativeLike:
        def __repr__(self):
            return "native pose"

    source = NativeLike()
    ref = weakref.ref(source)
    channel.sample("pose", source=source, position=[float("nan"), float("inf"), 0])
    del source
    gc.collect()
    assert ref() is None
    for i in range(100):
        channel.sample("latest", frame_id=i)
    records = finish(recorder)
    poses = [r for r in records if r.get("key") == "pose"]
    assert poses[0]["fields"]["position"] == ["nan", "inf", 0]
    assert [r["fields"]["frame_id"] for r in records if r.get("key") == "latest"] == [99]


def test_log_rotation_and_event_storm_stay_bounded(tmp_path, monkeypatch):
    monkeypatch.setattr(recording, "MAX_FILE_BYTES", 2048)
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    channel = recording.DebugChannel("test", recorder)
    for _ in range(1000):
        channel.event("same_error", detail="repeated")
    for i in range(200):
        channel.event(f"event_{i}", detail="x" * 3000)
    records = finish(recorder)
    files = list(recorder.output_dir.glob("events.jsonl*"))
    assert len(files) <= 2
    assert all(path.stat().st_size <= 2048 for path in files)
    assert records
    assert any(r.get("suppressed_events", 0) >= 999 for r in records)


def test_diagnostic_storage_failure_is_noop(tmp_path, monkeypatch):
    monkeypatch.setattr(recording, "_INITIALIZED", False)
    monkeypatch.setattr(recording, "_RECORDER", None)
    monkeypatch.setattr(recording, "_CHANNELS", {})
    monkeypatch.delenv("VR_DEBUG_DISABLE", raising=False)
    bad_root = tmp_path / "file"
    bad_root.write_text("not a directory")
    monkeypatch.setattr(recording, "DEBUG_ROOT", bad_root)
    channel = recording.get_debug_recorder("test")
    assert channel.output_dir is None
    channel.event("ignored")
    channel.sample("ignored", value=1)
    assert channel.call("work", lambda: 42) == 42


def test_observer_reads_active_stage_without_writer_thread(tmp_path, monkeypatch):
    from data_collection.meta_quest import debug_capture

    monkeypatch.setattr(debug_capture, "DEBUG_ROOT", tmp_path)
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    token = (1, "test", "native_render")
    recorder.begin_call(token, recording._CLOCK())
    raw = (recorder.output_dir / "last-call.bin").read_bytes()
    seq, started, updated, component, stage, active, end = recording.CALL_STATE.unpack(raw)
    assert seq == end and not seq % 2
    assert active and stage.rstrip(b"\0") == b"native_render"
    collector = Collector(os.getpid(), recorder.output_dir, recording._start_token(os.getpid()), 0)
    assert collector.observe()
    sample = json.loads(b"".join(collector.resources.lines).decode().splitlines()[-1])
    assert sample["last_call"]["stage"] == "native_render"
    assert sample["last_call"]["call_active"]
    recorder.end_call(token, started)
    finish(recorder)


def test_nested_call_completion_restores_active_outer_stage(tmp_path):
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    outer = (1, "test", "monitor_scope")
    inner = (1, "test", "render")
    outer_started = recording._CLOCK()
    recorder.begin_call(outer, outer_started)
    inner_started = recording._CLOCK()
    recorder.begin_call(inner, inner_started)
    recorder.end_call(inner, inner_started)
    raw = (recorder.output_dir / "last-call.bin").read_bytes()
    seq, started, updated, component, stage, active, end = recording.CALL_STATE.unpack(raw)
    assert seq == end and not seq % 2
    assert component.rstrip(b"\0") == b"test"
    assert stage.rstrip(b"\0") == b"monitor_scope"
    assert active
    recorder.end_call(outer, outer_started)
    finish(recorder)


def test_signal_note_delegates_to_original_handler(tmp_path):
    recorder = recording._Recorder("test", run_dir=tmp_path, observer=False)
    original = signal.getsignal(signal.SIGINT)
    try:
        recorder.install_signal_notes()
        handler = signal.getsignal(signal.SIGINT)
        with pytest.raises(KeyboardInterrupt):
            handler(signal.SIGINT, None)
    finally:
        signal.signal(signal.SIGINT, original)
    records = finish(recorder)
    assert any(r.get("event") == "python_signal_handler" and r["signal"] == signal.SIGINT for r in records)


def test_retention_preserves_active_and_unmanaged_runs(tmp_path, monkeypatch):
    monkeypatch.setattr(recording, "DEBUG_ROOT", tmp_path)
    monkeypatch.setattr(recording, "MAX_RUNS", 1)
    monkeypatch.setenv("VR_DEBUG_RUN_ID", "new")
    unmanaged = tmp_path / "user-notes"
    unmanaged.mkdir()
    active = tmp_path / "active"
    active.mkdir()
    (active / ".managed-vr-debug").touch()
    (active / f".active-{os.getpid()}").write_text(recording._start_token(os.getpid()))
    inactive = tmp_path / "old"
    inactive.mkdir()
    (inactive / ".managed-vr-debug").touch()
    (inactive / ".active-99999999").write_text("obsolete")
    os.utime(active, (1, 1))
    os.utime(inactive, (1, 1))
    assert recording._prepare_run() == tmp_path / "new"
    assert active.exists() and unmanaged.exists()
    assert not inactive.exists()
