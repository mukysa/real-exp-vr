"""Offline checks for bounded diagnostics; never touch a VR runtime."""

from datetime import datetime, timezone
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from data_collection.meta_quest import debug_capture as capture


@pytest.fixture(autouse=True)
def diagnostic_output_root(tmp_path, monkeypatch):
    """Confine synthetic collectors to this test's temporary output root."""
    monkeypatch.setattr(capture, "DEBUG_ROOT", tmp_path)


def steam_line(second: int, message: str) -> str:
    return f"Fri Oct 30 2026 11:35:{second:02d}.123456 [Info] - {message}\n"


def fake_proc(root: Path, pid: int, *, state: str = "S", token: str = "1234",
              comm: str = "a process (name)") -> None:
    base = root / str(pid)
    base.mkdir(parents=True, exist_ok=True)
    fields = [state] + ["0"] * 21
    fields[1] = "12"
    fields[11], fields[12] = "20", "30"
    fields[15], fields[16], fields[17] = "20", "0", "3"
    fields[19], fields[21] = token, "4"
    (base / "stat").write_text(f"{pid} ({comm}) " + " ".join(fields))
    (base / "status").write_text(
        "VmSwap:\t12 kB\nvoluntary_ctxt_switches:\t13\n"
        "SigPnd:\t0000000000000002\nShdPnd:\t0000000000000000\n"
    )
    (base / "wchan").write_text("futex_wait_queue")
    (base / "maps").write_text(
        "0000-1000 r-xp 0 0 0 /opt/sapien/libsapien.so\n"
        "1000-2000 r--p 0 0 0 /opt/sapien/libsapien.so\n"
        "2000-3000 r-xp 0 0 0 /opt/other.so\n"
    )


def test_log_cursor_excludes_history_and_handles_rotation_and_partial_write(tmp_path):
    path = tmp_path / "vrserver.txt"
    started = datetime(2026, 10, 30, 11, 35, 20).timestamp()
    path.write_text(steam_line(1, "OLD ERROR") + steam_line(21, "VR_Init successful"))
    cursor = capture.LogCursor(path, started)
    cursor.poll()
    first = b"".join(cursor.tail.lines).decode()
    assert "OLD ERROR" not in first
    assert "VR_Init successful" in first
    with path.open("a") as handle:
        handle.write(steam_line(22, "GPU HANG")[:-4])
    cursor.poll()
    assert "GPU HANG" not in b"".join(cursor.tail.lines).decode()
    with path.open("a") as handle:
        handle.write("ANG\n")
        handle.write(steam_line(23, "Failed old inode tail"))
    path.rename(tmp_path / "vrserver.previous.txt")
    path.write_text(steam_line(24, "VR_Shutdown called"))
    cursor.poll()
    output = b"".join(cursor.tail.lines).decode()
    assert "GPU HANG" in output
    assert "Failed old inode tail" in output
    assert "VR_Shutdown called" in output
    assert cursor.rotations == 1


def test_log_cursor_reports_skipped_bytes_and_caps_retained_output(tmp_path):
    path = tmp_path / "vrserver.txt"
    started = datetime(2026, 10, 30, 11, 35, 20).timestamp()
    path.write_text("".join(steam_line(21, "error " + str(i)) for i in range(100)))
    cursor = capture.LogCursor(path, started, limit=256)
    cursor.poll()
    assert cursor.skipped_input_bytes > 0
    assert cursor.tail.size <= 256
    assert b"error 99" in b"".join(cursor.tail.lines)


def test_log_cursor_keeps_bounded_context_around_lifecycle_events(tmp_path):
    path = tmp_path / "vrserver.txt"
    started = datetime(2026, 10, 30, 11, 35, 20).timestamp()
    path.write_text(
        steam_line(19, "OLD ERROR")
        + steam_line(20, "initialization context before")
        + steam_line(21, "VR_Init successful")
        + steam_line(22, "renderer connected")
        + steam_line(23, "GPU HANG")
        + steam_line(24, "renderer teardown detail")
    )
    cursor = capture.LogCursor(path, started)
    cursor.poll()
    output = b"".join(cursor.tail.lines).decode()
    assert "OLD ERROR" not in output
    assert "CONTEXT_BEFORE Fri Oct 30 2026 11:35:20.123456 [Info] - initialization" in output
    assert "EVENT Fri Oct 30 2026 11:35:21.123456 [Info] - VR_Init successful" in output
    assert "CONTEXT_AFTER Fri Oct 30 2026 11:35:22.123456 [Info] - renderer connected" in output
    assert "CONTEXT_AFTER Fri Oct 30 2026 11:35:24.123456 [Info] - renderer teardown detail" in output


def test_line_time_parses_iso_utc_alvr_timestamp():
    parsed = capture.line_time("[2026-10-30T03:35:20.123Z INFO alvr_server] encoder ready")
    expected = datetime(2026, 10, 30, 3, 35, 20, 123000, tzinfo=timezone.utc).timestamp()
    assert parsed == expected


def test_process_identity_parser_and_zombie_detection(tmp_path):
    fake_proc(tmp_path / "proc", 42)
    stat = capture.process_stat(42, tmp_path / "proc")
    assert stat["start_token"] == "1234"
    assert stat["cpu_ticks"] == 50
    collector = capture.Collector(
        42, tmp_path / "out", "1234", 0,
        proc_root=tmp_path / "proc", log_dir=tmp_path / "logs",
        sys_root=tmp_path / "sys",
    )
    assert collector.observe()
    process_sample = json.loads(b"".join(collector.resources.lines).decode().splitlines()[-1])
    assert process_sample["SigPnd"] == "0000000000000002"
    collector.collect_logs()
    assert collector.libraries == ["/opt/sapien/libsapien.so"]
    fake_proc(tmp_path / "proc", 42, state="Z")
    assert not collector.observe()
    assert collector.finished_reason == "parent_zombie"
    fake_proc(tmp_path / "proc", 42, token="other")
    assert not collector.observe()
    assert collector.finished_reason == "pid_reused"


def test_output_budget_and_kernel_failures_are_bounded(tmp_path, monkeypatch):
    collector = capture.Collector(
        42, tmp_path / "out", "1234", 0,
        proc_root=tmp_path / "proc", log_dir=tmp_path / "logs",
        sys_root=tmp_path / "sys",
    )
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=1, stdout="", stderr="journal unavailable")

    monkeypatch.setattr(capture.subprocess, "run", run)
    for _ in range(3):
        collector.collect_kernel()
    assert collector.errors["kernel"]["count"] == 3
    assert calls[0][1]["timeout"] == 3.0
    for _ in range(400):
        collector.resources.add(json.dumps({"sample": "x" * 1000}))
    for cursor in collector.logs.values():
        cursor.tail.add("error " + "x" * 100000)
    collector.libraries = ["/" + "x" * 2000 + str(i) for i in range(128)]
    collector.flush()
    files = list((tmp_path / "out").iterdir())
    assert sum(path.stat().st_size for path in files) < 1024 * 1024
    assert (tmp_path / "out/process-resources.jsonl").stat().st_size <= capture.RESOURCE_BYTES
    assert (tmp_path / "out/client-resources.jsonl").stat().st_size <= capture.CLIENT_RESOURCE_BYTES
    assert isinstance(json.loads((tmp_path / "out/graphics-libraries.json").read_text()), list)
    metadata = json.loads((tmp_path / "out/collector.json").read_text())
    assert metadata["discarded_resource_bytes"] > 0
    assert metadata["errors"]["kernel"]["count"] == 3
    assert metadata["client_resource_fields"].startswith("host CPU/")
    assert "minidumps" not in metadata


def test_parent_exit_collects_after_grace_without_sending_signals(tmp_path, monkeypatch):
    collector = capture.Collector(
        4242, tmp_path / "out", "missing", 0,
        proc_root=tmp_path / "proc", log_dir=tmp_path / "logs",
        sys_root=tmp_path / "sys",
    )
    calls = []
    monkeypatch.setattr(collector, "collect_kernel", lambda: calls.append("kernel"))
    monkeypatch.setattr(capture.time, "sleep", lambda duration: calls.append(duration))
    collector.run()
    assert calls == ["kernel", 2.0, "kernel"]
    metadata = json.loads((tmp_path / "out/collector.json").read_text())
    assert metadata["state"] == "parent_unavailable"


def test_observer_output_stays_under_vr_debug(tmp_path, monkeypatch):
    debug_root = tmp_path / "vr_debug"
    monkeypatch.setattr(capture, "DEBUG_ROOT", debug_root)
    inside = debug_root / "run" / "button_check-42"
    assert capture.validate_output_dir(inside) == inside
    with pytest.raises(ValueError, match="inside repository vr_debug"):
        capture.validate_output_dir(tmp_path / "outside")


def test_client_resources_are_written_as_bounded_jsonl(tmp_path):
    proc = tmp_path / "proc"
    fake_proc(proc, 42)
    (proc / "stat").write_text("cpu 100 0 50 1000 10 0 0 0 0 0\ncpu0 100 0 50 1000 10 0 0 0 0 0\n")
    (proc / "loadavg").write_text("0.25 0.30 0.20 1/12 42\n")
    (proc / "meminfo").write_text("MemTotal: 8000 kB\nMemAvailable: 4000 kB\nSwapTotal: 1000 kB\nSwapFree: 1000 kB\n")
    (proc / "vmstat").write_text("pgmajfault 1\npswpin 0\npswpout 0\n")
    (proc / "pressure").mkdir()
    for name in ("cpu", "memory", "io"):
        (proc / "pressure" / name).write_text("some avg10=0.00 avg60=0.00 avg300=0.00 total=0\n")
    output = tmp_path / "vr_debug" / "run" / "button_check-42"
    collector = capture.Collector(
        42, output, "1234", 0, proc_root=proc,
        log_dir=tmp_path / "logs", sys_root=tmp_path / "sys",
    )
    assert collector.observe()
    collector.flush()
    rows = [json.loads(line) for line in (output / "client-resources.jsonl").read_text().splitlines()]
    assert len(rows) == 1
    assert rows[0]["cpu"]["logical_cpus"] == 1
    assert rows[0]["memory"]["mem_available_kib"] == 4000
    assert any(process["role"] == "button_check" for process in rows[0]["processes"])
    assert (output / "client-resources.jsonl").stat().st_size <= capture.CLIENT_RESOURCE_BYTES


def test_optional_alvr_log_is_read_only_and_written_inside_debug_output(tmp_path, monkeypatch):
    fake_proc(tmp_path / "proc", 42)
    started = datetime(2026, 10, 30, 11, 35, 20).timestamp()
    alvr_log = tmp_path / "alvr-server.log"
    alvr_log.write_text(steam_line(20, "ALVR encoder started") + steam_line(21, "server session ready"))
    monkeypatch.setenv("VR_DEBUG_ALVR_LOG", str(alvr_log))
    output = tmp_path / "vr_debug" / "run" / "button_check-42"
    collector = capture.Collector(
        42, output, "1234", started, proc_root=tmp_path / "proc",
        log_dir=tmp_path / "logs", sys_root=tmp_path / "sys",
    )
    collector.collect_logs()
    collector.flush()
    captured = (output / "alvr-runtime.log").read_text()
    assert "EVENT Fri Oct 30 2026 11:35:20.123456 [Info] - ALVR encoder started" in captured
    assert "CONTEXT_AFTER Fri Oct 30 2026 11:35:21.123456 [Info] - server session ready" in captured
    metadata = json.loads((output / "collector.json").read_text())
    assert metadata["alvr_log_configured"] is True
    assert metadata["logs"]["runtime.log"]["available"] is True


def test_alvr_log_open_by_server_process_is_discovered_without_directory_scan(tmp_path):
    proc = tmp_path / "proc"
    fake_proc(proc, 42)
    fake_proc(proc, 43, token="5678", comm="alvr_server")
    log_path = tmp_path / "alvr-server.log"
    log_path.write_text(steam_line(1, "ALVR encoder initialized"))
    fd_dir = proc / "43" / "fd"
    fd_dir.mkdir()
    (fd_dir / "3").symlink_to(log_path)
    output = tmp_path / "vr_debug" / "run" / "button_check-42"
    collector = capture.Collector(
        42, output, "1234", 0, proc_root=proc,
        log_dir=tmp_path / "logs", sys_root=tmp_path / "sys",
    )
    assert collector.observe()
    collector.collect_logs()
    collector.flush()
    auto_files = list(output.glob("alvr-runtime-auto-*.log"))
    assert len(auto_files) == 1
    assert "ALVR encoder initialized" in auto_files[0].read_text()
    assert not collector.alvr_log_configured
    metadata = json.loads((output / "collector.json").read_text())
    assert metadata["alvr_log_discovery"] == "open_regular_file_detected"
