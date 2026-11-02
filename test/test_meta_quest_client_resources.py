"""Offline tests for the bounded procfs/sysfs client resource sampler."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from data_collection.meta_quest import client_resources


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now

    def time_ns(self) -> int:
        return int(self.now * 1_000_000_000)


def proc_stat(pid: int, comm: str, *, start: int, ticks: int = 0,
              state: str = "S", cpu: int = 0, threads: int = 1) -> str:
    # The list starts at Linux stat field 3 (state); indexes mirror procfs.
    fields = ["0"] * 37
    fields[0] = state
    fields[1] = "1"
    fields[11] = str(ticks // 2)
    fields[12] = str(ticks - ticks // 2)
    fields[15] = "20"
    fields[16] = "0"
    fields[17] = str(threads)
    fields[19] = str(start)
    fields[21] = "1000"
    fields[36] = str(cpu)
    return f"{pid} ({comm}) " + " ".join(fields) + "\n"


def put(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)


def put_host(proc: Path, *, cpu_busy: int, pgmajfault: int = 10,
             psi_total: int = 100_000) -> None:
    put(proc / "stat", f"cpu {cpu_busy} 0 0 {cpu_busy} 0 0 0 0 0 0\n"
        f"cpu0 {cpu_busy} 0 0 {cpu_busy} 0 0 0 0 0 0\n")
    put(proc / "loadavg", "1.25 0.90 0.80 2/100 1234\n")
    put(proc / "meminfo", "MemTotal: 8000000 kB\nMemAvailable: 3000000 kB\n"
        "SwapTotal: 1000000 kB\nSwapFree: 900000 kB\nCached: 2000000 kB\n")
    put(proc / "vmstat", f"pgmajfault {pgmajfault}\npswpin 20\npswpout 30\n")
    for name in ("cpu", "memory", "io"):
        put(proc / "pressure" / name,
            f"some avg10=1.25 avg60=0.50 avg300=0.10 total={psi_total}\n"
            f"full avg10=0.00 avg60=0.00 avg300=0.00 total={psi_total // 2}\n")


def put_process(proc: Path, pid: int, comm: str, *, start: int, ticks: int = 0,
                read_bytes: int = 100, write_bytes: int = 200,
                thread_wait: str = "futex_wait_queue", thread_state: str = "S",
                thread_start: int = 900) -> None:
    base = proc / str(pid)
    put(base / "stat", proc_stat(pid, comm, start=start, ticks=ticks, threads=1))
    put(base / "status", "Name:\ttest\nCpus_allowed_list:\t0-3\n")
    put(base / "schedstat", "10000000 20000000 3\n")
    put(base / "io", f"read_bytes: {read_bytes}\nwrite_bytes: {write_bytes}\n")
    task = base / "task" / str(pid)
    put(task / "stat", proc_stat(pid, comm, start=thread_start, state=thread_state))
    put(task / "wchan", thread_wait + "\n")
    put(task / "schedstat", "100000000 250000000 4\n")
    put(task / "stack", f"[<0>] {thread_wait}+0x10/0x20\n")


@pytest.fixture
def fake_roots(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    proc = tmp_path / "proc"
    sys = tmp_path / "sys"
    clock = FakeClock()
    monkeypatch.setattr(client_resources, "time", clock)
    return proc, sys, clock


def test_cpu_vmstat_psi_io_and_thread_wait_deltas(fake_roots) -> None:
    proc, sys, clock = fake_roots
    put_host(proc, cpu_busy=100, pgmajfault=10, psi_total=100_000)
    put_process(proc, 100, "button_check", start=777, ticks=100)
    sampler = client_resources.ClientResourceSampler(100, proc_root=proc, sys_root=sys)

    first = sampler.sample()
    assert first["cpu"]["total"]["busy_pct"] is None
    assert first["processes"][0]["cpu_pct_one_core"] is None
    assert first["processes"][0]["thread_waits"]["state"] == "no_suspicious_waits"

    clock.now += 2
    put_host(proc, cpu_busy=160, pgmajfault=14, psi_total=120_000)
    put_process(proc, 100, "button_check", start=777, ticks=110,
                read_bytes=500, write_bytes=600,
                thread_wait="i915_request_wait", thread_state="D")
    second = sampler.sample()

    assert second["cpu"]["total"]["busy_pct"] == 50.0
    assert second["vmstat"]["pgmajfault_per_s"] == 2.0
    assert second["psi"]["cpu"]["some"]["delta_pct"] == 1.0
    process = second["processes"][0]
    assert process["cpu_pct_one_core"] == 5.0
    assert process["io_read_bytes_per_s"] == 200.0
    assert process["io_write_bytes_per_s"] == 200.0
    waits = process["thread_waits"]["threads"]
    assert waits[0]["state"] == "D"
    assert waits[0]["wchan"] == "i915_request_wait"
    assert waits[0]["runtime_delta_ms"] == 0.0
    assert waits[0]["wait_delta_ms"] == 0.0
    assert "i915_request_wait" in waits[0]["kernel_stack"]


def test_pid_reuse_resets_process_and_thread_delta_baselines(fake_roots) -> None:
    proc, sys, clock = fake_roots
    put_host(proc, cpu_busy=100)
    put_process(proc, 100, "button_check", start=777, ticks=100,
                thread_wait="i915_request_wait", thread_state="D", thread_start=900)
    sampler = client_resources.ClientResourceSampler(100, proc_root=proc, sys_root=sys)
    sampler.sample()

    clock.now += 1
    put_host(proc, cpu_busy=120)
    put_process(proc, 100, "button_check", start=778, ticks=900,
                thread_wait="i915_request_wait", thread_state="D", thread_start=901)
    sample = sampler.sample()
    process = sample["processes"][0]
    assert process["start_token"] == "778"
    assert process["cpu_pct_one_core"] is None
    assert process["thread_waits"]["process_start_token"] == "778"
    assert process["thread_waits"]["threads"][0]["runtime_delta_ms"] is None


def test_missing_procfs_and_sysfs_fields_are_nonfatal(fake_roots) -> None:
    proc, sys, _ = fake_roots
    put(proc / "stat", "cpu 1 0 0 1\n")
    sampler = client_resources.ClientResourceSampler(999, proc_root=proc, sys_root=sys)
    result = sampler.sample()

    assert result["cpu"]["logical_cpus"] == 0
    assert result["memory"]["mem_total_kib"] is None
    assert result["gpu"]["state"] == "unknown"
    assert result["processes"] == []
    assert result["errors"]
    json.dumps(result)


def test_sample_size_is_hard_bounded_and_stale_baselines_are_pruned(fake_roots) -> None:
    proc, sys, clock = fake_roots
    put_host(proc, cpu_busy=100)
    for index in range(8):
        pid = 100 + index
        put_process(proc, pid, "vrserver" if index else "button_check",
                    start=1000 + index, ticks=100, thread_wait="drm_syncobj_wait_" + "x" * 80,
                    thread_state="D", thread_start=2000 + index)
    sampler = client_resources.ClientResourceSampler(100, proc_root=proc,
                                                     sys_root=sys, max_sample_bytes=2048)
    sample = sampler.sample()
    encoded = json.dumps(sample, ensure_ascii=False, separators=(",", ":")).encode()
    assert len(encoded) <= 2048
    assert sample.get("truncated") is True
    assert len(sampler._previous_threads) <= 8

    # Once those matching processes disappear, their per-thread baselines go too.
    clock.now += 1
    put_host(proc, cpu_busy=110)
    for path in proc.iterdir():
        if path.is_dir() and path.name.isdigit():
            for child in sorted(path.rglob("*"), reverse=True):
                if child.is_file():
                    child.unlink()
                elif child.is_dir():
                    child.rmdir()
            path.rmdir()
    sampler.sample()
    assert sampler._previous_threads == {}
    assert sampler._previous_io == {}


def test_network_counters_wireless_quality_and_temperature_frequency(fake_roots) -> None:
    proc, sys, _ = fake_roots
    net = proc / "net"
    put(net / "dev", "Inter-| Receive | Transmit\n face |bytes packets errs drop fifo frame compressed multicast|bytes packets errs drop fifo colls carrier compressed\n"
        " wlan0: 100 10 1 2 0 0 0 0 200 20 3 4 0 0 0 0\n")
    put(net / "wireless", "Inter-| sta | Quality | Discarded packets | Missed | WE\n face | tus | link level noise | nwid crypt frag retry misc | beacon | 22\n"
        " wlan0: 0000 70. -50. -95. 0 1 0 5 0 0 22\n")
    thermal = sys / "class/thermal/thermal_zone0"
    put(thermal / "type", "x86_pkg_temp\n")
    put(thermal / "temp", "51000\n")
    cpu = sys / "devices/system/cpu/cpu0/cpufreq"
    put(cpu / "scaling_cur_freq", "1800000\n")
    put(cpu / "scaling_max_freq", "4200000\n")
    sampler = client_resources.ClientResourceSampler(100, proc_root=proc, sys_root=sys)
    errors: list[str] = []

    first = sampler._network(2.0, errors)
    assert first["interfaces"][0]["wireless"]["quality"] == 70.0
    assert first["interfaces"][0]["wireless"]["level_dbm"] == -50.0
    assert first["interfaces"][0]["wireless"]["discard_retry"] == 5
    put(net / "wireless", "Inter-| sta | Quality | Discarded packets | Missed | WE\n face | tus | link level noise | nwid crypt frag retry misc | beacon | 22\n"
        " wlan0: 0000 68. -54. -95. 0 1 0 8 0 0 22\n")
    put(net / "dev", "Inter-| Receive | Transmit\n face |bytes packets errs drop fifo frame compressed multicast|bytes packets errs drop fifo colls carrier compressed\n"
        " wlan0: 200 20 2 4 0 0 0 0 400 40 4 8 0 0 0 0\n")
    second = sampler._network(2.0, errors)
    interface = second["interfaces"][0]
    assert interface["rx_bytes_per_s"] == 50.0
    assert interface["tx_bytes_per_s"] == 100.0
    assert interface["rx_dropped_delta"] == 2
    assert interface["tx_errors_delta"] == 1
    assert interface["wireless"]["discard_retry_delta"] == 3
    assert interface["wireless"]["quality"] == 68.0

    resources = sampler._thermal_cpu(errors)
    assert resources["temperatures"][0]["temp_c"] == 51.0
    assert resources["cpu_frequency"][0] == {
        "cpu": "cpu0", "current_khz": 1800000, "max_khz": 4200000,
    }

