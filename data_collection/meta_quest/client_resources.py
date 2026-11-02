"""Bounded, read-only client resource sampler for Quest/SteamVR diagnosis.

The sampler reads Linux procfs/sysfs directly (no subprocesses, signals, or
privileged operations). It is intended for a detached observer and emits a
small JSON-compatible snapshot every couple of seconds. Process CPU usage is
reported as percent of one logical CPU, so a multithreaded process can exceed
100 percent.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import time
from typing import Any


PROCESS_KEYWORDS = (
    "vrserver", "vrcompositor", "alvr", "steam", "gnome-shell", "xwayland",
)
PSI_FILES = ("cpu", "memory", "io")
PSI_FIELDS = ("some", "full")
MAX_KEY_PROCESSES = 8
MAX_BUSY_PROCESSES = 3
MAX_THREADS_PER_PROCESS = 32
MAX_THREAD_DETAILS = 5
MAX_KERNEL_STACKS_PER_PROCESS = 2
MAX_KERNEL_STACK_CHARS = 384
MAX_SAMPLE_BYTES = 12 * 1024
_DRM_WAIT = re.compile(r"drm|i915|syncobj|fence|gpu|vulkan|render", re.IGNORECASE)


def _proc_stat(path: Path) -> dict[str, Any]:
    """Parse one procfs stat record without splitting a comm containing spaces."""
    raw = path.read_text(errors="replace")
    close = raw.rfind(")")
    if close < 0:
        raise ValueError("malformed proc stat")
    comm = raw[raw.find("(") + 1:close]
    fields = raw[close + 2:].split()
    if len(fields) < 22:
        raise ValueError("short proc stat")
    return {
        "comm": comm[:64],
        "state": fields[0],
        "ppid": int(fields[1]),
        "cpu_ticks": int(fields[11]) + int(fields[12]),
        "priority": int(fields[15]),
        "nice": int(fields[16]),
        "threads": int(fields[17]),
        "start_token": fields[19],
        "rss_bytes": max(0, int(fields[21])) * os.sysconf("SC_PAGE_SIZE"),
        "cpu": int(fields[36]) if len(fields) > 36 else None,
    }


def _read_ints(path: Path) -> list[int]:
    return [int(value) for value in path.read_text().split()]


def _read_kv(path: Path) -> dict[str, int]:
    result: dict[str, int] = {}
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) >= 2:
            try:
                result[fields[0].rstrip(":")] = int(fields[1])
            except ValueError:
                continue
    return result


class ClientResourceSampler:
    """Sample host contention and a bounded set of SteamVR/client processes."""

    def __init__(
        self,
        parent_pid: int,
        *,
        proc_root: Path = Path("/proc"),
        sys_root: Path = Path("/sys"),
        max_sample_bytes: int = MAX_SAMPLE_BYTES,
    ) -> None:
        self.parent_pid = int(parent_pid)
        self.proc_root = Path(proc_root)
        self.sys_root = Path(sys_root)
        self.max_sample_bytes = min(MAX_SAMPLE_BYTES, max(2048, int(max_sample_bytes)))
        self._previous_at: float | None = None
        self._previous_cpu: dict[str, list[int]] = {}
        self._previous_vm: dict[str, int] = {}
        self._previous_psi: dict[tuple[str, str], int] = {}
        self._previous_processes: dict[tuple[int, str], dict[str, Any]] = {}
        self._previous_threads: dict[tuple[int, str, int, str], tuple[int, int]] = {}
        self._previous_io: dict[tuple[int, str], tuple[int, int]] = {}
        self._previous_net: dict[str, tuple[int, int, int, int, int, int, int, int]] = {}
        self._previous_wireless: dict[str, dict[str, int]] = {}
        self._clock_ticks = os.sysconf("SC_CLK_TCK")
        self._page_size = os.sysconf("SC_PAGE_SIZE")

    def _error(self, errors: list[str], source: str, exc: BaseException) -> None:
        if len(errors) < 24:
            errors.append(f"{source}:{type(exc).__name__}")

    def _cpu(self, elapsed: float, errors: list[str]) -> dict[str, Any]:
        path = self.proc_root / "stat"
        current: dict[str, list[int]] = {}
        try:
            for line in path.read_text().splitlines():
                values = line.split()
                if values and (values[0] == "cpu" or re.fullmatch(r"cpu\d+", values[0])):
                    current[values[0]] = [int(v) for v in values[1:]]
        except (OSError, ValueError) as exc:
            self._error(errors, "proc_stat", exc)

        result: dict[str, Any] = {"logical_cpus": max(0, len(current) - ("cpu" in current))}
        previous = self._previous_cpu
        if elapsed > 0 and current:
            totals: dict[str, dict[str, float | None]] = {}
            for name, values in current.items():
                before = previous.get(name)
                if before is None or len(before) < 5 or len(values) < 5:
                    totals[name] = {"busy_pct": None, "iowait_pct": None}
                    continue
                deltas = [max(0, after - before[i]) for i, after in enumerate(values[:8])]
                total = sum(deltas)
                idle = deltas[3]
                iowait = deltas[4]
                totals[name] = {
                    "busy_pct": round(100 * max(0, total - idle - iowait) / total, 2) if total else None,
                    "iowait_pct": round(100 * iowait / total, 2) if total else None,
                }
            result["total"] = totals.pop("cpu", {"busy_pct": None, "iowait_pct": None})
            per_cpu = {k: v for k, v in totals.items() if k.startswith("cpu")}
            busy = sorted(((v["busy_pct"], k) for k, v in per_cpu.items() if v["busy_pct"] is not None), reverse=True)
            result["busiest_cores"] = [{"cpu": k, **per_cpu[k]} for _, k in busy[:4]]
        else:
            result["total"] = {"busy_pct": None, "iowait_pct": None}
            result["busiest_cores"] = []
        self._previous_cpu = current
        try:
            load = (self.proc_root / "loadavg").read_text().split()
            runq, total = load[3].split("/", 1)
            result["load1"] = float(load[0])
            result["runnable"] = int(runq)
            result["processes"] = int(total)
        except (OSError, ValueError, IndexError) as exc:
            self._error(errors, "loadavg", exc)
        return result

    def _memory(self, errors: list[str]) -> dict[str, Any]:
        try:
            values = _read_kv(self.proc_root / "meminfo")
            return {
                "mem_total_kib": values.get("MemTotal"),
                "mem_available_kib": values.get("MemAvailable"),
                "swap_total_kib": values.get("SwapTotal"),
                "swap_free_kib": values.get("SwapFree"),
                "cached_kib": values.get("Cached"),
            }
        except (OSError, ValueError) as exc:
            self._error(errors, "meminfo", exc)
            return {"mem_total_kib": None, "mem_available_kib": None,
                    "swap_total_kib": None, "swap_free_kib": None}

    def _vmstat(self, elapsed: float, errors: list[str]) -> dict[str, Any]:
        keys = ("pgmajfault", "pswpin", "pswpout")
        try:
            values = _read_kv(self.proc_root / "vmstat")
            result: dict[str, Any] = {}
            for key in keys:
                current = values.get(key)
                previous = self._previous_vm.get(key)
                result[key] = current
                result[key + "_per_s"] = (
                    round(max(0, current - previous) / elapsed, 3)
                    if current is not None and previous is not None and elapsed > 0 else None
                )
                if current is not None:
                    self._previous_vm[key] = current
            return result
        except (OSError, ValueError) as exc:
            self._error(errors, "vmstat", exc)
            return {key: None for key in keys}

    def _pressure(self, elapsed: float, errors: list[str]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for resource in PSI_FILES:
            pressure: dict[str, Any] = {}
            try:
                lines = (self.proc_root / "pressure" / resource).read_text().splitlines()
                for line in lines:
                    values: dict[str, str] = {}
                    for pair in line.split():
                        if "=" in pair:
                            key, value = pair.split("=", 1)
                            values[key] = value
                    kind = line.split(maxsplit=1)[0]
                    total = int(values["total"]) if "total" in values else None
                    key = (resource, kind)
                    previous = self._previous_psi.get(key)
                    pressure[kind] = {
                        "avg10": float(values["avg10"]) if "avg10" in values else None,
                        "total_us": total,
                        "delta_pct": (round(max(0, total - previous) / (elapsed * 10000), 3)
                                      if total is not None and previous is not None and elapsed > 0 else None),
                    }
                    if total is not None:
                        self._previous_psi[key] = total
            except (OSError, ValueError, KeyError, IndexError) as exc:
                self._error(errors, f"psi_{resource}", exc)
            result[resource] = pressure
        return result

    def _network(self, elapsed: float, errors: list[str]) -> dict[str, Any]:
        """Read interface byte/error counters and wireless link quality from procfs."""
        counters: dict[str, tuple[int, int, int, int, int, int, int, int]] = {}
        try:
            lines = (self.proc_root / "net/dev").read_text().splitlines()[2:]
            for line in lines:
                if ":" not in line:
                    continue
                name, values_text = line.split(":", 1)
                values = [int(value) for value in values_text.split()]
                if len(values) < 12:
                    continue
                # rx/tx bytes, packets, errors and dropped packets.
                counters[name.strip()[:32]] = (values[0], values[1], values[2],
                                                values[3], values[8], values[9],
                                                values[10], values[11])
        except (OSError, ValueError) as exc:
            self._error(errors, "net_dev", exc)

        wireless: dict[str, dict[str, Any]] = {}
        try:
            for line in (self.proc_root / "net/wireless").read_text().splitlines()[2:]:
                if ":" not in line:
                    continue
                name, values_text = line.split(":", 1)
                values = values_text.split()
                if len(values) < 3:
                    continue
                interface = name.strip()[:32]
                metrics: dict[str, Any] = {
                    "quality": float(values[1].rstrip(".")),
                    "level_dbm": float(values[2].rstrip(".")),
                    "noise_dbm": float(values[3].rstrip(".")) if len(values) > 3 else None,
                }
                discard_keys = (("discard_nwid", 4), ("discard_crypt", 5),
                                ("discard_frag", 6), ("discard_retry", 7),
                                ("discard_misc", 8), ("missed_beacon", 9))
                current: dict[str, int] = {}
                for key, index in discard_keys:
                    if len(values) <= index:
                        continue
                    try:
                        current[key] = int(float(values[index].rstrip(".")))
                    except ValueError:
                        continue
                before = self._previous_wireless.get(interface, {})
                for key, value in current.items():
                    metrics[key] = value
                    previous = before.get(key)
                    if previous is not None:
                        metrics[key + "_delta"] = max(0, value - previous)
                self._previous_wireless[interface] = current
                wireless[interface] = metrics
        except (OSError, ValueError) as exc:
            # Wired-only clients commonly have no wireless procfs file.
            if not isinstance(exc, FileNotFoundError):
                self._error(errors, "net_wireless", exc)

        ranked = sorted(counters, key=lambda name: (
            name not in wireless,
            -sum(counters[name][index] for index in (0, 4)),
            name,
        ))[:8]
        interfaces: list[dict[str, Any]] = []
        for name in ranked:
            current = counters[name]
            before = self._previous_net.get(name)
            item: dict[str, Any] = {
                "name": name,
                "rx_bytes": current[0], "rx_packets": current[1],
                "rx_errors": current[2], "rx_dropped": current[3],
                "tx_bytes": current[4], "tx_packets": current[5],
                "tx_errors": current[6], "tx_dropped": current[7],
            }
            # Keep rates separate from cumulative counters so resets remain visible.
            if before is not None and elapsed > 0:
                item["rx_bytes_per_s"] = round(max(0, current[0] - before[0]) / elapsed, 1)
                item["tx_bytes_per_s"] = round(max(0, current[4] - before[4]) / elapsed, 1)
                item["rx_dropped_delta"] = max(0, current[3] - before[3])
                item["tx_dropped_delta"] = max(0, current[7] - before[7])
                item["rx_errors_delta"] = max(0, current[2] - before[2])
                item["tx_errors_delta"] = max(0, current[6] - before[6])
            if name in wireless:
                item["wireless"] = wireless[name]
            interfaces.append(item)
        self._previous_net = {name: counters[name] for name in ranked}
        self._previous_wireless = {name: self._previous_wireless.get(name, {}) for name in wireless}
        return {"interfaces": interfaces, "wireless_interfaces": sorted(wireless)[:8]}

    def _scan_processes(self, now: float, elapsed: float, errors: list[str]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        current: dict[tuple[int, str], dict[str, Any]] = {}
        try:
            entries = self.proc_root.iterdir()
            for path in entries:
                if not path.name.isdigit():
                    continue
                try:
                    stat = _proc_stat(path / "stat")
                except (OSError, ValueError, IndexError):
                    continue
                pid = int(path.name)
                key = (pid, stat["start_token"])
                before = self._previous_processes.get(key)
                cpu_pct = None
                if before is not None and elapsed > 0:
                    cpu_pct = round(max(0, stat["cpu_ticks"] - before["cpu_ticks"]) * 100 /
                                    (self._clock_ticks * elapsed), 2)
                current[key] = {**stat, "pid": pid, "cpu_pct_one_core": cpu_pct}
        except OSError as exc:
            self._error(errors, "proc_scan", exc)
        self._previous_processes = current

        by_pid = {item["pid"]: (key, item) for key, item in current.items()}
        parent_entry = by_pid.get(self.parent_pid)
        selected: list[tuple[tuple[int, str], dict[str, Any], str]] = []
        included: set[tuple[int, str]] = set()
        if parent_entry is not None:
            key, item = parent_entry
            selected.append((key, item, "button_check"))
            included.add(key)
        matches = []
        for key, item in current.items():
            if key in included:
                continue
            name = item["comm"].lower()
            matched = next((keyword for keyword in PROCESS_KEYWORDS if keyword in name), None)
            if matched:
                matches.append((key, item, matched))
        matches.sort(key=lambda entry: (PROCESS_KEYWORDS.index(entry[2]), -(entry[1].get("cpu_pct_one_core") or 0), entry[1]["pid"]))
        for entry in matches:
            if len(selected) >= MAX_KEY_PROCESSES:
                break
            selected.append(entry)
            included.add(entry[0])

        detail = [self._process_detail(key, item, label, now, elapsed, errors)
                  for key, item, label in selected]
        # Keep delta baselines only for identities sampled this round. Without
        # this, short-lived matching SteamVR processes can grow these maps over
        # a long diagnostic run.
        self._previous_threads = {
            k: v for k, v in self._previous_threads.items()
            if (k[0], k[1]) in included
        }
        self._previous_io = {k: v for k, v in self._previous_io.items() if k in included}
        busy = sorted(
            ((item.get("cpu_pct_one_core") or 0, key, item)
             for key, item in current.items() if key not in included),
            key=lambda entry: entry[0], reverse=True,
        )[:MAX_BUSY_PROCESSES]
        top_busy = [{"pid": item["pid"], "comm": item["comm"],
                     "cpu_pct_one_core": item["cpu_pct_one_core"],
                     "rss_kib": item["rss_bytes"] // 1024, "state": item["state"]}
                    for _, _, item in busy]
        self._previous_processes = current
        return detail, top_busy

    def _process_detail(
        self, key: tuple[int, str], stat: dict[str, Any], label: str,
        now: float, elapsed: float, errors: list[str],
    ) -> dict[str, Any]:
        pid, start_token = key
        base = self.proc_root / str(pid)
        result: dict[str, Any] = {
            "pid": pid, "start_token": start_token, "role": label, "comm": stat["comm"],
            "state": stat["state"], "cpu_pct_one_core": stat["cpu_pct_one_core"],
            "rss_kib": stat["rss_bytes"] // 1024, "threads": stat["threads"],
            "cpu": stat["cpu"], "priority": stat["priority"], "nice": stat["nice"],
        }
        try:
            status = (base / "status").read_text()
            for line in status.splitlines():
                if line.startswith("Cpus_allowed_list:"):
                    result["cpus_allowed"] = line.split(":", 1)[1].strip()[:48]
                    break
        except OSError:
            pass
        try:
            values = _read_ints(base / "schedstat")
            if len(values) >= 2:
                result["sched_runtime_ms"] = round(values[0] / 1e6, 2)
                result["sched_wait_ms"] = round(values[1] / 1e6, 2)
        except (OSError, ValueError) as exc:
            self._error(errors, f"schedstat_{pid}", exc)
        io_key = key
        try:
            io = _read_kv(base / "io")
            read_bytes, write_bytes = io.get("read_bytes"), io.get("write_bytes")
            before = self._previous_io.get(io_key)
            if read_bytes is not None and write_bytes is not None:
                result["io_read_bytes_per_s"] = (
                    round(max(0, read_bytes - before[0]) / elapsed, 1)
                    if before and elapsed > 0 else None
                )
                result["io_write_bytes_per_s"] = (
                    round(max(0, write_bytes - before[1]) / elapsed, 1)
                    if before and elapsed > 0 else None
                )
                self._previous_io[io_key] = (read_bytes, write_bytes)
        except (OSError, ValueError) as exc:
            self._error(errors, f"io_{pid}", exc)
        if stat["start_token"] == self._process_start_token(pid):
            result["thread_waits"] = self._thread_waits(pid, start_token, errors)
        else:
            result["thread_waits"] = {"state": "pid_reused", "threads": []}
        return result

    def _process_start_token(self, pid: int) -> str | None:
        try:
            return _proc_stat(self.proc_root / str(pid) / "stat")["start_token"]
        except (OSError, ValueError, IndexError):
            return None

    def _thread_waits(self, pid: int, start_token: str, errors: list[str]) -> dict[str, Any]:
        result: dict[str, Any] = {"process_start_token": start_token, "scanned": 0, "threads": []}
        task_root = self.proc_root / str(pid) / "task"
        try:
            tids = sorted((int(p.name) for p in task_root.iterdir() if p.name.isdigit()))
        except OSError as exc:
            self._error(errors, f"task_{pid}", exc)
            result["state"] = "unavailable"
            return result
        candidates: list[dict[str, Any]] = []
        for tid in tids[:MAX_THREADS_PER_PROCESS]:
            base = task_root / str(tid)
            try:
                stat = _proc_stat(base / "stat")
                state = stat["state"]
                try:
                    wchan = (base / "wchan").read_text().strip()[:96]
                except OSError:
                    wchan = "unknown"
                try:
                    sched = _read_ints(base / "schedstat")
                except (OSError, ValueError):
                    sched = []
                runtime_ns = sched[0] if len(sched) > 0 else None
                wait_ns = sched[1] if len(sched) > 1 else None
                identity = (pid, start_token, tid, stat["start_token"])
                before = self._previous_threads.get(identity)
                item = {
                    "tid": tid, "comm": stat["comm"], "state": state, "wchan": wchan,
                    "runtime_ms": round(runtime_ns / 1e6, 2) if runtime_ns is not None else None,
                    "wait_ms": round(wait_ns / 1e6, 2) if wait_ns is not None else None,
                    "runtime_delta_ms": (round(max(0, runtime_ns - before[0]) / 1e6, 2)
                                         if runtime_ns is not None and before and before[0] is not None else None),
                    "wait_delta_ms": (round(max(0, wait_ns - before[1]) / 1e6, 2)
                                      if wait_ns is not None and before and before[1] is not None else None),
                }
                if (state in {"R", "D"}) or _DRM_WAIT.search(wchan):
                    candidates.append(item)
                if runtime_ns is not None and wait_ns is not None:
                    self._previous_threads[identity] = (runtime_ns, wait_ns)
                result["scanned"] += 1
            except (OSError, ValueError, IndexError):
                continue
        self._previous_threads = {
            k: v for k, v in self._previous_threads.items()
            if k[0] != pid or k[1] == start_token
        }
        # Prefer D-state and recognizable graphics waits, then currently runnable work.
        candidates.sort(key=lambda x: (x["state"] == "D", bool(_DRM_WAIT.search(x["wchan"])),
                                      x["state"] == "R", x["wait_delta_ms"] or 0), reverse=True)
        result["threads"] = candidates[:MAX_THREAD_DETAILS]
        for item in result["threads"][:MAX_KERNEL_STACKS_PER_PROCESS]:
            try:
                stack = (task_root / str(item["tid"]) / "stack").read_text(errors="replace")
                item["kernel_stack"] = stack[:MAX_KERNEL_STACK_CHARS].strip()
            except OSError as exc:
                # Access is commonly restricted by kptr_restrict/ptrace policy.
                item["kernel_stack_unavailable"] = type(exc).__name__
        result["more_threads"] = max(0, len(tids) - MAX_THREADS_PER_PROCESS)
        if not result["threads"]:
            result["state"] = "no_suspicious_waits"
        return result

    def _gpu(self, errors: list[str]) -> dict[str, Any]:
        """Read a few common read-only DRM busy/frequency attributes."""
        cards: list[dict[str, Any]] = []
        root = self.sys_root / "class/drm"
        try:
            paths = sorted(root.glob("card[0-9]*/device"))
        except OSError as exc:
            self._error(errors, "drm_sysfs", exc)
            return {"state": "unknown", "cards": []}
        for device in paths[:3]:
            card: dict[str, Any] = {"name": device.parent.name}
            for attr, key in (("vendor", "vendor"), ("device", "device_id"),
                              ("gpu_busy_percent", "busy_pct"),
                              ("gt_cur_freq_mhz", "current_mhz"),
                              ("gt_act_freq_mhz", "actual_mhz"),
                              ("gt_max_freq_mhz", "max_mhz")):
                try:
                    text = (device / attr).read_text().strip()
                    if attr in {"vendor", "device"}:
                        card[key] = text[:24]
                    else:
                        card[key] = float(text) if "." in text else int(text)
                except (OSError, ValueError):
                    continue
            cards.append(card)
        return {"state": "available" if any("busy_pct" in c or "actual_mhz" in c for c in cards) else "unknown",
                "cards": cards}

    def _thermal_cpu(self, errors: list[str]) -> dict[str, Any]:
        """Best-effort temperature and CPU frequency hints, without polling tools."""
        zones: list[dict[str, Any]] = []
        thermal_root = self.sys_root / "class/thermal"
        try:
            paths = sorted(thermal_root.glob("thermal_zone[0-9]*"))
            for path in paths[:8]:
                try:
                    raw_temp = int((path / "temp").read_text().strip())
                except (OSError, ValueError):
                    continue
                try:
                    kind = (path / "type").read_text().strip()[:48]
                except OSError:
                    kind = "unknown"
                # Most thermal drivers export millidegrees Celsius; preserve a
                # coarse raw value when the unit scale is unusual.
                temp_c = round(raw_temp / 1000.0, 1) if abs(raw_temp) > 1000 else float(raw_temp)
                zones.append({"zone": path.name, "type": kind, "temp_c": temp_c})
        except OSError as exc:
            self._error(errors, "thermal_sysfs", exc)

        frequencies: list[dict[str, Any]] = []
        cpu_root = self.sys_root / "devices/system/cpu"
        try:
            cpu_paths = sorted(cpu_root.glob("cpu[0-9]*"), key=lambda p: int(p.name[3:]))
            for path in cpu_paths[:4]:
                values: dict[str, int] = {}
                for attr, key in (("scaling_cur_freq", "current_khz"),
                                  ("scaling_max_freq", "max_khz")):
                    try:
                        values[key] = int((path / "cpufreq" / attr).read_text().strip())
                    except (OSError, ValueError):
                        continue
                if values:
                    frequencies.append({"cpu": path.name, **values})
        except (OSError, ValueError) as exc:
            self._error(errors, "cpu_frequency_sysfs", exc)
        return {"temperatures": zones, "cpu_frequency": frequencies}

    def _compact(self, sample: dict[str, Any]) -> dict[str, Any]:
        """Enforce a hard JSON size bound while preserving useful host totals."""
        try:
            if len(json.dumps(sample, ensure_ascii=False, separators=(",", ":")).encode()) <= self.max_sample_bytes:
                return sample
        except (TypeError, ValueError):
            pass
        sample["truncated"] = True
        for process in sample.get("processes", []):
            waits = process.get("thread_waits", {}).get("threads", [])
            if len(waits) > 2:
                process["thread_waits"]["omitted"] = len(waits) - 2
                process["thread_waits"]["threads"] = waits[:2]
        sample["top_cpu_processes"] = sample.get("top_cpu_processes", [])[:2]
        sample["processes"] = sample.get("processes", [])[:MAX_KEY_PROCESSES]
        sample["cpu"].pop("busiest_cores", None)
        try:
            if len(json.dumps(sample, ensure_ascii=False, separators=(",", ":")).encode()) <= self.max_sample_bytes:
                return sample
        except (TypeError, ValueError):
            pass
        # If procfs exposes unusually long names or errors, retain concise host
        # aggregates and explicit omission information instead of emitting an
        # unbounded record.
        return {
            "schema": 1, "wall_time": sample.get("wall_time"),
            "monotonic_s": sample.get("monotonic_s"), "elapsed_s": sample.get("elapsed_s"),
            "cpu": {"logical_cpus": sample.get("cpu", {}).get("logical_cpus"),
                    "total": sample.get("cpu", {}).get("total"),
                    "load1": sample.get("cpu", {}).get("load1"),
                    "runnable": sample.get("cpu", {}).get("runnable")},
            "memory": {k: sample.get("memory", {}).get(k) for k in
                       ("mem_total_kib", "mem_available_kib", "swap_total_kib", "swap_free_kib")},
            "network": sample.get("network", {"interfaces": [], "wireless_interfaces": []}),
            "gpu": sample.get("gpu", {"state": "unknown", "cards": []}),
            "thermal_cpu": sample.get("thermal_cpu", {"temperatures": [], "cpu_frequency": []}),
            "processes_omitted": len(sample.get("processes", [])),
            "errors": sample.get("errors", [])[:8], "truncated": True,
        }

    def sample(self) -> dict[str, Any]:
        errors: list[str] = []
        now = time.monotonic()
        elapsed = now - self._previous_at if self._previous_at is not None else 0.0
        elapsed = max(elapsed, 0.001)
        sample: dict[str, Any] = {
            "schema": 1,
            "wall_time": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "wall_time_ns": time.time_ns(),
            "monotonic_s": now,
            "elapsed_s": round(elapsed, 3),
            "cpu": self._cpu(elapsed, errors),
            "memory": self._memory(errors),
            "vmstat": self._vmstat(elapsed, errors),
            "psi": self._pressure(elapsed, errors),
            "network": self._network(elapsed, errors),
            "gpu": self._gpu(errors),
            "thermal_cpu": self._thermal_cpu(errors),
        }
        sample["processes"], sample["top_cpu_processes"] = self._scan_processes(now, elapsed, errors)
        sample["errors"] = errors
        self._previous_at = now
        return self._compact(sample)
