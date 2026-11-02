"""Bounded, read-only observer for VR processes, independent of their GIL.

This process never calls a VR API or signals the observed process.  It survives
the Python monitor long enough to retain compositor/GPU errors during native
object destruction.  Missing permissions and missing tools are diagnostic data,
not reasons to stop the control process.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import stat as stat_module
import subprocess
import struct
import time
from typing import Any

from .client_resources import ClientResourceSampler


STEAM_LOGS = (
    "vrserver.txt", "vrmonitor.txt", "vrcompositor.txt",
    "vrcompositor-linux.txt", "vrclient_python3.txt",
)
KEY_LINES = re.compile(
    r"error|warn|fail|hang|reset|abort|timeout|watchdog|searching|"
    r"VR_Init|VR_Shutdown|startup|loaded server driver|load.*driver|"
    r"New Connect|VRApplication_|Disconnected|Streaming|encoder|"
    r"Controller_[LR]|Tracked device|SteamVRSystemState|Created remote Vulkan image",
    re.IGNORECASE,
)
ALVR_LINES = re.compile(
    r"error|warn|fail|hang|reset|abort|timeout|disconnect|connect|stream|client|"
    r"controller|tracking|encoder|decoder|network|packet|latency|bitrate|dropped|"
    r"frame|reprojection|recovery|driver",
    re.IGNORECASE,
)
LOG_CONTEXT_BEFORE = 6
LOG_CONTEXT_AFTER = 6
MAX_ALVR_LOGS = 2
KERNEL_PATTERN = (
    "GPU HANG|Resetting chip|context reset|GPU reset|drm.*(error|ERROR)|"
    "Out of memory|oom-kill|iwlwifi.*(error|Error|fail|restart)|"
    "NETDEV WATCHDOG|thermal.*thrott"
)
GRAPHICS_LIBRARIES = re.compile(
    r"sapien|vulkan|openvr|vrclient|libGL|libEGL|libdrm|libnvidia|libva|libstdc\+\+",
    re.IGNORECASE,
)
LOG_BYTES = 64 * 1024
RESOURCE_BYTES = 128 * 1024
CLIENT_RESOURCE_BYTES = 128 * 1024
CALL_STATE = struct.Struct("<Qdd64s96sBQ")
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEBUG_ROOT = REPOSITORY_ROOT / "vr_debug"


def validate_output_dir(path: Path) -> Path:
    """Resolve and constrain observer writes to this repository's vr_debug/."""
    root = DEBUG_ROOT.absolute()
    if DEBUG_ROOT.resolve() != root:
        raise ValueError("vr_debug output root must not be a symbolic link")
    output = Path(path).resolve()
    try:
        output.relative_to(root)
    except ValueError as exc:
        raise ValueError("observer output must stay inside repository vr_debug/") from exc
    if output == root:
        raise ValueError("observer output must use a run subdirectory inside vr_debug/")
    return output


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class BoundedText:
    """Keep complete recent lines, with an explicit count of discarded bytes."""

    def __init__(self, limit: int) -> None:
        self.limit = limit
        self.lines: deque[bytes] = deque()
        self.size = 0
        self.discarded_bytes = 0

    def add(self, text: str) -> None:
        for line in text.splitlines():
            data = (line + "\n").encode("utf-8", errors="replace")
            if len(data) > self.limit:
                self.discarded_bytes += len(data) - self.limit
                data = data[-self.limit:]
            self.lines.append(data)
            self.size += len(data)
            while self.size > self.limit:
                removed = self.lines.popleft()
                self.size -= len(removed)
                self.discarded_bytes += len(removed)

    def write(self, path: Path) -> None:
        # Leave the payload limit intact: metadata lives in collector.json.
        path.write_bytes(b"".join(self.lines))


def line_time(line: str) -> float | None:
    """Parse SteamVR timestamps and common ISO timestamps from ALVR logs."""
    try:
        iso = re.match(
            r"^\[?(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:\d{2})?)",
            line,
        )
        if iso:
            value = iso.group(1)
            if value.endswith("Z"):
                value = value[:-1] + "+00:00"
            return datetime.fromisoformat(value).timestamp()
        if line.startswith("["):
            return datetime.strptime(line[1:20], "%Y-%m-%d %H:%M:%S").timestamp()
        try:
            return datetime.strptime(line[:31], "%a %b %d %Y %H:%M:%S.%f").timestamp()
        except ValueError:
            return datetime.strptime(line[:24], "%a %b %d %Y %H:%M:%S").timestamp()
    except ValueError:
        return None


class LogCursor:
    """Read new bytes, including an inode moved to SteamVR's .previous file."""

    def __init__(
        self, path: Path, started_at: float, limit: int = LOG_BYTES,
        *, pattern: re.Pattern[str] = KEY_LINES,
    ) -> None:
        self.path = path
        self.started_at = started_at
        self.limit = limit
        self.pattern = pattern
        self.inode: tuple[int, int] | None = None
        self.offset = 0
        self.tail = BoundedText(limit)
        self.skipped_input_bytes = 0
        self.rotations = 0
        self._last_in_scope = False
        self._pending = b""
        self._recent: deque[str] = deque(maxlen=LOG_CONTEXT_BEFORE)
        self._context_remaining = 0
        self.available = False
        self.unavailable_reason: str | None = None

    def _retain(self, line: str, in_scope: bool) -> None:
        if not in_scope:
            self._recent.clear()
            self._context_remaining = 0
            return
        if self.pattern.search(line):
            for context in self._recent:
                self.tail.add("CONTEXT_BEFORE " + context)
            self.tail.add("EVENT " + line)
            self._context_remaining = LOG_CONTEXT_AFTER
        elif self._context_remaining:
            self.tail.add("CONTEXT_AFTER " + line)
            self._context_remaining -= 1
        self._recent.append(line)

    def _read(self, path: Path, offset: int, *, initial: bool) -> int:
        with path.open("rb") as handle:
            size = os.fstat(handle.fileno()).st_size
            start = max(offset, size - self.limit)
            self.skipped_input_bytes += max(0, start - offset)
            handle.seek(start)
            data = handle.read(self.limit)
            end = handle.tell()
        if start > offset:
            self._pending = b""
            if b"\n" in data:
                data = data.split(b"\n", 1)[1]
        data = self._pending + data
        complete, _, self._pending = data.rpartition(b"\n")
        # A partial write should not split an error keyword across two polls.
        if len(self._pending) > 4096:
            self.skipped_input_bytes += len(self._pending) - 4096
            self._pending = self._pending[-4096:]
        for line in complete.decode("utf-8", errors="replace").splitlines():
            stamp = line_time(line)
            if stamp is not None:
                self._last_in_scope = stamp >= int(self.started_at)
            # For byte increments with no timestamp, retain error continuations;
            # initial snapshots require a preceding timestamp in this run.
            # Preserve a small bounded window around errors, lifecycle changes,
            # watchdogs and controller state transitions.
            self._retain(line, self._last_in_scope or not initial)
        return end

    def poll(self) -> None:
        try:
            stat = self.path.stat()
        except FileNotFoundError:
            return
        if not stat_module.S_ISREG(stat.st_mode):
            self.unavailable_reason = "not_regular_file"
            return
        self.available = True
        identity = (stat.st_dev, stat.st_ino)
        if self.inode is None:
            self.offset = self._read(self.path, 0, initial=True)
        elif identity != self.inode or stat.st_size < self.offset:
            previous = self.path.with_name(self.path.stem + ".previous" + self.path.suffix)
            try:
                old = previous.stat()
                if (old.st_dev, old.st_ino) == self.inode and old.st_size >= self.offset:
                    self._read(previous, self.offset, initial=False)
            except FileNotFoundError:
                pass
            self.rotations += 1
            self._last_in_scope = False
            self._pending = b""
            self._recent.clear()
            self._context_remaining = 0
            self.offset = self._read(self.path, 0, initial=True)
        elif stat.st_size > self.offset:
            self.offset = self._read(self.path, self.offset, initial=False)
        self.inode = identity


def process_stat(pid: int, proc_root: Path = Path("/proc")) -> dict[str, Any]:
    text = (proc_root / str(pid) / "stat").read_text()
    # comm may contain whitespace or parentheses; the final ')' ends it.
    end = text.rfind(")")
    fields = text[end + 2:].split()
    return {
        "state": fields[0], "ppid": int(fields[1]),
        "cpu_ticks": int(fields[11]) + int(fields[12]),
        "priority": int(fields[15]), "nice": int(fields[16]),
        "threads": int(fields[17]), "start_token": fields[19],
        "rss_bytes": int(fields[21]) * os.sysconf("SC_PAGE_SIZE"),
    }


def process_resources(pid: int, proc_root: Path = Path("/proc")) -> dict[str, Any]:
    sample = process_stat(pid, proc_root)
    sample["wall_time"] = utc_now()
    sample["monotonic_s"] = time.monotonic()
    base = proc_root / str(pid)
    for name, key in (("wchan", "wchan"), ("schedstat", "schedstat")):
        try:
            sample[key] = (base / name).read_text().strip()[:256]
        except OSError:
            pass
    try:
        for line in (base / "status").read_text().splitlines():
            if line.startswith(("voluntary_ctxt_switches:", "nonvoluntary_ctxt_switches:", "VmSwap:",
                                "SigPnd:", "ShdPnd:", "SigBlk:")):
                key, value = line.split(":", 1)
                sample[key] = value.strip()
    except OSError:
        pass
    try:
        sample["memory_pressure"] = (proc_root / "pressure/memory").read_text()[:512].strip()
    except OSError:
        pass
    return sample


def graphics_maps(pid: int, proc_root: Path = Path("/proc")) -> list[str]:
    result = set()
    for line in (proc_root / str(pid) / "maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and fields[5].startswith("/") and GRAPHICS_LIBRARIES.search(fields[5]):
            result.add(fields[5])
    return sorted(result)[:128]


class Collector:
    def __init__(
        self, pid: int, output_dir: Path, parent_start: str, started_at: float,
        *, log_dir: Path | None = None, proc_root: Path = Path("/proc"),
        sys_root: Path = Path("/sys"),
    ) -> None:
        self.pid = pid
        self.output_dir = validate_output_dir(output_dir)
        self.parent_start = parent_start
        self.started_at = started_at
        self.proc_root = proc_root
        self.resources = BoundedText(RESOURCE_BYTES)
        self.client_resources = BoundedText(CLIENT_RESOURCE_BYTES)
        self.client_sampler = ClientResourceSampler(pid, proc_root=proc_root, sys_root=sys_root)
        self.kernel = BoundedText(LOG_BYTES)
        self.libraries: list[str] = []
        self.errors: dict[str, dict[str, Any]] = {}
        log_dir = log_dir or Path.home() / ".local/share/Steam/logs"
        self.logs = {name: LogCursor(log_dir / name, started_at) for name in STEAM_LOGS}
        alvr_log = os.environ.get("VR_DEBUG_ALVR_LOG", "").strip()
        self.alvr_log_configured = bool(alvr_log)
        if alvr_log:
            self.logs["runtime.log"] = LogCursor(
                Path(alvr_log).expanduser(), started_at, pattern=ALVR_LINES
            )
        self.finished_reason = "running"
        self.kernel_cursor = started_at
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def error(self, source: str, exc: BaseException) -> None:
        previous = self.errors.get(source, {})
        self.errors[source] = {
            "count": previous.get("count", 0) + 1,
            "last": f"{type(exc).__name__}: {exc}"[:400],
        }

    def _discover_alvr_logs(self, processes: list[dict[str, Any]]) -> None:
        """Follow only regular log files already open by ALVR processes."""
        if self.alvr_log_configured:
            return
        existing = {name for name in self.logs if name.startswith("runtime-auto-")}
        if len(existing) >= MAX_ALVR_LOGS:
            return
        for process in processes:
            if "alvr" not in str(process.get("comm", "")).lower():
                continue
            pid = process.get("pid")
            if not isinstance(pid, int):
                continue
            fd_root = self.proc_root / str(pid) / "fd"
            try:
                fds = sorted(fd_root.iterdir(), key=lambda path: int(path.name))
            except (OSError, ValueError):
                continue
            for fd in fds[:128]:
                if len(existing) >= MAX_ALVR_LOGS:
                    return
                try:
                    target = os.readlink(fd)
                    if target.endswith(" (deleted)"):
                        continue
                    candidate = Path(target)
                    if not candidate.is_absolute():
                        continue
                    basename = candidate.name.lower()
                    if candidate.suffix.lower() not in {".log", ".txt"}:
                        continue
                    if not any(token in basename for token in ("alvr", "server", "runtime", "log")):
                        continue
                    info = candidate.stat()
                    if not stat_module.S_ISREG(info.st_mode):
                        continue
                except (OSError, ValueError):
                    continue
                key = f"runtime-auto-{pid}-{fd.name}.log"
                if key not in self.logs:
                    self.logs[key] = LogCursor(candidate, self.started_at, pattern=ALVR_LINES)
                    existing.add(key)
                    break

    def observe(self) -> bool:
        try:
            snapshot = self.client_sampler.sample()
            self.client_resources.add(json.dumps(snapshot, separators=(",", ":")))
            self._discover_alvr_logs(snapshot.get("processes", []))
        except Exception as exc:
            self.error("client_resources", exc)
        try:
            sample = process_resources(self.pid, self.proc_root)
        except (OSError, ValueError, IndexError) as exc:
            self.finished_reason = "parent_unavailable"
            self.error("parent", exc)
            return False
        if sample["start_token"] != self.parent_start:
            self.finished_reason = "pid_reused"
            return False
        try:
            data = (self.output_dir / "last-call.bin").read_bytes()
            seq, started, updated, component, stage, active, end_seq = CALL_STATE.unpack(data)
            if seq and seq == end_seq and not seq % 2:
                sample["last_call"] = {
                    "component": component.rstrip(b"\0").decode(errors="replace"),
                    "stage": stage.rstrip(b"\0").decode(errors="replace"),
                    "call_active": bool(active), "started_monotonic_s": started,
                    "updated_wall_time": updated,
                    "elapsed_ms": max(0, (time.monotonic() - started) * 1000) if active else None,
                }
        except (OSError, ValueError, struct.error):
            pass
        self.resources.add(json.dumps(sample, separators=(",", ":")))
        if sample["state"] == "Z":
            self.finished_reason = "parent_zombie"
            return False
        return True

    def collect_logs(self) -> None:
        for name, cursor in self.logs.items():
            try:
                cursor.poll()
            except (OSError, ValueError) as exc:
                self.error(name, exc)
        try:
            if process_stat(self.pid, self.proc_root)["start_token"] == self.parent_start:
                self.libraries = graphics_maps(self.pid, self.proc_root)
        except (OSError, ValueError, IndexError):
            # Keep the most recent maps after the parent has exited.
            pass

    def collect_kernel(self) -> None:
        until = time.time()
        try:
            result = subprocess.run(
                ["journalctl", "-k", "--since", f"@{self.kernel_cursor:.3f}",
                 "--until", f"@{until:.3f}", "--no-pager", "-n", "200",
                 "-o", "short-iso-precise", "-g", KERNEL_PATTERN],
                capture_output=True, text=True, errors="replace", timeout=3.0,
                check=False,
            )
            if result.returncode == 0 or (result.returncode == 1 and not result.stderr.strip()):
                if result.stdout.strip() != "-- No entries --":
                    self.kernel.add(result.stdout[-LOG_BYTES:])
                self.kernel_cursor = until
            else:
                self.error("kernel", RuntimeError(result.stderr.strip()[:300] or f"exit {result.returncode}"))
        except (OSError, subprocess.SubprocessError) as exc:
            self.error("kernel", exc)

    def flush(self) -> None:
        self.resources.write(self.output_dir / "process-resources.jsonl")
        self.client_resources.write(self.output_dir / "client-resources.jsonl")
        self.kernel.write(self.output_dir / "kernel-errors.log")
        limited_libraries = []
        for path in self.libraries:
            candidate = json.dumps(limited_libraries + [path], ensure_ascii=False, indent=2)
            if len(candidate.encode("utf-8")) > 16 * 1024 - 1:
                break
            limited_libraries.append(path)
        (self.output_dir / "graphics-libraries.json").write_text(
            json.dumps(limited_libraries, ensure_ascii=False, indent=2) + "\n"
        )
        for name, cursor in self.logs.items():
            prefix = "steamvr-" if name in STEAM_LOGS else "alvr-"
            cursor.tail.write(self.output_dir / (prefix + name))
        metadata = {
            "pid": self.pid, "parent_start_token": self.parent_start,
            "started_at": self.started_at, "updated_at": utc_now(),
            "state": self.finished_reason,
            "scope": "read-only observer; SteamVR and optional ALVR event context since run start; PC times, not Quest sensor times",
            "limits": {"resource_bytes": RESOURCE_BYTES,
                       "client_resource_bytes": CLIENT_RESOURCE_BYTES,
                       "client_sample_bytes": 12 * 1024,
                       "bytes_per_log": LOG_BYTES,
                       "log_context_before_lines": LOG_CONTEXT_BEFORE,
                       "log_context_after_lines": LOG_CONTEXT_AFTER,
                       "max_alvr_logs": MAX_ALVR_LOGS,
                       "resources_sample_s": 2, "log_sample_s": 10,
                       "client_resources_sample_s": 2, "max_runtime_s": 8 * 3600},
            "discarded_resource_bytes": self.resources.discarded_bytes,
            "discarded_client_resource_bytes": self.client_resources.discarded_bytes,
            "client_resource_fields": (
                "host CPU/iowait/load, memory/swap/page faults, PSI pressure, DRM/network/wireless "
                "counters, thermal/frequency hints, selected process CPU/RSS/I/O/scheduler data, "
                "and bounded suspicious thread waits with best-effort kernel stacks"
            ),
            "logs": {name: {"discarded_bytes": c.tail.discarded_bytes,
                            "skipped_input_bytes": c.skipped_input_bytes,
                            "rotations": c.rotations, "available": c.available,
                            "unavailable_reason": c.unavailable_reason}
                     for name, c in self.logs.items()},
            "alvr_log_configured": self.alvr_log_configured,
            "alvr_log_discovery": (
                "explicit_path" if self.alvr_log_configured else
                "open_regular_file_detected" if any(name.startswith("runtime-auto-") for name in self.logs) else
                "no_open_log_detected"
            ),
            "errors": self.errors,
        }
        (self.output_dir / "collector.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")

    def run(self, max_runtime_s: float = 8 * 3600) -> None:
        deadline = time.monotonic() + max_runtime_s
        next_logs = next_kernel = 0.0
        while time.monotonic() < deadline:
            alive = self.observe()
            now = time.monotonic()
            if not alive or now >= next_logs:
                self.collect_logs()
                next_logs = now + 10.0
                self.flush()
            if not alive or now >= next_kernel:
                self.collect_kernel()
                next_kernel = now + 60.0
            if not alive:
                # Kernel/compositor error handling may trail parent teardown.
                time.sleep(2.0)
                self.collect_logs()
                self.collect_kernel()
                self.flush()
                return
            time.sleep(2.0)
        self.finished_reason = "runtime_limit"
        self.collect_logs()
        self.collect_kernel()
        self.flush()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--parent-start", required=True)
    parser.add_argument("--started-at", type=float, default=None)
    args = parser.parse_args(argv)
    marker = None
    try:
        output_dir = validate_output_dir(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        marker = output_dir / f".active-observer-{os.getpid()}"
        marker.write_text(process_stat(os.getpid())["start_token"])
        Collector(args.pid, output_dir, args.parent_start,
                  time.time() if args.started_at is None else args.started_at).run()
    except Exception:
        # Diagnostics must never require parent-process cleanup or recovery.
        return
    finally:
        if marker is not None:
            try:
                marker.unlink(missing_ok=True)
            except OSError:
                pass


if __name__ == "__main__":
    main()
