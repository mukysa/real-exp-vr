"""Bounded, best-effort diagnostics; never changes input/control decisions.

Control threads only update small in-memory summaries or enqueue events. A
daemon writes at most two 256 KiB JSONL files per process. A separate observer
can preserve SteamVR/kernel evidence even while native VR code holds the GIL.
"""

from __future__ import annotations

import atexit
from collections import deque
from datetime import datetime, timezone
import json
import math
import mmap
import os
from pathlib import Path
from queue import Empty, Full, Queue
import re
import shutil
import signal
import socket
import subprocess
import struct
import sys
import threading
import time
from typing import Any, Callable


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEBUG_ROOT = REPOSITORY_ROOT / "vr_debug"
REPORT_INTERVAL_S = 2.0
MAX_FILE_BYTES = 256 * 1024
MAX_RUNS = 10
CALL_STATE = struct.Struct("<Qdd64s96sBQ")
_CLOCK = time.monotonic
_WALL_NS = time.time_ns
_CHANNELS: dict[str, "DebugChannel"] = {}
_RECORDER: "_Recorder | None" = None
_INITIALIZED = False
_INIT_LOCK = threading.Lock()


def _safe(value: Any, depth: int = 0) -> Any:
    """Bound size, detach samples, and represent NaN without invalid JSON."""
    if depth > 5:
        return "<depth limit>"
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, str):
        return value[:2048]
    if isinstance(value, BaseException):
        try:
            return str(value)[:2048]
        except BaseException:
            return f"<{type(value).__name__}: message unavailable>"
    if isinstance(value, dict):
        return {str(k)[:96]: _safe(v, depth + 1) for k, v in list(value.items())[:64]}
    if isinstance(value, (list, tuple, deque)):
        return [_safe(v, depth + 1) for v in list(value)[:64]]
    # numpy scalars/arrays are converted by callers, without importing numpy.
    return repr(value)[:1024]


def _read_text(path: Path, limit: int = 8192) -> str:
    try:
        with path.open("r", errors="replace") as stream:
            return stream.read(limit)
    except OSError as exc:
        return f"<{type(exc).__name__}: {exc}>"


def _start_token(pid: int) -> str:
    # /proc/stat comm may contain spaces or parentheses.
    return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]


def _run_is_active(run_dir: Path) -> bool:
    for marker in run_dir.glob("**/.active-*"):
        try:
            pid = int(marker.name.rsplit("-", 1)[1])
            expected = marker.read_text().strip()
            if not expected or expected == _start_token(pid):
                # Check /proc even for older empty markers.
                _start_token(pid)
                return True
        except (OSError, ValueError, IndexError):
            pass
        try:
            marker.unlink(missing_ok=True)
        except OSError:
            pass
    return False


def _prepare_run() -> Path:
    if DEBUG_ROOT.resolve() != DEBUG_ROOT.absolute():
        raise OSError("Diagnostic root must not be a symbolic link")
    DEBUG_ROOT.mkdir(parents=True, exist_ok=True)
    supplied = os.environ.get("VR_DEBUG_RUN_ID", "")
    run_id = re.sub(r"[^a-zA-Z0-9_.-]", "_", supplied)[:100]
    if not run_id or run_id in {".", ".."}:
        run_id = datetime.now().strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}-{time.time_ns() % 1000000:06d}"
    run_dir = DEBUG_ROOT / run_id
    if run_dir.is_symlink() or run_dir.resolve().parent != DEBUG_ROOT.resolve():
        raise OSError("Diagnostic run must stay inside vr_debug")
    run_dir.mkdir(exist_ok=True)
    (run_dir / ".managed-vr-debug").touch(exist_ok=True)
    # Only prune our marked directories, never other user files in vr_debug/.
    # Active observers leave .active-* markers; never remove their run.
    runs = sorted(
        (p for p in DEBUG_ROOT.iterdir() if not p.is_symlink() and p.is_dir() and (p / ".managed-vr-debug").is_file()),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    for old in runs[MAX_RUNS:]:
        if old != run_dir and not _run_is_active(old):
            shutil.rmtree(old, ignore_errors=True)
    return run_dir


class DebugChannel:
    """A component label sharing one bounded writer in each process."""

    def __init__(self, component: str, recorder: "_Recorder | None") -> None:
        self.component = component
        self._recorder = recorder

    @property
    def output_dir(self) -> Path | None:
        return None if self._recorder is None else self._recorder.output_dir

    def event(self, name: str, **fields: Any) -> None:
        if self._recorder is not None:
            self._recorder.event(self.component, name, fields)

    def sample(self, key: str, **fields: Any) -> None:
        if self._recorder is not None:
            self._recorder.sample(self.component, key, fields)

    def call(self, stage: str, function: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        if self._recorder is None:
            return function(*args, **kwargs)
        recorder = self._recorder
        started = _CLOCK()
        token = (threading.get_ident(), self.component, stage)
        recorder.begin_call(token, started)
        try:
            return function(*args, **kwargs)
        except BaseException as exc:
            self.event("call_exception", stage=stage, exception=type(exc).__name__, detail=exc)
            raise
        finally:
            recorder.end_call(token, started)

    def close(self) -> None:
        if self._recorder is not None:
            self._recorder.close()


class _Recorder:
    def __init__(self, component: str, *, run_dir: Path | None = None, observer: bool = True) -> None:
        self.pid = os.getpid()
        self.started_at = time.time()
        self.output_dir = (run_dir or _prepare_run()) / f"{component}-{self.pid}"
        if self.output_dir.is_symlink():
            raise OSError("Diagnostic process directory must not be a symbolic link")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._marker = self.output_dir / f".active-{self.pid}"
        self._marker.write_text(_start_token(self.pid))
        self._queue: Queue[dict[str, Any]] = Queue(maxsize=128)
        self._lock = threading.Lock()
        self._samples: dict[tuple[str, str], dict[str, Any]] = {}
        self._stats: dict[tuple[str, str], list[float]] = {}
        self._active: dict[tuple[int, str, str], float] = {}
        self._rates: dict[tuple[str, ...], float] = {}
        self._dropped = 0
        self._suppressed = 0
        self._stop = threading.Event()
        self._closed = False
        self._file = None
        self._file_bytes = 0
        self._signals: deque[tuple[int, int, int]] = deque(maxlen=8)
        self._old_handlers: dict[int, Any] = {}
        self._manifest_component = component
        self._observer_allowed = observer
        self._observer_started = False
        self._call_state = None
        self._call_sequence = 0
        try:
            # Shared memory exposes the last call even if native code holds the
            # GIL. Updates dirty one page; no per-frame write()/flush()/fsync().
            with (self.output_dir / "last-call.bin").open("w+b") as state_file:
                state_file.truncate(CALL_STATE.size)
                self._call_state = mmap.mmap(state_file.fileno(), CALL_STATE.size)
        except OSError:
            pass
        self._thread = threading.Thread(target=self._write_loop, name="vr-debug-writer", daemon=True)
        self._thread.start()
        atexit.register(self.close)

    def event(self, component: str, name: str, fields: dict[str, Any]) -> None:
        try:
            now = _CLOCK()
            # Keep different state transitions distinct, but cap repeat storms.
            key = (component, name, *(str(fields.get(k, ""))[:96] for k in ("stage", "side", "state", "pressed", "clutch")))
            with self._lock:
                if self._closed:
                    return
                if name != "button_state_changed" and now - self._rates.get(key, -math.inf) < 0.5:
                    self._suppressed += 1
                    return
                if len(self._rates) >= 128:
                    self._rates.clear()
                self._rates[key] = now
            item = {"event": name, "component": component, "wall_time_ns": _WALL_NS(),
                    "monotonic_s": now, "fields": _safe(fields)}
            try:
                self._queue.put_nowait(item)
            except Full:
                self._dropped += 1
        except Exception:
            pass

    def sample(self, component: str, key: str, fields: dict[str, Any]) -> None:
        try:
            item = {"event": "sample", "component": component, "key": key,
                    "wall_time_ns": _WALL_NS(), "monotonic_s": _CLOCK(), "fields": _safe(fields)}
            with self._lock:
                if not self._closed and (len(self._samples) < 64 or (component, key) in self._samples):
                    self._samples[component, key] = item
        except Exception:
            pass

    def begin_call(self, token: tuple[int, str, str], started: float) -> None:
        try:
            with self._lock:
                if len(self._active) < 64:
                    self._active[token] = started
                self._publish_call(token, started, True)
        except Exception:
            pass

    def end_call(self, token: tuple[int, str, str], started: float) -> None:
        try:
            elapsed = max(0.0, _CLOCK() - started)
            with self._lock:
                self._active.pop(token, None)
                # Restore the still-active outer phase after a nested API call
                # returns, so the sidecar does not mislabel destructor waits
                # as the last (already completed) inner operation.
                remaining = max(self._active.items(), key=lambda item: item[1], default=None)
                if remaining is None:
                    self._publish_call(token, started, False)
                else:
                    self._publish_call(remaining[0], remaining[1], True)
                key = token[1:]
                if len(self._stats) < 64 or key in self._stats:
                    stats = self._stats.setdefault(key, [0, 0.0, 0.0, 0])
                    stats[0] += 1
                    stats[1] += elapsed
                    stats[2] = max(stats[2], elapsed)
                    stats[3] += int(elapsed >= 0.25)
            if elapsed >= 0.25:
                self.event(token[1], "slow_call", {"stage": token[2], "duration_ms": elapsed * 1000})
        except Exception:
            pass

    def _publish_call(self, token: tuple[int, str, str], started: float, active: bool) -> None:
        if self._call_state is None:
            return
        try:
            self._call_sequence += 2
            sequence = self._call_sequence
            self._call_state[:8] = struct.pack("<Q", sequence - 1)
            data = CALL_STATE.pack(sequence, started, time.time(), token[1].encode()[:64],
                                   token[2].encode()[:96], active, sequence)
            self._call_state[8:] = data[8:]
            self._call_state[:8] = data[:8]
        except Exception:
            pass

    def install_signal_notes(self) -> None:
        """Delegate immediately to the original handler; do no I/O/locking here."""
        if threading.current_thread() is not threading.main_thread():
            return
        for signum in (signal.SIGINT, signal.SIGTERM):
            old = signal.getsignal(signum)
            # Do not turn default SIGTERM into a Python exception or delay it.
            if not callable(old) or signum in self._old_handlers:
                continue
            self._old_handlers[signum] = old

            def note(sig: int, frame: Any, previous: Any = old) -> Any:
                self._signals.append((sig, _WALL_NS(), time.monotonic_ns()))
                return previous(sig, frame)

            signal.signal(signum, note)

    def start_observer(self) -> None:
        if self._observer_started or not self._observer_allowed or os.environ.get("VR_DEBUG_CAPTURE", "1") == "0":
            return
        self._observer_started = True
        try:
            subprocess.Popen(
                [sys.executable, "-m", "data_collection.meta_quest.debug_capture",
                 "--pid", str(self.pid), "--output-dir", str(self.output_dir),
                 "--parent-start", _start_token(self.pid), "--started-at", str(self.started_at)],
                cwd=str(REPOSITORY_ROOT), stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True, close_fds=True,
            )
        except Exception as exc:
            self.event("diagnostics", "observer_unavailable", {"detail": str(exc)})

    def _manifest(self) -> None:
        environment = {k: v[:4096] for k, v in os.environ.items() if k in {
            "DISPLAY", "WAYLAND_DISPLAY", "XDG_SESSION_TYPE", "XDG_RUNTIME_DIR", "XAUTHORITY",
            "PYTHONPATH", "LD_LIBRARY_PATH", "CONDA_PREFIX", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
            "SAPIEN_SITE_PACKAGES", "STEAMVR_PYTHON", "OPENVR_SITE_PACKAGES", "VR_DEBUG_RUN_ID",
        } or k.startswith(("VK_", "MESA_", "LIBGL_"))}
        manifest = {"schema_version": 1, "pid": self.pid, "ppid": os.getppid(),
                    "component": self._manifest_component, "hostname": socket.gethostname(),
                    "wall_time_ns": _WALL_NS(), "monotonic_ns": time.monotonic_ns(),
                    "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                    "boot_id": _read_text(Path("/proc/sys/kernel/random/boot_id"), 128).strip(),
                    "kernel": os.uname().release, "python": sys.executable, "python_version": sys.version,
                    "argv": sys.argv, "cwd": os.getcwd(), "repository": str(REPOSITORY_ROOT),
                    "environment": environment,
                    "limits": {"events_bytes": MAX_FILE_BYTES, "event_backups": 1, "summary_interval_s": REPORT_INTERVAL_S}}
        try:
            result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT,
                                    capture_output=True, text=True, timeout=1.0)
            manifest["git_head"] = result.stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            pass
        # This is a deliberately small diagnostic snapshot, not a full environment dump.
        payload = json.dumps(_safe(manifest), ensure_ascii=False, allow_nan=False, indent=2)
        if len(payload.encode()) > 65535:
            manifest["snapshot_omitted"] = "snapshot exceeded 64 KiB budget"
            payload = json.dumps(_safe(manifest), ensure_ascii=False, allow_nan=False, indent=2)
        if len(payload.encode()) > 65535:
            manifest["environment"] = {k[:96]: v[:256] for k, v in list(environment.items())[:16]}
            manifest["argv"] = [str(arg)[:256] for arg in sys.argv[:16]]
            manifest["snapshot_truncated"] = True
            payload = json.dumps(_safe(manifest), ensure_ascii=False, allow_nan=False, indent=2)
        if len(payload.encode()) > 65535:
            payload = json.dumps({
                "schema_version": 1, "pid": self.pid, "wall_time_ns": _WALL_NS(),
                "monotonic_ns": time.monotonic_ns(),
                "snapshot_omitted": "snapshot exceeded 64 KiB budget",
            })
        (self.output_dir / "manifest.json").write_text(payload + "\n")

    def _write(self, item: dict[str, Any]) -> None:
        payload = (json.dumps(item, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n").encode()
        if len(payload) > min(16384, MAX_FILE_BYTES):
            payload = (json.dumps({"event": "record_too_large", "bytes": len(payload)}) + "\n").encode()
        path = self.output_dir / "events.jsonl"
        if self._file is None:
            self._file = path.open("ab", buffering=0)
            self._file_bytes = path.stat().st_size
        if self._file_bytes + len(payload) > MAX_FILE_BYTES:
            self._file.close()
            path.replace(self.output_dir / "events.jsonl.1")
            self._file = path.open("wb", buffering=0)
            self._file_bytes = 0
        self._file.write(payload)
        self._file_bytes += len(payload)

    def _report(self) -> None:
        now = _CLOCK()
        with self._lock:
            samples, self._samples = self._samples, {}
            stats, self._stats = self._stats, {}
            active = [{"thread": t, "component": c, "stage": s, "elapsed_ms": (now - start) * 1000}
                      for (t, c, s), start in self._active.items() if now - start >= 0.25]
            dropped, self._dropped = self._dropped, 0
            suppressed, self._suppressed = self._suppressed, 0
        for item in samples.values():
            self._write(item)
        timings = {f"{c}.{s}": {"count": n, "avg_ms": total / n * 1000,
                                 "max_ms": maximum * 1000, "slow_calls": slow}
                   for (c, s), (n, total, maximum, slow) in stats.items()}
        self._write({"event": "summary", "wall_time_ns": _WALL_NS(), "monotonic_s": now,
                     "timings": timings, "active_calls": active,
                     "dropped_events": dropped, "suppressed_events": suppressed})
        while self._signals:
            sig, wall, mono = self._signals.popleft()
            self._write({"event": "python_signal_handler", "signal": sig, "wall_time_ns": wall,
                         "monotonic_ns": mono, "note": "Python handler time, not physical keypress time"})

    def _write_loop(self) -> None:
        try:
            try:
                self._manifest()
            except Exception as exc:
                self._write({"event": "manifest_error", "detail": str(exc)[:512]})
            self._write({"event": "process_start", "wall_time_ns": _WALL_NS(), "monotonic_s": _CLOCK(), "pid": self.pid})
            next_report = _CLOCK() + REPORT_INTERVAL_S
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    self._write(self._queue.get(timeout=0.1))
                except Empty:
                    pass
                if _CLOCK() >= next_report:
                    self._report()
                    next_report = _CLOCK() + REPORT_INTERVAL_S
            self._report()
            self._write({"event": "python_atexit", "wall_time_ns": _WALL_NS(), "monotonic_s": _CLOCK(),
                         "note": "Native/interpreter teardown and OS process exit may follow"})
        except Exception:
            # Diagnostics failing (including a full disk) must not stop control.
            pass
        finally:
            if self._file is not None:
                try:
                    self._file.close()
                except OSError:
                    pass

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=0.3)
        try:
            self._marker.unlink(missing_ok=True)
        except OSError:
            pass


def get_debug_recorder(component: str) -> DebugChannel:
    """Create diagnostics lazily; unavailable storage produces a no-op channel."""
    global _RECORDER, _INITIALIZED
    component = re.sub(r"[^a-zA-Z0-9_.-]", "_", component)[:80] or "process"
    with _INIT_LOCK:
        if not _INITIALIZED:
            _INITIALIZED = True
            if os.environ.get("VR_DEBUG_DISABLE", "0") != "1":
                try:
                    _RECORDER = _Recorder(component)
                except Exception:
                    _RECORDER = None
        channel = _CHANNELS.setdefault(component, DebugChannel(component, _RECORDER))
    if _RECORDER is not None and component == "button_check":
        try:
            _RECORDER.install_signal_notes()
            _RECORDER.start_observer()
        except Exception:
            pass
    return channel

