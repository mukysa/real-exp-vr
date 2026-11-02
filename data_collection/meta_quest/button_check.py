"""Read-only Touch button monitor; OpenVR input is the default backend."""

from __future__ import annotations

import argparse
from collections import defaultdict
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable

from .debug_recording import DebugChannel, get_debug_recorder
from .steamvr_bridge import (
    BUTTON_A, BUTTON_B, BUTTON_GRIP, BUTTON_TRIGGER, CONTROLLERS,
    _configure_sapien_vr, _load_sapien, alvr_is_streaming,
)


class _Diagnostics:
    """Time the existing calls; the watcher never calls a VR API."""

    def __init__(self, stall_s: float, input_gap_s: float, debug: DebugChannel | None = None) -> None:
        self.debug = debug
        self.stall_s = stall_s
        self.input_gap_s = input_gap_s
        self.started = time.monotonic()
        self.frame = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._active: tuple[str, float] | None = None
        self._last_warning = float("-inf")
        self._stats: dict[str, list[float]] = defaultdict(lambda: [0, 0.0, 0.0])
        self._last_reads: dict[str, float] = {}
        self._last_loop: float | None = None
        self._max_loop_gap = 0.0
        self._report_start = self.started
        self._report_frames = 0
        self._thread = threading.Thread(target=self._watch, daemon=True, name="quest-button-diagnostics")

    def emit(self, message: str) -> None:
        if self.debug is not None and not message.startswith("TIMING "):
            self.debug.event("diagnostic", message=message, frame=self.frame,
                             stage=message.split(" ", 1)[0])
        print(
            f"[{time.strftime('%H:%M:%S')} +{time.monotonic() - self.started:.3f}s "
            f"frame={self.frame}] {message}",
            flush=True,
        )

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=0.5)

    def _watch(self) -> None:
        while not self._stop.wait(0.1):
            self._check_stall()

    def _check_stall(self) -> None:
        now = time.monotonic()
        with self._lock:
            active = self._active
            if active is None or now - active[1] < self.stall_s or now - self._last_warning < 1.0:
                return
            self._last_warning = now
        self.emit(f"STALL stage={active[0]} elapsed_ms={(now - active[1]) * 1000:.1f} call_not_returned=true")

    def call(self, stage: str, function: Callable[..., Any], *args: Any) -> Any:
        started = time.monotonic()
        with self._lock:
            self._active = (stage, started)
            self._last_warning = float("-inf")
        try:
            if self.debug is not None:
                return self.debug.call(stage, function, *args)
            return function(*args)
        finally:
            elapsed = time.monotonic() - started
            with self._lock:
                self._active = None
                stats = self._stats[stage]
                stats[0] += 1
                stats[1] += elapsed
                stats[2] = max(stats[2], elapsed)
            if elapsed >= self.stall_s:
                self.emit(f"SLOW stage={stage} duration_ms={elapsed * 1000:.1f} call_returned=true")

    def begin_frame(self) -> None:
        now = time.monotonic()
        self.frame += 1
        if self._last_loop is not None:
            gap = now - self._last_loop
            self._max_loop_gap = max(self._max_loop_gap, gap)
            if gap > self.input_gap_s:
                self.emit(f"LOOP_GAP gap_s={gap:.3f} threshold_s={self.input_gap_s:g}")
        self._last_loop = now

    def read_completed(self, side: str) -> None:
        now = time.monotonic()
        previous = self._last_reads.get(side)
        self._last_reads[side] = now
        if previous is not None and now - previous > self.input_gap_s:
            self.emit(f"READ_GAP side={side} gap_s={now - previous:.3f} threshold_s={self.input_gap_s:g}")

    def report(self) -> None:
        now = time.monotonic()
        elapsed = now - self._report_start
        stages = " ".join(
            f"{name}={total / count * 1000:.1f}/{maximum * 1000:.1f}"
            for name, (count, total, maximum) in self._stats.items()
        )
        self.emit(
            f"TIMING window_s={elapsed:.2f} loops_hz={(self.frame - self._report_frames) / max(elapsed, 1e-9):.1f} "
            f"max_loop_gap_ms={self._max_loop_gap * 1000:.1f} stage_ms(avg/max): {stages}"
        )
        self._stats.clear()
        self._max_loop_gap = 0.0
        self._report_start = now
        self._report_frames = self.frame


def _read_trigger(display: Any, controller_id: int, mask: int, diagnostics: _Diagnostics, side: str) -> tuple[float, str, str]:
    """Keep the bridge's axis-1/digital fallback, but expose read failures."""
    try:
        state = diagnostics.call(f"{side}_axis", display.get_controller_axis_state, controller_id, 1)
        value = float(state[0])
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"axis[1].x is outside [0, 1]: {value!r}")
        return value, "axis1", "none"
    except Exception as exc:
        if diagnostics.debug is not None:
            diagnostics.debug.event("trigger_fallback", side=side, controller_id=controller_id,
                                    exception=type(exc).__name__, detail=exc, mask=mask)
        return (1.0 if mask & BUTTON_TRIGGER else 0.0), "button_fallback", f"{type(exc).__name__}: {exc}"


def _wait_for_alvr(session_path: Path) -> None:
    last_report = 0.0
    while not alvr_is_streaming(session_path):
        now = time.monotonic()
        if now - last_report >= 5.0:
            print("Waiting for ALVR Streaming...", flush=True)
            last_report = now
        time.sleep(0.5)


def _button_names(mask: int, side: str) -> str:
    names = []
    if mask & BUTTON_A:
        names.append("X" if side == "left" else "A")
    if mask & BUTTON_B:
        names.append("Y" if side == "left" else "B")
    if mask & BUTTON_GRIP:
        names.append("Grip")
    return ",".join(names) or "none"


def run(
    duration_s: float,
    poll_hz: float,
    alvr_session: str,
    no_alvr_wait: bool,
    *,
    report_interval_s: float = 2.0,
    stall_ms: float = 250.0,
    input_gap_s: float = 2.0,
    check_pose: bool = False,
    backend: str = "openvr",
) -> None:
    debug = get_debug_recorder("button_check")
    debug.event("run_start", duration_s=duration_s, poll_hz=poll_hz, check_pose=check_pose,
                alvr_session=alvr_session, no_alvr_wait=no_alvr_wait, backend=backend)
    if debug.output_dir is not None:
        print(f"VR diagnostics: {debug.output_dir}", flush=True)
    if not no_alvr_wait:
        debug.call("wait_for_alvr", _wait_for_alvr, Path(alvr_session).expanduser())
    diagnostics = _Diagnostics(stall_ms / 1000.0, input_gap_s, debug)
    diagnostics.start()
    try:
        # Include backend cleanup in the outer timing. For the legacy SAPIEN
        # backend this includes native Scene/RenderVRDisplay destruction;
        # the default input backend only has a balanced OpenVR shutdown.
        debug.call("monitor_scope", _monitor, duration_s, poll_hz,
                   report_interval_s, check_pose, diagnostics, backend=backend)
        debug.event("monitor_returned", reason="duration elapsed")
    except KeyboardInterrupt:
        debug.event("keyboard_interrupt_caught", frame=diagnostics.frame)
        diagnostics.emit(f"Stopping read-only {backend}/SteamVR button check.")
    except BaseException as exc:
        debug.event("run_exception", exception=type(exc).__name__, detail=exc)
        raise
    finally:
        diagnostics.close()
        diagnostics.report()
        debug.event("run_finally_complete", frame=diagnostics.frame)


def _monitor(
    duration_s: float,
    poll_hz: float,
    report_interval_s: float,
    check_pose: bool,
    diagnostics: _Diagnostics,
    *,
    backend: str = "openvr",
) -> None:
    if backend == "sapien":
        _monitor_sapien(duration_s, poll_hz, report_interval_s, check_pose, diagnostics)
    elif backend == "openvr":
        from .openvr_button_input import openvr_button_input

        diagnostics.emit("Read-only OpenVR background button check; no TCP/ROS/robot output.")
        diagnostics.emit("No SAPIEN scene, Vulkan rendering, compositor access, or frame submission.")
        diagnostics.emit("packet_unchanged_s measures state-packet changes, not tracking freshness.")
        diagnostics.emit("Unavailable input may require dismissing the SteamVR dashboard or fixing bindings.")
        diagnostics.emit("Optional poses use raw OpenVR coordinates; Ctrl-C stops the check.")
        with openvr_button_input(check_pose=check_pose) as display:
            _poll_controllers(display, display.prepare_frame, duration_s, poll_hz,
                              report_interval_s, check_pose, diagnostics)
    else:
        raise ValueError(f"Unknown button-check backend: {backend}")


def _monitor_sapien(
    duration_s: float,
    poll_hz: float,
    report_interval_s: float,
    check_pose: bool,
    diagnostics: _Diagnostics,
) -> None:
    sapien = diagnostics.call("sapien_import", _load_sapien)
    if diagnostics.debug is not None:
        diagnostics.debug.event("sapien_loaded", path=getattr(sapien, "__file__", None),
                                version=getattr(sapien, "__version__", None))
    explicit_vr_init = diagnostics.call("vr_configure", _configure_sapien_vr, sapien)
    mode = "explicit enable_vr()" if explicit_vr_init else "RenderVRDisplay constructor"
    diagnostics.emit("Read-only SAPIEN/SteamVR button check; no TCP/ROS/robot output.")
    diagnostics.emit(f"SteamVR input path: {mode}")
    diagnostics.emit("signal=OK means input calls returned; it does not prove fresh physical input.")
    diagnostics.emit(
        f"Diagnostics: report_interval_s={report_interval_s:g} stall_ms={diagnostics.stall_s * 1000:g} "
        f"input_gap_s={diagnostics.input_gap_s:g} poll_hz={poll_hz:g} check_pose={check_pose}"
    )
    diagnostics.emit("READ_GAP measures completed local reads, not remote tracking timestamps or ROS delivery.")
    diagnostics.emit("The stall watcher needs Python thread scheduling; a native call holding the GIL may delay its output.")
    diagnostics.emit("Press and release each Touch button; hold/release each trigger. Ctrl-C stops the check.")

    scene = diagnostics.call("scene_init", sapien.Scene)
    display = diagnostics.call("display_init", sapien.render.RenderVRDisplay)
    display.root_pose = sapien.Pose()
    diagnostics.call("set_scene", display.set_scene, scene)

    def prepare_frame() -> None:
        # Preserve the existing SAPIEN rendering/input refresh order.
        diagnostics.call("scene_update", scene.update_render)
        diagnostics.call("display_update", display.update_render)
        diagnostics.call("render", display.render)

    _poll_controllers(display, prepare_frame, duration_s, poll_hz,
                      report_interval_s, check_pose, diagnostics)


def _poll_controllers(
    display: Any,
    prepare_frame: Callable[[], None],
    duration_s: float,
    poll_hz: float,
    report_interval_s: float,
    check_pose: bool,
    diagnostics: _Diagnostics,
) -> None:
    last_ids: tuple[int, ...] | None = None
    previous: dict[str, tuple[Any, ...] | None] = {side: None for side in CONTROLLERS}
    deadline = None if duration_s == 0.0 else time.monotonic() + duration_s
    next_report = time.monotonic()
    try:
        while deadline is None or time.monotonic() < deadline:
            diagnostics.begin_frame()
            prepare_frame()
            ids = tuple(int(value) for value in diagnostics.call("controller_ids", display.get_controller_ids))
            controllers = getattr(display, "controllers", CONTROLLERS)
            report_due = time.monotonic() >= next_report
            ids_changed = ids != last_ids
            if ids_changed:
                diagnostics.emit(f"controllers={ids} side_ids={controllers}")
                last_ids = ids
            for side, controller_id in controllers.items():
                if controller_id not in ids:
                    if diagnostics.debug is not None:
                        diagnostics.debug.sample(f"{side}_input", frame=diagnostics.frame,
                                                 side=side, controller_id=controller_id, present=False)
                    if ids_changed or report_due or previous[side] is not None:
                        diagnostics.emit(f"{side}: signal=MISSING controller_id={controller_id}")
                    previous[side] = None
                    continue
                try:
                    mask = int(diagnostics.call(f"{side}_buttons", display.get_controller_button_pressed, controller_id))
                except Exception as exc:
                    if diagnostics.debug is not None:
                        diagnostics.debug.sample(f"{side}_input", frame=diagnostics.frame, side=side,
                                                 controller_id=controller_id, present=True,
                                                 button_error=f"{type(exc).__name__}: {exc}")
                    error = ("button_error", type(exc).__name__, str(exc))
                    if report_due or previous[side] != error:
                        diagnostics.emit(f"{side}: signal=ERROR button_read={type(exc).__name__}: {exc}")
                    previous[side] = error
                    continue
                trigger, trigger_source, axis_error = _read_trigger(display, controller_id, mask, diagnostics, side)
                if trigger_source == "axis1":
                    diagnostics.read_completed(side)
                pose_status = "unchecked"
                if check_pose:
                    try:
                        pose = diagnostics.call(f"{side}_pose", display.get_controller_pose, controller_id)
                        values = (*pose.p, *pose.q)
                        if len(values) != 7 or not all(math.isfinite(float(value)) for value in values):
                            raise ValueError("pose must contain seven finite values")
                        if sum(float(value) ** 2 for value in pose.q) <= 1e-12:
                            raise ValueError("pose quaternion has zero norm")
                        pose_status = "FINITE"
                    except Exception as exc:
                        pose_status = f"ERROR({type(exc).__name__}: {exc})"
                input_available = getattr(display, "input_available", True)
                details = display.input_details(controller_id) if hasattr(display, "input_details") else {}
                state = (mask, trigger, trigger_source, axis_error, pose_status, input_available)
                old = previous[side]
                if diagnostics.debug is not None and (old is None or old[0] != mask):
                    diagnostics.debug.event(
                        "button_state_changed", side=side, controller_id=controller_id,
                        frame=diagnostics.frame, mask=mask, pressed=mask,
                        buttons=_button_names(mask, side),
                    )
                if (
                    report_due or old is None or old[0] != mask
                    or abs(trigger - old[1]) >= 0.05 or old[2:] != state[2:]
                ):
                    signal = "OK" if trigger_source == "axis1" and not pose_status.startswith("ERROR") else "DEGRADED"
                    if not input_available:
                        signal = "UNAVAILABLE"
                    extra = "".join(f" {key}={value}" for key, value in details.items())
                    diagnostics.emit(
                        f"{side}: signal={signal} mask=0x{mask:x} pressed={_button_names(mask, side)} "
                        f"trigger={trigger:.2f} trigger_source={trigger_source} axis_error={axis_error!r} pose={pose_status}{extra}"
                    )
                previous[side] = state
                if diagnostics.debug is not None:
                    diagnostics.debug.sample(
                        f"{side}_input", frame=diagnostics.frame, side=side,
                        controller_id=controller_id, present=True, mask=mask, trigger=trigger,
                        trigger_source=trigger_source, axis_error=axis_error, pose_status=pose_status,
                        **details,
                        **({"position": list(values[:3]), "quaternion_wxyz": list(values[3:])}
                           if check_pose and pose_status == "FINITE" else {}),
                    )
            if report_due:
                diagnostics.report()
                next_report = time.monotonic() + report_interval_s
            # Keep the original additional sleep; poll_hz is an upper bound.
            diagnostics.call("poll_sleep", time.sleep, 1.0 / poll_hz)
    finally:
        if diagnostics.debug is not None:
            exception = sys.exc_info()[0]
            diagnostics.debug.event(
                "monitor_scope_exit", frame=diagnostics.frame,
                exception=None if exception is None else exception.__name__,
                note="Input loop exited; backend cleanup may follow",
            )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Read-only SteamVR Touch button check (input-only OpenVR by default)")
    parser.add_argument("--backend", choices=("openvr", "sapien"), default="openvr",
                        help="openvr avoids rendering; sapien retains the legacy VR renderer for diagnostics")
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--poll-hz", type=float, default=60.0)
    parser.add_argument("--alvr-session", default="~/.config/alvr/session.json")
    parser.add_argument("--no-alvr-wait", action="store_true")
    parser.add_argument("--report-interval-s", type=float, default=2.0, help="status and timing summary interval")
    parser.add_argument("--stall-ms", type=float, default=250.0, help="warn when a stage takes this long")
    parser.add_argument("--input-gap-s", type=float, default=2.0, help="warn about gaps between local reads/loops")
    parser.add_argument("--check-pose", action="store_true", help="also read poses and check for nonfinite values")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_s) or args.duration_s < 0.0:
        parser.error("--duration-s must be non-negative")
    for name in ("poll_hz", "report_interval_s", "stall_ms", "input_gap_s"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    run(
        args.duration_s, args.poll_hz, args.alvr_session, args.no_alvr_wait,
        report_interval_s=args.report_interval_s, stall_ms=args.stall_ms,
        input_gap_s=args.input_gap_s, check_pose=args.check_pose, backend=args.backend,
    )


if __name__ == "__main__":
    main()
