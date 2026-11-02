"""Read-only SAPIEN/SteamVR Touch button monitor."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from .steamvr_bridge import (
    BUTTON_A, BUTTON_B, BUTTON_GRIP, CONTROLLERS,
    _configure_sapien_vr, _load_sapien, _trigger_value, alvr_is_streaming,
)


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


def run(duration_s: float, poll_hz: float, alvr_session: str, no_alvr_wait: bool) -> None:
    sapien = _load_sapien()
    if not no_alvr_wait:
        _wait_for_alvr(Path(alvr_session).expanduser())
    explicit_vr_init = _configure_sapien_vr(sapien)
    mode = "explicit enable_vr()" if explicit_vr_init else "RenderVRDisplay constructor"
    print("Read-only SAPIEN/SteamVR button check; no TCP/ROS/robot output.", flush=True)
    print(f"SteamVR input path: {mode}", flush=True)
    print("Status means controller ID is present and SAPIEN returned input state.", flush=True)
    print("Press and release each Touch button. Ctrl-C stops the check.", flush=True)

    scene = sapien.Scene()
    display = sapien.render.RenderVRDisplay()
    display.root_pose = sapien.Pose()
    display.set_scene(scene)
    last_ids: tuple[int, ...] = ()
    previous: dict[int, tuple[int, float] | None] = {value: None for value in CONTROLLERS.values()}
    deadline = None if duration_s == 0.0 else time.monotonic() + duration_s
    last_report = 0.0
    try:
        while deadline is None or time.monotonic() < deadline:
            # This is the original SAPIEN path that refreshed both poses and
            # legacy controller button state before the direct OpenVR bridge.
            scene.update_render()
            display.update_render()
            display.render()
            now = time.monotonic()
            ids = tuple(int(value) for value in display.get_controller_ids())
            if ids != last_ids:
                print(f"controllers={ids} expected_left=1 expected_right=2", flush=True)
                last_ids = ids
            for side, controller_id in CONTROLLERS.items():
                if controller_id not in ids:
                    if previous[controller_id] is not None:
                        print(f"{side}: signal=MISSING controller_id={controller_id}", flush=True)
                    previous[controller_id] = None
                    continue
                try:
                    mask = int(display.get_controller_button_pressed(controller_id))
                    trigger = _trigger_value(display, controller_id, mask)
                except Exception as exc:
                    print(f"{side}: signal=ERROR button_read={type(exc).__name__}: {exc}", flush=True)
                    previous[controller_id] = None
                    continue
                state = (mask, trigger)
                old = previous[controller_id]
                if old is None or mask != old[0] or abs(trigger - old[1]) >= 0.05 or now - last_report >= 2.0:
                    print(
                        f"{side}: signal=OK mask=0x{mask:x} "
                        f"pressed={_button_names(mask, side)} trigger={trigger:.2f}",
                        flush=True,
                    )
                previous[controller_id] = state
            last_report = now
            time.sleep(1.0 / poll_hz)
    except KeyboardInterrupt:
        print("Stopping read-only SAPIEN/SteamVR button check.", flush=True)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Read-only SAPIEN/SteamVR Touch button check")
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--poll-hz", type=float, default=60.0)
    parser.add_argument("--alvr-session", default="~/.config/alvr/session.json")
    parser.add_argument("--no-alvr-wait", action="store_true")
    args = parser.parse_args(argv)
    if args.duration_s < 0.0:
        parser.error("--duration-s must be non-negative")
    if args.poll_hz <= 0.0:
        parser.error("--poll-hz must be positive")
    run(args.duration_s, args.poll_hz, args.alvr_session, args.no_alvr_wait)


if __name__ == "__main__":
    main()
