"""Scoped Quest power-state control for an active pose stream."""

from __future__ import annotations

import subprocess
import time
from collections.abc import Callable


_PROX_CLOSE_ACTION = "com.oculus.vrpowermanager.prox_close"
_AUTOMATION_DISABLE_ACTION = "com.oculus.vrpowermanager.automation_disable"


class QuestKeepAwake:
    """Keep Quest awake only while fresh controller packets are arriving."""

    def __init__(
        self,
        enabled: bool = True,
        adb_command: str = "adb",
        refresh_interval_s: float = 2.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.enabled = enabled
        self.adb_command = adb_command
        self.refresh_interval_s = refresh_interval_s
        self.monotonic = monotonic
        self.active = False
        self._last_refresh_s: float | None = None

    def _broadcast(self, action: str) -> bool:
        try:
            result = subprocess.run(
                [self.adb_command, "shell", "am", "broadcast", "-a", action],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def update(self, stream_active: bool) -> bool:
        """Acquire/release Quest's off-head wake state as stream activity changes."""

        if not self.enabled:
            return False
        now = self.monotonic()
        refresh_due = (
            self._last_refresh_s is None
            or now - self._last_refresh_s >= self.refresh_interval_s
        )
        if stream_active and (not self.active or refresh_due):
            self.active = self._broadcast(_PROX_CLOSE_ACTION)
            if self.active:
                self._last_refresh_s = now
        elif not stream_active and self.active:
            self.active = not self._broadcast(_AUTOMATION_DISABLE_ACTION)
            if not self.active:
                self._last_refresh_s = None
        return self.active

    def close(self) -> None:
        if self.active:
            self._broadcast(_AUTOMATION_DISABLE_ACTION)
            self.active = False
            self._last_refresh_s = None
