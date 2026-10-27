"""Map the physical left Touch trigger to Franka Hand opening."""

import math


def trigger_to_open_fraction(trigger: float) -> float:
    """Released trigger opens fully; fully pressed trigger closes fully."""
    value = float(trigger)
    if not math.isfinite(value):
        raise ValueError("trigger must be finite")
    return 1.0 - min(1.0, max(0.0, value))
