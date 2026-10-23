"""Offline sampled joint reference; no controller, plant or feedback simulation.

Velocity is the interval's position difference / dt. Acceleration is the
change of that velocity / dt, matching A-line check_candidate. These sampled
bounds do not establish continuous 1 kHz controller compatibility.
"""
import numpy as np


class BoundedJointReference:
    def __init__(self, limits, position):
        self.limits = limits
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = np.zeros(7)

    @property
    def stopped(self):
        return bool(np.all(self.velocity == 0))

    def advance(self, desired, dt, *, stop=False):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Reference requires a positive finite dt')
        desired = np.asarray(desired, dtype=float)
        if desired.shape != (7,) or not np.isfinite(desired).all():
            raise ValueError('Reference requires seven finite desired joints')
        vmax = np.minimum(self.limits.velocity, self.limits.max_step_rad / dt)
        acceleration = np.asarray(self.limits.acceleration)
        # Feedback is deliberately the last bounded reference, never an
        # unbounded virtual IK state. A 6/s position servo absorbs ordinary
        # target jitter and leaves the target mapping intact when it lags.
        requested_velocity = np.zeros(7) if stop else np.clip(
            6.0 * (desired - self.position), -vmax, vmax)
        # Start braking before the operational joint boundary. Reserve a full
        # maximum input interval as well as continuous braking distance; a
        # longer gap belongs to fault handling, not motion catch-up.
        reserve = dt + self.limits.input_gap_s
        distance_lower = np.maximum(0, self.position - self.limits.lower)
        distance_upper = np.maximum(0, np.asarray(self.limits.upper) - self.position)
        def safe_speed(distance):
            return acceleration * (np.sqrt(reserve**2 + 2*distance/acceleration) - reserve)
        requested_velocity = np.clip(requested_velocity, -safe_speed(distance_lower), safe_speed(distance_upper))
        velocity = np.clip(requested_velocity,
                           self.velocity - acceleration * dt,
                           self.velocity + acceleration * dt)
        position = self.position + velocity * dt
        if (np.any(np.abs(velocity) > vmax + 1e-9)
                or np.any(position < self.limits.lower)
                or np.any(position > self.limits.upper)):
            raise ValueError('No bounded reference step inside joint/step envelope')
        limited = not np.allclose(velocity, requested_velocity, atol=1e-14, rtol=0)
        self.position, self.velocity = position, velocity
        return position.copy(), dict(reference_rate_limited=limited,
            reference_velocity_rad_s=velocity.tolist(),
            reference_phase=('STOPPED' if self.stopped else 'STOPPING') if stop else 'FOLLOWING')
