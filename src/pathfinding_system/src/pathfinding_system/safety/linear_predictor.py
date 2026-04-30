from __future__ import annotations
import math


class LinearPredictor:
    def __init__(self, safety_radius: float, time_step: float) -> None:
        self._safety_radius = safety_radius
        self._time_step = time_step

    def will_collide(self, s1, s2, horizon: float) -> bool:
        t = 0.0
        while t <= horizon + 1e-9:
            x1, y1 = self._extrapolate(s1, t)
            x2, y2 = self._extrapolate(s2, t)
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if dist < self._safety_radius:
                return True
            t += self._time_step
        return False

    def _extrapolate(self, state, dt: float) -> tuple[float, float]:
        x = state.pose.x
        y = state.pose.y
        theta = state.pose.theta
        v = state.velocity.linear.x
        omega = state.velocity.angular.z
        if abs(omega) < 1e-6:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
        else:
            x += (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
            y -= (v / omega) * (math.cos(theta + omega * dt) - math.cos(theta))
        return x, y
