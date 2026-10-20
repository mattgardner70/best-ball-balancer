"""Setpoint generators for each mode. Each is a callable: (t, ball_xy) -> setpoint."""

import numpy as np

CORNERS = np.array([[-0.062, 0.040], [0.062, 0.040], [0.062, -0.040], [-0.062, -0.040]])


class CenterHold:
    name = "Center hold"

    def __call__(self, t, ball):
        return np.zeros(2)


def min_jerk(s):
    s = np.clip(s, 0, 1)
    return s**3 * (10 - 15 * s + 6 * s**2)


class Waypoint:
    """Glide to each corner in turn; hold 3 s once the ball has arrived."""
    name = "Waypoint"

    def __init__(self, hold=3.0, arrive_radius=0.008, travel_time=1.4):
        self.hold, self.arrive_radius, self.travel = hold, arrive_radius, travel_time
        self.index, self.arrived_at = 0, None
        self.leg_start, self.leg_from = 0.0, np.zeros(2)

    def __call__(self, t, ball):
        target = CORNERS[self.index]
        sp = self.leg_from + (target - self.leg_from) * min_jerk((t - self.leg_start) / self.travel)
        if self.arrived_at is None and np.linalg.norm(ball - target) < self.arrive_radius:
            self.arrived_at = t
        if self.arrived_at is not None and t - self.arrived_at >= self.hold:
            self.leg_from, self.leg_start = target, t
            self.index = (self.index + 1) % len(CORNERS)
            self.arrived_at = None
        return sp


class Circle:
    name = "Circle"

    def __init__(self, radius=0.045, period=3.5):
        self.r, self.w = radius, 2 * np.pi / period

    def __call__(self, t, ball):
        return self.r * np.array([np.cos(self.w * t), np.sin(self.w * t)])


class FigureEight:
    name = "Figure-8"

    def __init__(self, ax=0.060, ay=0.035, period=5.5):
        self.ax, self.ay, self.w = ax, ay, 2 * np.pi / period

    def __call__(self, t, ball):
        return np.array([self.ax * np.sin(self.w * t), self.ay * np.sin(2 * self.w * t)])


class Square:
    """Trace the waypoint square continuously, easing through each corner."""
    name = "Square"

    def __init__(self, speed=0.08):
        self.pts = np.vstack([CORNERS, CORNERS[:1]])
        lengths = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        # Min-jerk per side peaks at 1.875x average speed; size durations for `speed` peak.
        self.durations = lengths / speed * 1.875
        self.period = self.durations.sum()

    def __call__(self, t, ball):
        s = t % self.period
        for i, T in enumerate(self.durations):
            if s <= T:
                return self.pts[i] + (self.pts[i + 1] - self.pts[i]) * min_jerk(s / T)
            s -= T
        return self.pts[0]


class Wander:
    """Slow smooth random drift: sum of incommensurate sines."""
    name = "Wander"

    def __call__(self, t, ball):
        x = 0.035 * np.sin(0.75 * t) + 0.02 * np.sin(1.9 * t + 1.0)
        y = 0.022 * np.sin(1.1 * t + 0.5) + 0.012 * np.sin(2.6 * t + 2.0)
        return np.array([x, y])


ALL = [CenterHold, Waypoint, Circle, FigureEight, Square, Wander]
