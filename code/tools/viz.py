"""Top-down live view: the ball, the target it's chasing, and the pattern's path.

Runs alongside tools/console.py (which owns the port): it reads tools/console.log and sends commands
through tools/console.cmd, so opening it doesn't reset the Mega. It turns the firmware's 50 Hz viz
stream on (":viz 1") and off again on close, and re-reads the settings whenever one changes.
The buttons on the left switch the pattern; Reset clears the ball's trail.

    uv run python -m tools.console          (in one terminal, or already running)
    uv run python -m tools.viz

Plate frame, mm, as the firmware sees it: +x toward motor 1. The plate's centre is (cx, cy).
"""
import math
import re
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons

try:
    from .console import CMD, LOG
except ImportError:                     # run as a script: python code/tools/viz.py
    from console import CMD, LOG

PANEL = (187.0, 141.0)                  # panel's long side (plate x) and short side (plate y), mm
JOINT_R, LEGS = 65.0, (0, 120, 240)
MODES = ["centre", "circle", "figure-8", "square", "wander"]
GOAL = {"circle": (45, 45), "figure-8": (60, 35), "square": (45, 45), "wander": (35, 22)}   # the path the ball
                                        # should trace: each pattern's original size (the target runs a reshaped one)
LAP = {"circle": 1, "figure-8": 1, "square": 4}   # periods per lap (the shape table spans one lap)
GOAL_FILES = {"figure-8": "figure8_ideal_goal_path.csv"}   # a drawn goal (goal_x/y_mm per phase degree) replaces
                                        # the plain pattern as the goal: tools/paths/<file>


def goal_points(name, per):
    """The goal path (mm about the centre): the pattern's file in tools/paths if it has one, else the plain
    pattern at its original size."""
    file = GOAL_FILES.get(name)
    if file:
        import csv
        import pathlib
        rows = list(csv.DictReader(open(pathlib.Path(__file__).parent / "paths" / file)))
        pts = [(float(r["goal_x_mm"]), float(r["goal_y_mm"])) for r in rows]
        return pts + pts[:1]
    span = 60.0 if name == "wander" else LAP[name] * per
    return [pattern_xy(name, span * i / 400, *GOAL[name], per) for i in range(401)]
TRAIL_LAP = 0.9                         # (the target trail shows one lap; the ball trail keeps everything until Reset)


def send(line):
    with open(CMD, "a") as f:
        f.write(line + "\n")


def table_xy(table, phase):
    """The firmware's shape table (100 (dx, dy) mm per lap) at a lap phase 0-1, interpolated."""
    u = (phase % 1) * len(table)
    i = int(u) % len(table)
    j, f = (i + 1) % len(table), u - int(u)
    return (table[i][0] + (table[j][0] - table[i][0]) * f, table[i][1] + (table[j][1] - table[i][1]) * f)


def target_xy(name, t, ax, ay, per, shape=(0,) * 8, table=None):
    """The target the firmware runs: the pattern plus its shape table (mm, about the centre)."""
    x, y = pattern_xy(name, t, ax, ay, per, shape)
    if table and name in LAP:
        dx, dy = table_xy(table, t / (LAP[name] * max(per, 0.5)))
        x, y = x + dx, y + dy
    return x, y


def pattern_xy(name, t, ax, ay, per, shape=(0,) * 8):
    """firmware patternAt(), without the 2 s glide-in (mm, about the centre)."""
    per = max(per, 0.5)
    w = 2 * math.pi / per
    if name == "circle":
        th = w * t
        dr = sum(shape[2 * k - 2] * math.cos(k * th) + shape[2 * k - 1] * math.sin(k * th) for k in range(1, 5))
        return (ax + dr) * math.cos(th), (ay + dr) * math.sin(th)
    if name == "figure-8":
        return ax * math.sin(w * t), ay * math.sin(2 * w * t)
    if name == "square":
        corners = [(-1, 1), (1, 1), (1, -1), (-1, -1), (-1, 1)]
        u = t % (4 * per)
        i = int(u // per)
        s = min(max((u - i * per - 0.6 * per) / (0.4 * per), 0), 1)
        k = s ** 3 * (10 - 15 * s + 6 * s * s)
        (x0, y0), (x1, y1) = corners[i], corners[i + 1]
        return ax * (x0 + (x1 - x0) * k), ay * (y0 + (y1 - y0) * k)
    if name == "wander":
        u = t * 8.4 / per
        return (ax * (math.sin(0.75 * u) + 0.571 * math.sin(1.9 * u + 1.0)),
                ay * (math.sin(1.1 * u + 0.5) + 0.545 * math.sin(2.6 * u + 2.0)))
    return 0.0, 0.0


class Feed:
    """Follows the console log: viz samples, the current settings, and setting changes."""

    def __init__(self):
        self.f = open(LOG)
        self.f.seek(0, 2)
        self.cfg, self.pats, self.offs = {}, {}, {}
        self.ball = []                      # (t s, x, y): every sample since the last Reset (gaps = NaN)
        self.targets = deque()              # (t s, x, y): the target's own trail (the learned shape)
        self.target = (0.0, 0.0)
        self.now = 0.0                      # Mega time of the latest sample, s
        self.trail_s = 3.0
        self.changed = True

    def poll(self):
        for line in self.f.readlines():
            parts = line.split(None, 1)
            if len(parts) < 2:
                continue
            text = parts[1].strip()
            if text.startswith("V "):
                v = text.split()
                if len(v) == 7:
                    t, seen, bx, by, tx, ty = int(v[1]) / 1000, int(v[2]), *map(float, v[3:])
                    self.target, self.now = (tx, ty), t
                    self.targets.append((t, tx, ty))
                    while self.targets[-1][0] - self.targets[0][0] > self.trail_s / TRAIL_LAP:
                        self.targets.popleft()
                    if seen:
                        if self.ball and t - self.ball[-1][0] > 0.3:     # back after a gap: break the line
                            self.ball.append((t, math.nan, math.nan))
                        self.ball.append((t, bx, by))
            elif text.startswith("offs "):
                v = text.split()
                self.offs[v[1]] = [tuple(int(n) / 10 for n in p.split(",")) for p in v[2:]]
                self.changed = True
            elif text.startswith("cfg "):
                self.cfg = dict(kv.split("=") for kv in text.split()[1:])
                self.changed = True
            elif text.startswith("pattern "):
                m = re.match(r"pattern (\S+) ax=(\S+) ay=(\S+) per=(\S+)", text)
                if m:
                    self.pats[m[1]] = tuple(map(float, m.groups()[1:]))
                    self.changed = True
            elif (text.startswith("= ") and not text.startswith("= viz")) or text.startswith("Shape table"):
                send("get")                 # a setting changed: re-read them all
                send("offs")
            elif "Console connected" in text:
                send("viz 1")               # the Mega reset: turn the stream back on
                send("get")
                send("offs")


def main():
    feed = Feed()
    send("viz 1")
    send("get")
    send("offs")
    fig, ax = plt.subplots(figsize=(9, 6.5))
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm), toward motor 1")
    ax.set_ylabel("y (mm)")
    plate = plt.Rectangle((0, 0), *PANEL, fc="0.96", ec="0.6")
    ax.add_patch(plate)
    legs = [ax.plot([], [], "s", color="0.4", ms=8)[0] for _ in LEGS]
    leg_txt = [ax.text(0, 0, f"M{i + 1}", ha="center", va="center", fontsize=9, color="0.3") for i in range(3)]
    path, = ax.plot([], [], "-", color="black", lw=2, alpha=0.8, label="target path")
    goal, = ax.plot([], [], "-", color="tab:orange", lw=1.5, alpha=0.7, label="goal (ball)")
    trail, = ax.plot([], [], "-", color="tab:blue", lw=1, alpha=0.5)
    ttrail, = ax.plot([], [], "-", color="tab:red", lw=0.8, alpha=0.35, label="target (learned shape)")
    ball, = ax.plot([], [], "o", color="tab:blue", ms=14, label="ball")
    target, = ax.plot([], [], "x", color="tab:red", ms=12, mew=3, label="target")
    err = ax.plot([], [], ":", color="tab:red", lw=1)[0]
    ax.legend(loc="upper right", fontsize=8)
    title = ax.set_title("waiting for the stream (is tools.console running?)")
    fig.subplots_adjust(bottom=0.12, left=0.22)
    picker = RadioButtons(fig.add_axes([0.01, 0.55, 0.13, 0.3]), MODES, active=1)
    picker_quiet = [False]                  # set while the picker follows the firmware (no command back)

    def pick(label):
        if not picker_quiet[0]:
            send(f"mode {label}")
            feed.ball.clear()
            feed.targets.clear()
    picker.on_clicked(pick)
    reset = Button(fig.add_axes([0.42, 0.015, 0.16, 0.05]), "Reset trail")
    reset.on_clicked(lambda _: (feed.ball.clear(), feed.targets.clear()))

    def redraw_static():
        cx, cy = float(feed.cfg.get("cx", 0)), float(feed.cfg.get("cy", 0))
        plate.set_xy((cx - PANEL[0] / 2, cy - PANEL[1] / 2))
        for i, a in enumerate(LEGS):
            p = (cx + JOINT_R * math.cos(math.radians(a)), cy + JOINT_R * math.sin(math.radians(a)))
            legs[i].set_data([p[0]], [p[1]])
            leg_txt[i].set_position((p[0] + 10 * math.cos(math.radians(a)), p[1] + 10 * math.sin(math.radians(a))))
        name = feed.cfg.get("mode", "centre")
        if name in MODES and MODES.index(name) != MODES.index(picker.value_selected):
            picker_quiet[0] = True
            picker.set_active(MODES.index(name))
            picker_quiet[0] = False
        lap = {"square": 4, "circle": 1, "figure-8": 1}.get(name)
        feed.trail_s = TRAIL_LAP * lap * feed.pats[name][2] if lap and name in feed.pats else 3.0
        if name in feed.pats:
            a_, b_, per = feed.pats[name]
            span = 60.0 if name == "wander" else 4 * per if name == "square" else per
            ts = [span * i / 400 for i in range(401)]
            shape = [float(feed.cfg.get(f"h{k}{f}", 0)) for k in range(1, 5) for f in "cs"]
            pts = [target_xy(name, t, a_, b_, per, shape, feed.offs.get(name)) for t in ts]
            path.set_data([cx + p[0] for p in pts], [cy + p[1] for p in pts])
        else:
            path.set_data([cx], [cy])
        if name in GOAL and name in feed.pats:
            gp = goal_points(name, feed.pats[name][2])
            goal.set_data([cx + p[0] for p in gp], [cy + p[1] for p in gp])
        else:
            goal.set_data([], [])
        ax.set_xlim(cx - PANEL[0] / 2 - 15, cx + PANEL[0] / 2 + 15)
        ax.set_ylim(cy - PANEL[1] / 2 - 15, cy + PANEL[1] / 2 + 15)

    def update(_):
        feed.poll()
        if feed.changed:
            redraw_static()
            feed.changed = False
        tx, ty = feed.target
        target.set_data([tx], [ty])
        ttrail.set_data([p[1] for p in feed.targets], [p[2] for p in feed.targets])
        trail.set_data([b[1] for b in feed.ball], [b[2] for b in feed.ball])
        if feed.ball and feed.now - feed.ball[-1][0] < 0.3 and not math.isnan(feed.ball[-1][1]):
            _, bx, by = feed.ball[-1]
            ball.set_data([bx], [by])
            err.set_data([bx, tx], [by, ty])
            title.set_text(f"{feed.cfg.get('mode', '?')}   ball ({bx:.0f}, {by:.0f})   target ({tx:.0f}, {ty:.0f})   "
                           f"off by {math.hypot(bx - tx, by - ty):.0f} mm")
        else:
            ball.set_data([], [])
            err.set_data([], [])
            title.set_text(f"{feed.cfg.get('mode', '?')}   no ball")
        return path, goal, trail, ttrail, ball, target, err, title

    anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        send("viz 0")


if __name__ == "__main__":
    main()
