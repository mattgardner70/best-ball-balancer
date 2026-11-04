"""Export a pattern's paths to CSV, or load an edited CSV back as the pattern's target shape.

Runs alongside tools/console.py (it talks through tools/console.log / console.cmd, no reset).

    uv run python -m tools.shape export figure-8        -> tools/paths/figure-8_paths.csv
    uv run python -m tools.shape load figure-8 FILE.csv

CSV columns (mm about the plate centre, one row per degree of lap phase):
    phase_deg, target_path_x_mm, target_path_y_mm, goal_x_mm, goal_y_mm
Edit target_path_x/y (the path the target runs) and load it: the difference from the plain pattern
goes into the firmware's shape table (100 points a lap). Loading parks the arms first (a path changed
while the plate moves throws the ball) and leaves them parked: 'home' starts again. 'save' in the console keeps it through a reset.
"""
import csv
import pathlib
import re
import sys
import time

try:
    from .viz import LAP, goal_points, pattern_xy, send, table_xy, target_xy
    from .console import LOG
except ImportError:                     # run as a script: python code/tools/shape.py
    from viz import LAP, goal_points, pattern_xy, send, table_xy, target_xy
    from console import LOG

OUT = pathlib.Path(__file__).parent / "paths"


def settings(wait=2.0):
    """Ask the firmware for its settings and shape table; read them back from the log."""
    start = LOG.stat().st_size
    send("get")
    send("offs")
    cfg, pats, offs = None, {}, None
    t0 = time.time()
    while time.time() - t0 < wait and (cfg is None or offs is None):
        time.sleep(0.2)
        with open(LOG) as f:
            f.seek(start)
            for line in f:
                text = line.split(None, 1)[-1].strip()
                if text.startswith("cfg "):
                    cfg = dict(kv.split("=") for kv in text.split()[1:])
                elif text.startswith("pattern "):
                    m = re.match(r"pattern (\S+) ax=(\S+) ay=(\S+) per=(\S+)", text)
                    pats[m[1]] = tuple(map(float, m.groups()[1:]))
                elif text.startswith("offs "):
                    v = text.split()
                    offs = (v[1], [tuple(int(n) / 10 for n in p.split(",")) for p in v[2:]])
    if cfg is None or offs is None:
        sys.exit("no answer from the balancer: is tools.console running?")
    return cfg, pats, offs


def switch(mode):
    cfg, pats, offs = settings()
    if cfg["mode"] != mode:
        send(f"mode {mode}")
        time.sleep(0.5)
        cfg, pats, offs = settings()
    return cfg, pats, offs[1]


def export(mode):
    # No switch: changing pattern moves the plate (and drops the running pattern's unsaved table).
    # Only the running pattern's table can be read back; any other exports as its plain path.
    cfg, pats, (running, table) = settings()
    if running != mode:
        table = None
    ax, ay, per = pats[mode]
    shape = [float(cfg.get(f"h{k}{f}", 0)) for k in range(1, 5) for f in "cs"]
    lap = LAP[mode] * per
    OUT.mkdir(exist_ok=True)
    path = OUT / f"{mode}_paths.csv"
    goal = goal_points(mode, per)
    goal = [goal[round(d * (len(goal) - 1) / 360) % (len(goal) - 1)] for d in range(360)]   # one per degree
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase_deg", "target_path_x_mm", "target_path_y_mm", "goal_x_mm", "goal_y_mm"])
        for d in range(360):
            t = lap * d / 360
            x, y = target_xy(mode, t, ax, ay, per, shape, table)
            gx, gy = goal[d]
            w.writerow([d, f"{x:.2f}", f"{y:.2f}", f"{gx:.2f}", f"{gy:.2f}"])
    print(path)


def park(timeout=20.0):
    """Lower the arms to their stops and switch the motors off; wait until the firmware says so.
    A path changed while the plate is moving jumps the target and throws the ball."""
    start = LOG.stat().st_size
    send("park")
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        with open(LOG) as f:
            f.seek(start)
            text = f.read()
        if "Parked, motors off" in text or "Not homed" in text:
            return
    sys.exit("the balancer didn't confirm the park: nothing loaded")


def load(mode, file):
    park()
    print("parked (arms down, motors off)")
    cfg, pats, _ = switch(mode)
    ax, ay, per = pats[mode]
    shape = [float(cfg.get(f"h{k}{f}", 0)) for k in range(1, 5) for f in "cs"]
    rows = list(csv.DictReader(open(file)))
    key = "phase_deg" if "phase_deg" in rows[0] else "angle_deg"
    pts = [(float(r[key]) / 360, float(r["target_path_x_mm"]), float(r["target_path_y_mm"])) for r in rows]
    lap = LAP[mode] * per
    # the table = file path minus the plain pattern, resampled to 100 bins
    diff = [(ph, x - pattern_xy(mode, ph * lap, ax, ay, per, shape)[0], y - pattern_xy(mode, ph * lap, ax, ay, per, shape)[1])
            for ph, x, y in pts]
    table = [(d[1], d[2]) for d in diff]
    want = [table_xy(table, i / 100) for i in range(100)]
    todo = list(range(100))
    for attempt in range(4):              # send, read the table back, resend any bin that didn't land
        for i in todo:
            send(f"off {i} {want[i][0]:.2f} {want[i][1]:.2f}")
        time.sleep(0.04 * len(todo) + 0.5)
        got = settings()[2][1]
        todo = [i for i in range(100) if max(abs(got[i][0] - want[i][0]), abs(got[i][1] - want[i][1])) > 0.15]
        if not todo:
            break
    worst = max(max(abs(dx), abs(dy)) for dx, dy in want)
    status = "all 100 points confirmed" if not todo else f"{len(todo)} points didn't take: {todo}"
    print(f"{mode}: loaded {len(rows)} points as 100, largest offset {worst:.1f} mm (limit 30), {status}. "
          f"Still parked: place the ball, then 'home' in the console. 'save' keeps the shape.")


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("export", "load") or sys.argv[2] not in LAP:
        sys.exit(__doc__)
    export(sys.argv[2]) if sys.argv[1] == "export" else load(sys.argv[2], sys.argv[3])
