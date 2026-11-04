# Tools

Host-side tools for running and tuning the balancer over USB. Why the paths are tuned by hand, and the
tuning workflow: [tuning.md](../../tuning.md).

| File | What it does |
|---|---|
| `console.py` | Holds the Mega's serial port open, prints the log, sends what you type. Also logs to `console.log` and sends any lines appended to `console.cmd`, so the other tools (and Claude) can drive it while it runs. |
| `viz.py` | Live top-down view: ball, target, the path the target follows (black), the goal path (orange), the ball's trail. Buttons pick the pattern; **Reset trail** clears the trail. |
| `shape.py` | `export <pattern>` writes the pattern's paths to `paths/<pattern>_paths.csv`; `load <pattern> FILE.csv` **parks the arms first**, then loads an edited path as the target's shape (read back and checked); `home` starts again. |
| `paths/` | The tuned circle and figure-8 paths, and the figure-8's goal shape. |
| `animate.py`, `wiring_anim.py`, `wiring_list.py` | Render the assembly animations and the wiring video. |

```
uv run python code/tools/console.py        # first: it owns the port (opening it resets the Mega: it homes and raises)
uv run python code/tools/viz.py            # then, in another terminal
uv run python code/tools/shape.py export figure-8
uv run python code/tools/shape.py load figure-8 code/tools/paths/figure8_target_test8_revert_test5.csv
```

Only one program can hold the port. Close the console (Ctrl-C) before flashing, and `park` first.

## Console commands

In `tools.console`, type a word and a value (`kp 6.5`); the console sends it as the firmware command
`:kp 6.5`. A single character is sent as a key. The firmware answers each command with `= name value`.

### Settings and saving

| Command | What it does |
|---|---|
| `get` | Print every setting on one line, then each pattern's size and period. |
| `save` | Keep all settings, and the current pattern's shape table, through a reset and through flashing (EEPROM). |
| `defaults` | Forget everything saved: the firmware's own values come back after a reset. |

### Patterns

| Command | What it does |
|---|---|
| `mode <name>` | Switch pattern: `centre`, `circle`, `figure-8`, `square`, `wander` (or 0–4). Power-on is always `circle`. |
| `ax <mm>` / `ay <mm>` | The current pattern's half-size in x / y. |
| `per <s>` | The current pattern's period (square: per corner, 60% hold, 40% move). |
| `n` (key) | Next pattern. The MODE button does the same. |

### Path shape (the tuning)

| Command | What it does |
|---|---|
| `h1c` `h1s` … `h4c` `h4s` `<mm>` | Circle only: how the radius changes round the circle, as harmonics (`k` cos / `k` sin for k = 1–4; `h3` = three lobes, one per leg). `ax`/`ay` set the average radius. |
| `off <bin> <dx> <dy>` | One point of the current pattern's shape table: 100 points per lap, mm added to the path. Usually sent by `tools.shape load`. |
| `offs` | Print the current pattern's shape table (0.1 mm units). |
| `ilcclear` | Zero the shape table. |

### Learning (optional, off by default)

| Command | What it does |
|---|---|
| `ilc 1` / `ilc 0` | Learn the shape table lap by lap from the ball's error. It works, but makes the plate loud. |
| `ilcg <0–1>` | How much of each lap's error it learns (0.3). |
| `ilclead <bins>` | How far ahead it applies a correction (3 of 100 per lap). |

### Ball control

| Command | What it does |
|---|---|
| `kp` `ki` `kd` | Ball gains: position, integral, speed. |
| `md` | Motor damping. |
| `lead <ms>` | Feed-forward lead: how early the pattern's turns are anticipated. |
| `rot <deg>` | Turns the tilt command (cancels a sideways drift). |
| `cx` / `cy <mm>` | Where the plate's real centre reads on the panel (hold the ball there, read `?`). |
| `t1` `t2` `t3 <deg>` | Leg trims (degrees up), for a level plate. |

### Running

| Command | What it does |
|---|---|
| `park` | Lower the arms slowly to their stops, motors off. **Do this before flashing.** |
| `home` | Home, raise and start balancing (Enter on its own does nothing in the console). |
| `s` (key) | Stop: all motors off at once (the plate drops). |
| `l` / `b` (keys) | Hold the plate level / balance the ball. |
| `?` (key) | Print the ball's position once. |
| `viz 1` / `viz 0` | The 50 Hz ball/target stream for `tools.viz` (it turns this on itself). |
| `e` (key) | Pin and encoder report (motors off only). |
| `h` (key) | Heartbeat log on/off. |

Older single-key commands (gain steps, trims, centre nudges, tilt tests, the 100 Hz debug stream)
are listed at the top of `code/firmware/balance/balance.ino`.
