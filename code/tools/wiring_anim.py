"""Wiring animation: every wire in the build, drawn one at a time, as an MP4.

    cd code && uv run python tools/wiring_anim.py        -> ../images/assembly/wiring.mp4

The connection list comes from wiring/wiring.py (the same list as the wiring PDF). Pin positions are
placed on each board from its datasheet layout, so they're close, not exact. Wires are drawn as smooth
arcs from pin to pin in their real colours; the Wago lever connectors (not in the 3D model) are drawn
as blocks on the floor. Needs ffmpeg on the PATH.
"""

import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "code"))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "wiring"))
import wiring  # noqa: E402
from CAD import chassis as ch  # noqa: E402
from CAD import enclosure as en  # noqa: E402
from animate import Scene, camera  # noqa: E402

OUT = ROOT / "images" / "assembly" / "wiring_top.mp4"
W, H, FPS = 1280, 720, 20
DRAW_S, HOLD_S = 2.0, 6.0

COLOURS = {"12v": "#d6332c", "5v": "#e8890c", "gnd": "#24282b", "link": "#8a939a", "mot+": "#e0a800",
           "mot-": "#f2f2f2", "encA": "#3bc9db", "encB": "#be4bdb", "pwm": "#1971c2", "touch": "#0c8f73",
           "btn": "#c2255c", "ctl": "#e8590c", "plug": "#24282b"}
mm = 1e-3


def rgb(h):
    return [int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)]


# --- where each pin is (mm, world) ------------------------------------------------------------------
def mega_pin(pin):
    """Arduino Mega pins, in the board's own (hx along the long edge from the USB end, hy up from the
    power-header edge) layout, then flipped and stood on edge like the holder holds it."""
    if pin.startswith("D") and pin[1:].isdigit():
        n = int(pin[1:])
        if 2 <= n <= 7:
            hx, hy = 67.3 - (n - 2) * 2.54, 50.8
        elif 8 <= n <= 13:
            hx, hy = 50.5 - (n - 8) * 2.54, 50.8
        elif 14 <= n <= 21:
            hx, hy = 77.5 + (n - 14) * 2.54, 50.8
        else:                                            # the double row at the far end, D22..D53
            k = (n - 22) // 2
            hx, hy = 96.5 + 2.54 * ((n - 22) % 2), 48.3 - 2.54 * k
    elif pin.startswith("A"):
        hx, hy = 50.8 + int(pin[1:]) * 2.54, 2.5
    elif pin == "VIN":
        hx, hy = 45.7, 2.5
    else:                                                # GND
        hx, hy = 40.6, 2.5
    L, Hb = ch.MEGA
    if ch.MEGA_FLIP:
        hx, hy = L - hx, Hb - hy
    x0, y0 = ch.MEGA_AT
    return np.array([x0 + 10 - 6 - 1.6 - 8.0, y0 - L / 2 + hx, ch.PLATE_T + ch.MEGA_LIFT + hy])


def driver_pin(dev, pin):
    side = 1 if dev == "MD1" else -1                     # MD1 on the face toward the Mega, MD2 on the other
    x = ch.DRIVER_WALL_X + side * (1.5 + ch.DRIVER_STANDOFF + 1.6 + 7.0)
    z0 = ch.PLATE_T + ch.MEGA_LIFT
    if pin.endswith(" in"):                              # input header, bottom edge
        order = ["M1A in", "M1B in", "5V0", "GND", "M2A in", "M2B in"]
        return np.array([x, ch.DRIVER_Y + (order.index(pin) - 2.5) * 2.54 * side, z0 + 4.0])
    order = ["M1A", "M1B", "VB+", "VB−", "M2A", "M2B"]    # terminal blocks, top edge
    return np.array([x, ch.DRIVER_Y + (order.index(pin) - 2.5) * 5.0 * side, z0 + ch.MDD3A[0] - 5.0])


def motor_pin(dev, pin):
    k = int(dev[1]) - 1
    a = math.radians((0, 120, 240)[k])
    u, t = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    p = 30.0 * u + (-91.0 - 4.0) * t + (int(pin[0]) - 3.5) * 2.5 * u
    return np.array([p[0], p[1], ch.BODY_Z + 6.0])


def switch_pin(pin):
    x0, y0 = ch.POWER_SWITCH_AT
    x = x0 + 1.6 + 6.0
    big = {"VIN": -7.5, "GND": 0.0, "VOUT": 7.5}
    if pin in big:
        return np.array([x, y0 + big[pin], ch.PLATE_T + 4 + 22.0])
    return np.array([x, y0 + {"A": -5.0, "OFF": 5.0}[pin], ch.PLATE_T + 4 + 4.0])


def buck_pin(pin):
    bx, by = ch.BUCK_AT
    end = -1 if pin.startswith("IN") else 1
    return np.array([bx + (3 if pin.endswith("+") else -3), by + end * 30.0, ch.PLATE_T + 9.0])


def button_pin(dev, pin):
    face_y = -(en.SHELL_D / 2 + en.FIT + en.CONSOLE_DEPTH + en.CRADLE_WALL)
    x = (-en.BUTTON_PITCH / 2 if dev == "BTNP" else en.BUTTON_PITCH / 2)
    order = ["LED+", "LED−", "C", "NO", "NC"]
    return np.array([x + (order.index(pin) - 2) * 3.0, face_y + 28.0, en.BUTTON_Z - 4.0])


def jack_pin(pin):
    jx = en.POWER_TAB[0]
    return np.array([jx + (2.0 if pin == "+" else -2.0), en.SHELL_D / 2 - 16.0, en.POWER_TAB[3]])


WAGOS = ["P12a", "P12b", "P5a", "G1", "G2", "G3", "G4"]
WAGO_AT = {name: (-62.0 + 14.0 * k, 34.0) for k, name in enumerate(WAGOS)}    # a row on the open floor
WAGO_SIZE = (12.0, 19.0, 16.0)                                               # 221-415, ports along y
WAGO_COLOUR = {"P12a": (0.32, 0.01, 0.01, 1.0), "P12b": (0.32, 0.01, 0.01, 1.0),      # +12 V: dark red
               "P5a": (0.85, 0.32, 0.0, 1.0),                                          # +5 V: orange
               **{g: (0.06, 0.06, 0.07, 1.0) for g in ("G1", "G2", "G3", "G4")}}      # ground: near black


def wago_pin(dev, port):
    x, y = WAGO_AT[dev]
    return np.array([x + 4.0, y + (int(port) - 3) * 3.6, ch.PLATE_T + WAGO_SIZE[2] - 3.0])


def pin_xyz(dev, pin, plate_z):
    if dev == "MEGA":
        p = mega_pin(pin)
    elif dev in ("MD1", "MD2"):
        p = driver_pin(dev, pin)
    elif dev in ("M1", "M2", "M3"):
        p = motor_pin(dev, pin)
    elif dev == "SW":
        p = switch_pin(pin)
    elif dev == "BUCK":
        p = buck_pin(pin)
    elif dev in ("BTNP", "BTNM"):
        p = button_pin(dev, pin)
    elif dev == "JACK":
        p = jack_pin(pin)
    elif dev in WAGO_AT:
        p = wago_pin(dev, pin)
    elif dev == "FPC":                                   # the panel tail's connector, tucked under the plate
        p = np.array([-60.0, (int(pin[0]) - 2.5) * 2.54, plate_z - 8.0])
    else:
        return None
    return p * mm


def arc(p0, p1, n=40):
    """A smooth wire: up and out of each pin, sagging a little between."""
    lift = 0.012 + 0.25 * np.linalg.norm(p1[:2] - p0[:2]) * 0.3
    top = max(p0[2], p1[2]) + lift
    c0 = p0 + np.array([0, 0, top - p0[2]])
    c1 = p1 + np.array([0, 0, top - p1[2]])
    s = np.linspace(0, 1, n)[:, None]
    return (1 - s) ** 3 * p0 + 3 * (1 - s) ** 2 * s * c0 + 3 * (1 - s) * s ** 2 * c1 + s ** 3 * p1


def add_capsule(scn, a, b, width, colour):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.array([*colour, 1.0], dtype=np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
    scn.ngeom += 1


def add_box(scn, centre, half, colour):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.asarray(half), np.asarray(centre), np.eye(3).flatten(),
                        np.array(colour, dtype=np.float32))
    scn.ngeom += 1


def caption(img, top, bottom):
    try:
        big = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 30)
        small = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 22)
    except OSError:
        big = small = ImageFont.load_default()
    d = ImageDraw.Draw(img)
    for text, font, y in ((top, big, H - 92), (bottom, small, H - 48)):
        if not text:
            continue
        w = d.textlength(text, font=font)
        d.rounded_rectangle((W / 2 - w / 2 - 14, y - 6, W / 2 + w / 2 + 14, y + font.size + 8), 8, fill=(20, 20, 24))
        d.text((W / 2 - w / 2, y), text, font=font, fill=(240, 240, 240))
    return img


def main():
    sc = Scene()
    m, d = sc.m, sc.d
    for i, n in list(sc.groups.items()):
        if n.startswith("legs"):
            sc.groups[i] = f"{sc.mesh(i)}{n[-1]}"
    plate_z = d.xpos[m.body("plate").id][2] / mm
    base_shown = {n for n in sc.groups.values() if n not in ("shell", "liner", "ball", "spare_ball", "rim", "insert", "glass", "studs")}
    see_through = {"frame": 0.12, "rim": 0.15, "insert": 0.15, "glass": 0.1, "motors": 0.2, "straps": 0.2,
                   "arm1": 0.2, "arm2": 0.2, "arm3": 0.2}
    r = mujoco.Renderer(m, H, W)
    r.scene.maxgeom = max(r.scene.maxgeom, 20000) if hasattr(r.scene, "maxgeom") else r.scene.maxgeom

    wires = []
    for group, da, pa, db, pb, cls in wiring.CONNECTIONS:
        a, b = pin_xyz(da, pa, plate_z), pin_xyz(db, pb, plate_z)
        if a is None or b is None:
            continue
        label = f"{wiring.NAMES.get(da, da)} {pa}  to  {wiring.NAMES.get(db, db)} {pb}".replace("→", "-")
        wires.append((arc(a, b), rgb(COLOURS.get(cls, "#888888")), label, wiring.WIRES[cls][1], "FPC" in (da, db)))

    tmp = Path(tempfile.mkdtemp())
    cam = camera(np.array([0.0, -0.012, 0.04]), 0.31, -90.0, 90)   # orthographic: view height = 2 d tan(fovy / 2)
    cam.orthographic = 1                                # straight down, no perspective

    def frame(k, prog, title, sub, pulse=0.0):
        """Wires before k faint; wire k drawn up to `prog` (0..1), its ends marked."""
        shown = set(base_shown)
        if 0 <= k < len(wires) and wires[k][4]:
            shown |= {"rim", "insert", "glass", "studs"}
        for i in range(m.ngeom):
            m.geom_pos[i] = sc.pos0[i]
            m.geom_rgba[i] = sc.rgba0[i]
            name = sc.groups.get(i)
            if sc.rgba0[i][3] == 0 or (name is not None and name not in shown):
                m.geom_rgba[i][3] = 0
                m.geom_pos[i] = sc.pos0[i] + np.array([0, 0, -5.0])
            elif name in see_through:
                m.geom_rgba[i][3] = see_through[name]
        mujoco.mj_kinematics(m, d)
        opt = mujoco.MjvOption()
        for g in range(6):
            opt.geomgroup[g], opt.sitegroup[g] = 1, 0
        r.update_scene(d, cam, opt)
        r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        for name, (x, y) in WAGO_AT.items():
            add_box(r.scene, np.array([x, y, ch.PLATE_T + WAGO_SIZE[2] / 2]) * mm,
                    np.array(WAGO_SIZE) / 2 * mm, WAGO_COLOUR[name])
        upto = len(wires) if k == -1 else (0 if k < 0 else k)   # -1: the finished harness; -2: the title, no wires
        for j in range(upto):
            pts, col = wires[j][0], wires[j][1]
            faded = [0.7 + 0.3 * c for c in col] if k >= 0 else col
            for a, b in zip(pts[:-1], pts[1:]):
                add_capsule(r.scene, a, b, 0.0006 if k >= 0 else 0.0009, faded)
        if 0 <= k < len(wires):
            pts, col = wires[k][0], wires[k][1]
            n = max(2, int(len(pts) * prog))
            for a, b in zip(pts[:n - 1], pts[1:n]):     # dark outline first, so light wires show on the white base
                add_capsule(r.scene, a, b, 0.0034, (0.1, 0.1, 0.12))
            for a, b in zip(pts[:n - 1], pts[1:n]):
                add_capsule(r.scene, a + np.array([0, 0, 0.002]), b + np.array([0, 0, 0.002]), 0.0024, col)
            size = 0.003 + 0.0015 * pulse
            add_box(r.scene, pts[0], np.array([size] * 3), (1.0, 0.15, 0.6, 1.0))
            if prog >= 1:
                add_box(r.scene, pts[-1], np.array([size] * 3), (1.0, 0.15, 0.6, 1.0))
        return caption(Image.fromarray(r.render()), title, sub)

    n = 0

    def put(img):
        nonlocal n
        img.save(tmp / f"{n:05d}.png")
        n += 1

    for _ in range(int(3 * FPS)):
        put(frame(-2, 0, "Wiring, one connection at a time", f"{len(wires)} connections"))
    for k, w in enumerate(wires):
        title, sub = f"{k + 1}/{len(wires)}   {w[2]}", w[3]
        for f in range(int((DRAW_S + HOLD_S) * FPS)):
            t = f / FPS
            put(frame(k, min(1.0, t / DRAW_S), title, sub, pulse=0.5 + 0.5 * math.sin(t * 2 * math.pi)))
    for _ in range(int(5 * FPS)):
        put(frame(-1, 1, "All wired", "Check 12 V and 5 V with a meter before plugging in the motors"))
    total = n
    OUT.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(tmp / "%05d.png"),
                    "-c:v", "libx264", "-profile:v", "main", "-pix_fmt", "yuv420p", "-crf", "24", "-g", str(2 * FPS),
                    "-movflags", "+faststart", str(OUT)], check=True)
    shutil.rmtree(tmp)
    print(OUT, f"{OUT.stat().st_size / 1e6:.1f} MB, {len(wires)} wires, {total // FPS} s")


if __name__ == "__main__":
    main()
