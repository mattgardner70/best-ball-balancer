"""Assembly animations: one GIF per build step, rendered from the MuJoCo model.

    cd code && uv run python tools/animate.py        -> ../images/assembly/*.gif

Every part is drawn where the simulation puts it when assembled. For each step, that step's parts
fly in from an exploded position and settle into place; parts from earlier steps are already there,
later ones are hidden. The camera turns slowly while it plays.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sim import modes  # noqa: E402
from sim.sim import Sim  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "images" / "assembly"
W, H, FPS = 640, 400, 15


def smooth(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


class Scene:
    def __init__(self):
        self.sim = Sim(modes.ALL[0]())
        self.m, self.d = self.sim.model, self.sim.data
        mujoco.mj_forward(self.m, self.d)
        self.pos0 = self.m.geom_pos.copy()
        self.rgba0 = self.m.geom_rgba.copy()
        self.r = mujoco.Renderer(self.m, H, W)
        m = self.m
        self.body = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        self.mesh = lambda g: (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
                               if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else None)
        self.groups = self._group()

    def _group(self):
        """Assign every visible geom to a build step."""
        m, g = self.m, {}
        world_boxes = []
        for i in range(m.ngeom):
            if self.rgba0[i][3] == 0 or i == 0:
                continue
            b, mesh = self.body(i), self.mesh(i)
            if mesh in ("base_plate",):
                g[i] = "base"
            elif mesh == "ga37":
                g[i] = "motors"
            elif mesh in ("strap_gearbox", "strap_motor"):
                g[i] = "straps"
            elif b.startswith("arm") or b.startswith("rod"):
                g[i] = "legs" + b[-1]
            elif mesh in ("mega_mount", "driver_mount", "switch_mount"):
                g[i] = mesh
            elif b == "plate":
                g[i] = {"plate_rim": "rim", "plate_insert": "insert", "ball_stud": "studs"}.get(mesh, "glass")
            elif mesh == "frame":
                g[i] = "frame"
            elif mesh in ("button", "button_ring"):
                g[i] = "buttons"
            elif mesh == "lattice_shell":
                g[i] = "shell"
            elif mesh == "liner":
                g[i] = "liner"
            elif b == "ball":
                g[i] = "ball"
            elif m.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE:
                g[i] = "spare_ball"                 # the spare ball parked in the console dish
            elif m.geom_type[i] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                g[i] = "jack"                       # the power jack
            else:
                world_boxes.append(i)
        # Electronics stand-ins (boxes in the world body): sort them to the holder they sit on.
        x = lambda i: self.d.geom_xpos[i]
        for i in world_boxes:
            px, py = x(i)[0], x(i)[1]
            if px > 0.08 and py > -0.045:
                g[i] = "mega_mount"
            elif px > 0.05:
                g[i] = "driver_mount"
            elif px < -0.07 and py > 0.0:
                g[i] = "switch_mount"
            else:
                g[i] = "buck"
        return g

    def ids(self, *names):
        return [i for i, n in self.groups.items() if n in names]

    def offset(self, i, dworld):
        """Move geom i by a world-frame displacement (converted into its body's frame)."""
        R = self.d.xmat[self.m.geom_bodyid[i]].reshape(3, 3)
        self.m.geom_pos[i] = self.pos0[i] + R.T @ np.asarray(dworld)

    def render(self, shown, moving, cam):
        for i in range(self.m.ngeom):
            self.m.geom_pos[i] = self.pos0[i]
            self.m.geom_rgba[i] = self.rgba0[i]
            if i in self.groups and self.groups[i] not in shown:
                self.m.geom_rgba[i][3] = 0
                self.m.geom_pos[i] = self.pos0[i] + np.array([0, 0, -5.0])   # hidden parts still catch shadows
        for i in range(self.m.ngeom):          # invisible physics geoms still catch shadows: park them out of shot
            if self.rgba0[i][3] == 0:
                self.m.geom_pos[i] = self.pos0[i] + np.array([0, 0, -5.0])
        for i, d in moving.items():
            if self.m.geom_rgba[i][3] > 0:
                self.offset(i, d)
        mujoco.mj_kinematics(self.m, self.d)
        opt = mujoco.MjvOption()
        for k in range(6):
            opt.geomgroup[k] = 1
            opt.sitegroup[k] = 0
        self.r.update_scene(self.d, cam, opt)
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False   # a raised part's shadow read as a ghost part
        return Image.fromarray(self.r.render())


def camera(lookat, dist, el, az):
    c = mujoco.MjvCamera()
    c.lookat[:] = lookat
    c.distance, c.elevation, c.azimuth = dist, el, az
    return c


def caption(img, text):
    from PIL import ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 22)
    except OSError:
        font = ImageFont.load_default()
    d = ImageDraw.Draw(img)
    w = d.textlength(text, font=font)
    d.rounded_rectangle((W / 2 - w / 2 - 12, H - 46, W / 2 + w / 2 + 12, H - 12), 8, fill=(20, 20, 24))
    d.text((W / 2 - w / 2, H - 41), text, font=font, fill=(240, 240, 240))
    return img


def clip(sc, name, before, steps, cam, seconds_hold=1.2):
    """steps: list of (groups, displacement fn(geom id) -> world offset at s=0, start, length[, caption]) in seconds."""
    steps = [st if len(st) == 5 else (*st, None) for st in steps]
    end = max(t0 + dt for _, _, t0, dt, _ in steps)
    n = int((end + seconds_hold) * FPS)
    frames = []
    for k in range(n):
        t = k / FPS
        shown, moving, text = set(before), {}, None
        for groups, disp, t0, dt, cap in steps:
            s = smooth((t - t0) / dt)
            if t >= t0:
                shown |= set(groups)
                text = cap or text
            for i in sc.ids(*groups):          # offsets add up: a later step can move parts already placed
                moving[i] = moving.get(i, 0) + (1 - s) * np.asarray(disp(i))
        c = camera(cam[0], cam[1], cam[2], cam[3] + 12 * k / n)
        img = sc.render(shown, moving, c)
        if text:
            img = caption(img, text)
        frames.append(img.convert("P", palette=Image.ADAPTIVE, colors=96))
    OUT.mkdir(parents=True, exist_ok=True)
    frames[0].save(OUT / f"{name}.gif", save_all=True, append_images=frames[1:], duration=1000 // FPS, loop=0, optimize=True)
    print(name, len(frames), "frames")


LEG_T = {k: np.array([-np.sin(np.radians(a)), np.cos(np.radians(a)), 0.0]) for k, a in ((1, 0), (2, 120), (3, 240))}


def leg_part(sc, mesh_name):
    """Group name for one part type on all three legs (hub, arm, bushing, ...)."""
    return [f"{mesh_name}{k}" for k in (1, 2, 3)]


def main(only=None):
    sc = Scene()
    # Split the legs into their parts, so each can go on in its own step.
    for i, n in list(sc.groups.items()):
        if n.startswith("legs"):
            sc.groups[i] = f"{sc.mesh(i)}{n[-1]}"
    up = lambda h: (lambda i: (0, 0, h))

    def along_t(dist, ref="arm"):
        """Slide along the leg's shaft axis, away from the part it fits onto."""
        def f(i):
            k = int(sc.groups[i][-1])
            t = LEG_T[k]
            ref_ids = sc.ids(f"{ref}{k}")
            p = sc.d.geom_xpos[i] - (sc.d.geom_xpos[ref_ids[0]] if ref_ids else 0)
            sgn = 1.0 if (np.dot(p, t) >= 0 or not ref_ids or ref_ids[0] == i) else -1.0
            return sgn * dist * t
        return f

    a1 = sc.d.geom_xpos[sc.ids("arm1")[0]]
    leg_cam = ((a1[0] - 0.01, a1[1] - 0.005, a1[2] + 0.005), 0.17, -16, 55)   # close on leg 1
    base = ["base", "motors", "straps"]
    L = lambda n: leg_part(sc, n)
    legs = sum((L(n) for n in ("hub", "arm", "rod", "bushing", "shoulder_bolt", "stud_m3", "ball_socket")), [])
    boards = ["mega_mount", "driver_mount", "switch_mount", "buck"]
    plate = ["rim", "insert", "studs", "glass"]

    def outward(h, r):
        def f(i):
            p = sc.d.geom_xpos[i]
            u = np.array([p[0], p[1], 0]) / max(1e-6, np.hypot(p[0], p[1]))
            return r * u + np.array([0, 0, h])
        return f

    steps = {
        "1_base_motors": (["base"], [
            ([], up(0), 0.0, 1.2, "Base plate: 36 heat-set inserts pressed in"),
            (["motors"], up(0.12), 1.2, 1.4, "Motors 1, 2, 3 into their numbered saddles"),
            (["straps"], up(0.10), 3.0, 1.3, "Straps on, M3 x 12: tighten until the ears touch"),
        ], ((0, 0, 0.03), 0.4, -38, 120)),
        "2_legs": (base, [
            (L("hub"), along_t(0.05), 0.0, 1.2, "Hub onto the motor shaft, set screw on the flat"),
            (L("arm"), along_t(0.06), 1.6, 1.2, "Arm onto the hub, 4 x M3 x 12 through the flange"),
            (L("rod"), up(0.07), 3.2, 1.0, "Heat-press the bronze bushing into the rod"),
            (L("bushing"), along_t(0.04, "rod"), 3.8, 1.0, "Heat-press the bronze bushing into the rod"),
            (L("shoulder_bolt"), along_t(0.05, "rod"), 5.2, 1.2, "Shoulder bolt through the bushing into the arm"),
            (L("stud_m3"), up(0.05), 6.8, 1.0, "Stud into the rod top, jam nut down"),
            (L("ball_socket"), up(0.06), 8.0, 1.1, "Rod end screws onto the stud"),
        ], leg_cam),
        "3_electronics": (base + legs, [
            (["mega_mount"], up(0.12), 0.0, 1.3, "Mega on its holder, holder down with 6 x M3 x 8"),
            (["driver_mount"], up(0.12), 1.8, 1.3, "Heatsinks on, both MDD3As on the driver holder, 4 x M3 x 8"),
            (["switch_mount"], up(0.12), 3.6, 1.3, "Pololu taped and zip-tied to its plate, 2 x M3 x 8"),
            (["buck"], up(0.12), 5.4, 1.3, "Buck converter into its tray, zip tie over it"),
        ], ((0.01, 0.0, 0.03), 0.4, -45, 160)),
        "4_enclosure_plate": ([], [
            (["frame"], up(0.0), 0.0, 0.6, "The frame"),
            (["buttons"], lambda i: (0, -0.06, 0), 0.6, 1.2, "Buttons in through the console face, nuts on behind"),
            (["jack"], lambda i: (0, 0.06, 0), 2.0, 1.2, "Power jack into the back tab from outside"),
            (base + legs + boards, up(0.16), 3.4, 1.6, "Wired base drops into the frame"),
            (["liner", "shell"], up(0.16), 5.4, 1.6, "Liner (optional) and shell over the top, rods up through the slots"),
            (["rim", "studs"], up(0.18), 7.4, 1.1, "Plate: the rim"),
            (["insert"], up(0.30), 8.6, 1.2, "Insert into the rim, a few drops of super glue"),
            (["glass"], up(0.22), 10.0, 1.2, "Panel tail down through the slot and under the clips, glass in"),
            (plate, up(0.07), 11.6, 1.4, "Plate onto the three ball joints"),
            (["ball", "spare_ball"], up(0.10), 13.4, 0.9, "Ball on (and a spare in the dish). Power on: home, raise, balance"),
        ], ((0, -0.02, 0.07), 0.62, -26, 115)),
    }
    for name, (before, parts, cam) in steps.items():
        if only and name not in only:
            continue
        clip(sc, name, before, parts, cam)


if __name__ == "__main__":
    main(sys.argv[1:] or None)
