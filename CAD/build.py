"""Export every printed part to stl/ and render a preview of each.

    uv run python -m CAD.build [part ...]
"""

import sys
from pathlib import Path

import cadquery as cq
import mujoco
import numpy as np

from .chassis import PARTS as CHASSIS
from .enclosure import frame, lattice_shell, liner
from . import hardware
from .parts import PARTS

PARTS = {**PARTS, "lattice_shell": lattice_shell, "frame": frame, "liner": liner, **CHASSIS}

ROOT = Path(__file__).resolve().parent.parent
STL = ROOT / "stl"
SIM_MESHES = ROOT / "code" / "sim" / "meshes"   # coarser copies for the MuJoCo viewer
PREVIEW = ROOT / "CAD" / "previews"


def render(stl: Path, png: Path, az=135, el=-35):
    """Shaded preview through MuJoCo's offscreen renderer."""
    xml = f"""<mujoco><compiler meshdir="{stl.parent}"/>
      <asset><mesh name="m" file="{stl.name}" scale="0.001 0.001 0.001"/></asset>
      <visual><global offwidth="900" offheight="700"/><headlight ambient="0.35 0.35 0.35" diffuse="0.55 0.55 0.55"/></visual>
      <worldbody><light pos="0.3 -0.3 0.6" dir="-0.4 0.4 -0.8"/>
      <geom type="mesh" mesh="m" rgba="0.78 0.82 0.9 1"/></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    lo, hi = m.mesh_vert.min(0), m.mesh_vert.max(0)
    cam.lookat[:] = (lo + hi) / 2
    cam.distance = 1.6 * np.linalg.norm(hi - lo)
    cam.azimuth, cam.elevation = az, el
    with mujoco.Renderer(m, 700, 900) as r:
        r.update_scene(d, cam)
        from PIL import Image
        Image.fromarray(r.render()).save(png)


def export_for_sim(shape, path: Path, max_faces=190_000):
    """MuJoCo caps STL meshes at 200k faces: coarsen until it fits."""
    path.parent.mkdir(exist_ok=True)
    tol = 0.2
    while True:
        cq.exporters.export(shape, str(path), tolerance=tol, angularTolerance=0.35 + tol)
        if (path.stat().st_size - 84) // 50 <= max_faces:
            return
        tol *= 1.5


PRINTER = ("Bambu Lab P1S", (256.0, 256.0, 256.0))
BED_MARGIN = 5.0  # keep clear of the plate edge and the purge area


def fits_printer(bb) -> str:
    """Does the part fit the bed in some orientation (any axis can be 'up')?"""
    name, vol = PRINTER
    dims = sorted([bb.xlen, bb.ylen, bb.zlen])
    limit = sorted(v - BED_MARGIN for v in vol)
    ok = all(d <= l for d, l in zip(dims, limit))
    return f"fits {name}" if ok else f"TOO BIG for {name} ({limit[0]:.0f} mm usable)"


def export_hardware():
    """Bought parts: viewer meshes only (they're bought, not printed)."""
    for name, fn in hardware.PARTS.items():
        export_for_sim(fn(), SIM_MESHES / f"{name}.stl")
    print(f"hardware       {len(hardware.PARTS)} bought-part models -> sim/meshes/")


def main(names):
    if not names or "hardware" in names:
        export_hardware()
        names = [n for n in names if n != "hardware"]
        if not names and len(sys.argv) > 1:
            return
    STL.mkdir(exist_ok=True)
    PREVIEW.mkdir(exist_ok=True)
    for name in names or PARTS:
        shape = PARTS[name]()
        stl = STL / f"{name}.stl"
        cq.exporters.export(shape, str(stl), tolerance=0.05, angularTolerance=0.1)
        export_for_sim(shape, SIM_MESHES / f"{name}.stl")
        bb = shape.val().BoundingBox()
        print(f"{name:<14} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f} mm  {fits_printer(bb):<20} -> {stl.relative_to(ROOT)}")
        render(SIM_MESHES / f"{name}.stl", PREVIEW / f"{name}.png")
        render(SIM_MESHES / f"{name}.stl", PREVIEW / f"{name}_below.png", az=135, el=35)


if __name__ == "__main__":
    main(sys.argv[1:])
