"""The base: the showpiece. A rounded box with a Voronoi lattice skin over a
solid inner wall, so you see depth through the cells but never the wiring.

Open at the bottom; a separate bottom plate carries the electronics.
Units: mm.
"""

import math

import cadquery as cq
import numpy as np
from scipy.spatial import Voronoi
from shapely.affinity import translate as _sh_translate
from shapely.geometry import Point, Polygon, box

from sim.params import GEOMETRY

# Overall form
BASE_W, BASE_D, BASE_H = 196.0, 162.0, 72.0
CORNER_R = 24.0
SKIN_T = 2.8                  # outer lattice skin
GAP = 4.0                     # air between the skins: this is the depth you see
INNER_T = 2.4                 # solid inner wall
BAND_BOTTOM, BAND_TOP = 4.0, 5.0   # thin solid rims above and below the lattice

# Lattice
CELL = 11.0                   # mean cell spacing
MERGE = 0.18                  # fraction of seeds dropped, so some cells grow large
STRUT = 2.4                   # strut width between cells
ROUND = 0.8                   # cell corner rounding: small = crisp
BEVEL = 0.0                   # soft lead-in around every cell (0 = sharp edges)
PILLAR_D = 3.0                # supports under the top lattice, at strut junctions
SEED = 7

# Front panel buttons (mode + power), side by side
BUTTON_D = 16.0               # PLACEHOLDER: panel cutout, from bought-parts research
BUTTON_PITCH = 28.0
BUTTON_Z = 36.0


def shapely_translate(geom, dx):
    return _sh_translate(geom, xoff=dx)


def _rounded_box(w, d, h, r):
    return cq.Workplane("XY").sketch().rect(w, d).vertices().fillet(r).finalize().extrude(h)


def _cells(width, height, keepout=(), seed=SEED):
    """Rounded Voronoi cells filling a width x height rectangle centred on (0, height/2)."""
    rng = np.random.default_rng(seed)
    region = box(-width / 2, 0, width / 2, height)
    for k in keepout:
        region = region.difference(k)
    # Jittered grid, spilling past the edges so border cells are well formed.
    xs = np.arange(-width / 2 - CELL, width / 2 + CELL, CELL)
    ys = np.arange(-CELL, height + CELL, CELL * 0.87)
    pts = np.array([(x + (j % 2) * CELL / 2, y) for j, y in enumerate(ys) for x in xs])
    pts += rng.normal(0, CELL * 0.22, pts.shape)
    vor = Voronoi(pts)
    cells = []
    for idx in vor.point_region:
        reg = vor.regions[idx]
        if not reg or -1 in reg:
            continue
        poly = Polygon(vor.vertices[reg]).intersection(region)
        poly = poly.buffer(-STRUT / 2).buffer(-ROUND).buffer(ROUND)
        for p in getattr(poly, "geoms", [poly]):
            if p.area > 25 and p.geom_type == "Polygon":
                cells.append(p)
    return cells


def _ring(poly, n=32):
    """n points evenly spaced around a polygon, starting at its lowest-left point."""
    ext = poly.exterior
    start = min(range(len(ext.coords) - 1), key=lambda i: (ext.coords[i][1], ext.coords[i][0]))
    d0 = ext.project(Point(ext.coords[start]))
    return [tuple(ext.interpolate((d0 + ext.length * k / n) % ext.length).coords[0]) for k in range(n)]


def _cell_cutter(cells, plane: cq.Plane, depth):
    """Each cell: a bevelled lead-in lofted from a wider outline, then straight through."""
    cutter = None
    for c in cells:
        if BEVEL <= 0:
            try:
                solid = (cq.Workplane(plane).workplane(offset=0.6).polyline(_ring(c, 40)).close()
                         .extrude(-(depth + 0.6)))
            except Exception:
                continue
            cutter = solid if cutter is None else cutter.union(solid, clean=False)
            continue
        outer = c.buffer(BEVEL)
        try:
            lead = (cq.Workplane(plane).workplane(offset=0.6).spline(_ring(outer), periodic=True).close()
                    .workplane(offset=-(0.6 + BEVEL)).spline(_ring(c), periodic=True).close().loft())
            through = (cq.Workplane(plane).workplane(offset=-BEVEL + 0.01).spline(_ring(c), periodic=True)
                       .close().extrude(-(depth - BEVEL)))
            solid = lead.union(through)
        except Exception:
            continue
        cutter = solid if cutter is None else cutter.union(solid, clean=False)
    return cutter


def _perimeter(w, d, r):
    """Walk the outer wall: u (mm along the wall, from the front-left) -> point, tangent, normal."""
    sx, sy = w - 2 * r, d - 2 * r
    arc = math.pi * r / 2
    # (length, kind, data): straights carry start point + direction; arcs carry centre + start angle.
    segs = [
        (sx, "line", ((-sx / 2, -d / 2), (1, 0))), (arc, "arc", ((sx / 2, -sy / 2), -90)),
        (sy, "line", ((w / 2, -sy / 2), (0, 1))), (arc, "arc", ((sx / 2, sy / 2), 0)),
        (sx, "line", ((sx / 2, d / 2), (-1, 0))), (arc, "arc", ((-sx / 2, sy / 2), 90)),
        (sy, "line", ((-w / 2, sy / 2), (0, -1))), (arc, "arc", ((-sx / 2, -sy / 2), 180)),
    ]
    total = sum(L for L, _, _ in segs)

    def at(u):
        u %= total
        for L, kind, data in segs:
            if u <= L:
                if kind == "line":
                    (x0, y0), (tx, ty) = data
                    return (x0 + tx * u, y0 + ty * u), (tx, ty), (ty, -tx)
                (cx, cy), a0 = data
                a = math.radians(a0) + u / r
                n = (math.cos(a), math.sin(a))
                return (cx + r * n[0], cy + r * n[1]), (-n[1], n[0]), n
            u -= L
        raise ValueError(u)

    return total, at


def _strip_cells(length, height, keepout, seed=SEED):
    """Voronoi cells on a strip that wraps around (periodic in u)."""
    rng = np.random.default_rng(seed)
    nx = int(round(length / CELL))
    pitch = length / nx
    ys = np.arange(-CELL, height + CELL, CELL * 0.87)
    base = np.array([((i + (j % 2) / 2) * pitch, y) for j, y in enumerate(ys) for i in range(nx)])
    base += rng.normal(0, CELL * 0.28, base.shape)
    base = base[rng.random(len(base)) > MERGE]
    base[:, 0] %= length
    pts = np.vstack([base, base + (length, 0), base - (length, 0)])
    vor = Voronoi(pts)
    band = box(-length, BAND_BOTTOM, 2 * length, height - BAND_TOP)
    cells = []
    for k in range(len(base)):
        reg = vor.regions[vor.point_region[k]]
        if not reg or -1 in reg:
            continue
        poly = Polygon(vor.vertices[reg]).intersection(band)
        for ko in keepout:
            for shift in (-length, 0, length):
                poly = poly.difference(shapely_translate(ko, shift))
        poly = poly.buffer(-STRUT / 2).buffer(-ROUND).buffer(ROUND)
        for p in getattr(poly, "geoms", [poly]):
            if p.geom_type == "Polygon" and not p.buffer(-1.8).is_empty:
                cells.append(p)
    return cells


def _wrapped_lattice(w, d, h):
    total, at = _perimeter(w, d, CORNER_R)
    front_mid = (w - 2 * CORNER_R) / 2
    keepout = [Point(front_mid + dx, BUTTON_Z).buffer(BUTTON_D / 2 + 5) for dx in (-BUTTON_PITCH / 2, BUTTON_PITCH / 2)]
    solids = []
    for c in _strip_cells(total, h, keepout):
        uc = c.centroid.x
        (px, py), (tx, ty), (nx, ny) = at(uc)
        plane = cq.Plane(origin=(px, py, 0), xDir=(tx, ty, 0), normal=(nx, ny, 0))
        local = shapely_translate(c, -uc)
        solid = _cell_cutter([local], plane, SKIN_T + GAP / 2)
        if solid is not None:
            solids.extend(solid.solids().vals())
    return cq.Workplane("XY").add(cq.Compound.makeCompound(solids))


def _top_cells(w, d, rim=7.0):
    """Voronoi cells on the top skin, plus pillar points where struts meet."""
    rng = np.random.default_rng(SEED + 1)
    region = box(-w / 2 + rim, -d / 2 + rim, w / 2 - rim, d / 2 - rim)
    keep = [Point(66 * math.cos(math.radians(a)), 66 * math.sin(math.radians(a))).buffer(1)
            .buffer(13 + 2.4 + 3) for a in GEOMETRY.leg_angles_deg]
    xs = np.arange(-w / 2 - CELL, w / 2 + CELL, CELL)
    ys = np.arange(-d / 2 - CELL, d / 2 + CELL, CELL * 0.87)
    pts = np.array([(x + (j % 2) * CELL / 2, y) for j, y in enumerate(ys) for x in xs])
    pts += rng.normal(0, CELL * 0.28, pts.shape)
    pts = pts[rng.random(len(pts)) > MERGE]
    vor = Voronoi(pts)
    allowed = region
    for k in keep:
        allowed = allowed.difference(k)
    cells = []
    for idx in vor.point_region:
        reg = vor.regions[idx]
        if not reg or -1 in reg:
            continue
        poly = Polygon(vor.vertices[reg]).intersection(allowed)
        poly = poly.buffer(-STRUT / 2).buffer(-ROUND).buffer(ROUND)
        for p in getattr(poly, "geoms", [poly]):
            if p.geom_type == "Polygon" and not p.buffer(-2.5).is_empty:
                cells.append(p)
    # Pillars under strut junctions, inside the printable area and clear of every cell.
    inner = box(-w / 2 + SKIN_T + GAP + 3, -d / 2 + SKIN_T + GAP + 3, w / 2 - SKIN_T - GAP - 3, d / 2 - SKIN_T - GAP - 3)
    pillars = [tuple(v) for v in vor.vertices
               if inner.contains(Point(v)) and all(c.distance(Point(v)) > PILLAR_D / 2 for c in cells)
               and not any(k.contains(Point(v)) for k in keep)]
    return cells, pillars


def base_shell(lattice=True) -> cq.Workplane:
    w, d, h = BASE_W, BASE_D, BASE_H
    outer = _rounded_box(w, d, h, CORNER_R).edges(">Z").fillet(5).edges("<Z").fillet(1.5)
    # Hollow out between the skins, and inside the inner wall.
    gap_w, gap_d = w - 2 * SKIN_T, d - 2 * SKIN_T
    z0 = BAND_BOTTOM - 1                     # the gap starts just above the bottom rim
    body = outer.cut(_rounded_box(gap_w, gap_d, h - SKIN_T - z0, CORNER_R - SKIN_T).translate((0, 0, z0)))
    # Inner shell: solid walls and deck, a GAP behind the lattice skins. Hides the electronics.
    inner_w, inner_d = gap_w - 2 * GAP, gap_d - 2 * GAP
    inner_top = h - SKIN_T - GAP
    inner = _rounded_box(inner_w, inner_d, inner_top - z0, CORNER_R - SKIN_T - GAP).translate((0, 0, z0))
    body = body.union(inner)
    # Open bottom inside the inner shell.
    body = body.cut(_rounded_box(inner_w - 2 * INNER_T, inner_d - 2 * INNER_T, inner_top - INNER_T,
                                 CORNER_R - SKIN_T - GAP - INNER_T).translate((0, 0, -0.01)))

    # Rod slots in the deck: the elbow swings between radius ~60 and ~72 mm.
    for a in GEOMETRY.leg_angles_deg:
        slot = (cq.Workplane("XY").workplane(offset=h - SKIN_T - GAP - INNER_T - 1)
                .transformed(rotate=(0, 0, a)).center(66, 0))
        # A sleeve through the gap keeps the slot walls closed, then the slot itself.
        body = body.union(slot.slot2D(22 + 2 * 2.4, 14 + 2 * 2.4, 0).extrude(GAP + INNER_T + 1.5))
        body = body.cut(slot.slot2D(22, 14, 0).extrude(SKIN_T + GAP + INNER_T + 3))

    # Buttons on the front face, through both skins.
    front = cq.Plane(origin=(0, -d / 2, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
    btn_pts = [(-BUTTON_PITCH / 2, BUTTON_Z), (BUTTON_PITCH / 2, BUTTON_Z)]
    body = body.cut(cq.Workplane(front).workplane(offset=1).pushPoints(btn_pts)
                    .circle(BUTTON_D / 2).extrude(-(SKIN_T + GAP + INNER_T + 2)))

    if lattice:
        top_cells, pillars = _top_cells(w, d)
        body = body.union(cq.Workplane("XY").workplane(offset=h - SKIN_T - GAP - 0.5)
                          .pushPoints(pillars).circle(PILLAR_D / 2).extrude(GAP + 1))
        body = body.cut(_wrapped_lattice(w, d, h))
        top = cq.Plane(origin=(0, 0, h), xDir=(1, 0, 0), normal=(0, 0, 1))
        body = body.cut(_cell_cutter(top_cells, top, SKIN_T + GAP / 2))
    return body
