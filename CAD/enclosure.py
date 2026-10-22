"""Two-part base, two colours:

- lattice_shell: black single-skin Voronoi shell over the motors, cells graded
  smaller toward the top rim and the bottom edge.
- frame: white frame the shell drops into: low rim all round, and a tall front
  console with 45 deg ends carrying the two buttons.
- shelf: the console's top as a separate lid, with a stadium-shaped ball dish.

Units: mm. Origin: centre of the shell footprint, on the desk.
"""

import math

import cadquery as cq
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Point, Polygon, box

from sim.params import GEOMETRY

from .base import _perimeter, _ring, shapely_translate

# Shell
SHELL_W, SHELL_D, SHELL_H = 225.0, 200.0, 72.0   # SHELL_H: top surface height above the desk
SHELL_R = 22.0
SHELL_T = 2.4
RIM_TOP, RIM_BOTTOM = 3.5, 3.0      # solid rims on the walls
TOP_RIM = 6.0                        # solid border on the top face
# Cells: spacing grows from SPACING_MIN at the edges to SPACING_MAX mid-wall
SPACING_MIN, SPACING_MAX = 5.5, 10.5
STRUT = 1.9
ROUND = 1.2
SEED = 11

# Cradle
CRADLE_H = 12.0                      # low rim height
CRADLE_WALL = 3.0
SEAT = 8.0                           # how far the shell drops into the cradle
FIT = 0.3                            # clearance around the shell
CONSOLE_DEPTH = 40.0 - 12.7 + 7.62   # console stands this far in front of the shell (trimmed 0.5", then +0.3" so the parked ball clears the shell)
CONSOLE_HIGH = 60.0                  # console height (shell top is SHELL_H)
BUTTON_D, BUTTON_PITCH = 16.2, 30.0  # APIELE 16 mm buttons: cutout 16.0-16.2 mm

# Touch-panel ribbon: drops from the plate's breakout through the top here
CABLE_PASS = (-74.0, 0.0)            # between the 120 and 240 deg legs
CABLE_SLOT = (24.0, 10.0)            # length along y, width along x

# Open-bottom notches: the shell and liner lift straight off over the wiring (nothing passes through them)
POWER_TAB = (-55.0, 30.0, 36.0, 22.0)  # back of the frame: x, width, height, jack centre height
JACK_HOLE = 12.2                     # DC-099 pre-wired 5.5x2.1 panel jack: ~12 mm mounting hole
CORD_NOTCH = (POWER_TAB[0], POWER_TAB[1] - 4.0, POWER_TAB[2] - 2.0)  # behind the tab: room for the jack body
BUTTON_Z = 0.55 * 60.0               # button centre height on the console face
WIRE_NOTCH = (0.0, 30.0 + 26.0, BUTTON_Z + 13.0)  # front: room behind both buttons (bodies + terminals)
SHELF_T = 9.0                        # the shelf lid (console top); thick enough under the deepest point of the dish
SHELF_SEAT = 60.0 - SHELF_T          # console walls stop here; the lid sits on them, flush at the top
SHELF_RIM = 3.0                      # flat rim left round the dish (front and ends)
BALL_CLEAR = 1.5                     # a ball parked in the dish clears the shell's front face by this much
DISH_R = 0.75 * 25.4 + 0.5           # dish radius matches the 1.5" ball

# Optional liner: solid inner shell for noise, slides up inside the lattice shell
LINER_T = 1.8
LINER_FIT = 0.4


def _rounded_box(w, d, h, r):
    return cq.Workplane("XY").sketch().rect(w, d).vertices().fillet(r).finalize().extrude(h)


def _spacing(v):
    """Cell spacing at normalized height v in [0, 1]: tight at both edges."""
    return SPACING_MIN + (SPACING_MAX - SPACING_MIN) * math.sin(math.pi * min(1, max(0, v))) ** 0.8


def _poisson(width, height, spacing_at, seed, periodic=False, tries=40000):
    """Variable-radius dart throwing. spacing_at(x, y) -> minimum distance there."""
    rng = np.random.default_rng(seed)
    pts = np.empty((0, 2))
    for _ in range(tries):
        p = rng.uniform((0, -SPACING_MIN), (width, height + SPACING_MIN))
        r = spacing_at(*p)
        if len(pts):
            d = pts - p
            if periodic:
                d[:, 0] = (d[:, 0] + width / 2) % width - width / 2
            if (np.hypot(d[:, 0], d[:, 1]) < r).any():
                continue
        pts = np.vstack([pts, p])
    return pts


def _voronoi_cells(pts, region, period=None):
    """Rounded, strut-separated Voronoi cells of pts, clipped to region."""
    seeds = np.vstack([pts, pts + (period, 0), pts - (period, 0)]) if period else pts
    vor = Voronoi(seeds)
    cells = []
    for k in range(len(pts)):
        reg = vor.regions[vor.point_region[k]]
        if not reg or -1 in reg:
            continue
        poly = Polygon(vor.vertices[reg]).intersection(region)
        poly = poly.buffer(-STRUT / 2).buffer(-ROUND).buffer(ROUND)
        for g in getattr(poly, "geoms", [poly]):
            if g.geom_type == "Polygon" and not g.buffer(-1.4).is_empty:
                cells.append(g)
    return cells


def _prism(cell, plane, depth, n=36):
    return cq.Workplane(plane).workplane(offset=0.6).polyline(_ring(cell, n)).close().extrude(-(depth + 0.6)).val()


ROD_SLOT = (48.0, 33.0)              # rod pass-through in the shell top: radial length, width (was 32 x 22, before that 22 x 14;
                                     # the real rods lean more than the model's, so give them room)


def _rod_slots():
    """Slot centres (x, y, angle) where the rods pass through the top."""
    r = GEOMETRY.plate_joint_radius * 1000
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), a) for a in GEOMETRY.leg_angles_deg]


def _cable_slot(z, depth, grow=0.0):
    """Pass-through for the touch-panel ribbon, centred on CABLE_PASS."""
    return (cq.Workplane("XY").workplane(offset=z).center(*CABLE_PASS)
            .slot2D(CABLE_SLOT[0] + grow, CABLE_SLOT[1] + grow, 90).extrude(depth))


def _bottom_notches(t, grow=0.0):
    """Open-bottom notches through the back and front walls (shell, liner)."""
    out = None
    for (x, w, top), y in ((CORD_NOTCH, SHELL_D / 2), (WIRE_NOTCH, -SHELL_D / 2)):
        n = cq.Workplane("XY").center(x, y).rect(w + grow, 4 * t + 20).extrude(top)
        out = n if out is None else out.union(n)
    return out


def lattice_shell() -> cq.Workplane:
    w, d, h, t = SHELL_W, SHELL_D, SHELL_H, SHELL_T
    seat = CRADLE_H - SEAT
    body = _rounded_box(w, d, h - seat, SHELL_R).edges(">Z").fillet(4).translate((0, 0, seat))
    body = body.cut(_rounded_box(w - 2 * t, d - 2 * t, h - seat - t, SHELL_R - t).translate((0, 0, seat - 0.01)))
    for x, y, a in _rod_slots():
        body = body.cut(cq.Workplane("XY").workplane(offset=h - t - 1).center(x, y)
                        .transformed(rotate=(0, 0, a)).slot2D(*ROD_SLOT, 0).extrude(t + 2))

    solids = []
    # Walls: one periodic strip around the whole perimeter, starting just above the cradle rim.
    z0, z1 = CRADLE_H + RIM_BOTTOM, h - RIM_TOP
    total, at = _perimeter(w, d, SHELL_R)
    pts = _poisson(total, z1 - z0, lambda u, v: _spacing(v / (z1 - z0)), SEED, periodic=True)
    for c in _voronoi_cells(pts, box(-total, 0, 2 * total, z1 - z0), period=total):
        uc = c.centroid.x
        (px, py), (tx, ty), (nx, ny) = at(uc)
        plane = cq.Plane(origin=(px, py, z0), xDir=(tx, ty, 0), normal=(nx, ny, 0))
        solids.append(_prism(shapely_translate(c, -uc), plane, t + 1.5))

    # Top: graded from the border inward, solid around each rod slot.
    tw, td = w - 2 * TOP_RIM, d - 2 * TOP_RIM
    edge = box(-tw / 2, -td / 2, tw / 2, td / 2)

    def top_spacing(x, y):
        dist = min(tw / 2 - abs(x - tw / 2), td / 2 - abs(y - td / 2))
        return _spacing(min(1, dist / 40) / 2)

    pts = _poisson(tw, td, top_spacing, SEED + 1) - (tw / 2, td / 2)
    region = edge
    for x, y, _ in _rod_slots():
        region = region.difference(Point(x, y).buffer(ROD_SLOT[0] / 2 + 5))
    top = cq.Plane(origin=(0, 0, h), xDir=(1, 0, 0), normal=(0, 0, 1))
    for c in _voronoi_cells(pts, region):
        solids.append(_prism(c, top, t + 1.5))

    body = body.cut(_bottom_notches(t))
    # Pass the cells as separate tools: neighbouring prisms can overlap at the
    # corners, and a single overlapping compound makes the boolean drop everything.
    return cq.Workplane("XY").add(body.val().cut(*solids))


def frame() -> cq.Workplane:
    """White frame the shell drops into, after the reference organizer frame:
    a low rim all round, and a tall front console whose ends slope down at 45 deg.

    The console stands CONSOLE_DEPTH proud of the shell so the buttons have room
    behind them. Open at the bottom; the shell rests on an internal ledge.
    """
    t = CRADLE_WALL
    w = SHELL_W + 2 * (FIT + t)
    back = SHELL_D / 2 + FIT + t
    front = -(SHELL_D / 2 + FIT + CONSOLE_DEPTH + t)
    d, yc = back - front, (back + front) / 2
    r = SHELL_R + FIT + t

    # Side profile: tall over the console, then 45 deg down to the rim.
    knee = -SHELL_D / 2 - FIT + 12
    foot = knee + (CONSOLE_HIGH - CRADLE_H)
    def profile(drop):
        k = knee - drop * math.tan(math.radians(22.5))  # keep the slope wall ~t thick
        return (cq.Workplane("YZ").polyline([(front - 1, -1), (front - 1, CONSOLE_HIGH - drop), (k, CONSOLE_HIGH - drop),
                                             (k + (CONSOLE_HIGH - CRADLE_H), CRADLE_H - drop),
                                             (back + 1, CRADLE_H - drop), (back + 1, -1)]).close()
                .extrude(w / 2 + 1, both=True))

    body = _rounded_box(w, d, CONSOLE_HIGH, r).translate((0, yc, 0)).intersect(profile(0))
    try:
        body = body.edges(">Z").fillet(1.5)
    except Exception:
        pass
    # Hollow it by hand (shell() fails on the sloped ends): walls and cap t thick, open bottom.
    hollow = _rounded_box(w - 2 * t, d - 2 * t, CONSOLE_HIGH, r - t).translate((0, yc, -0.01)).intersect(profile(t))
    body = body.cut(hollow)

    # Pocket for the shell (opens the cap over it), and the ledge it rests on.
    seat = CRADLE_H - SEAT
    body = body.cut(_rounded_box(SHELL_W + 2 * FIT, SHELL_D + 2 * FIT, CONSOLE_HIGH + 2, SHELL_R + FIT)
                    .translate((0, 0, seat)))
    # Down to the desk: a ledge hanging seat - 3 above the bed has nothing under it across the console.
    ledge = (_rounded_box(SHELL_W + 2 * FIT + 1, SHELL_D + 2 * FIT + 1, seat, SHELL_R + FIT)
             .cut(_rounded_box(SHELL_W - 8, SHELL_D - 8, seat, SHELL_R - 4)))
    body = body.union(ledge)
    # Wiring notches through the ledge (front and back) and a cord notch in the back rim.
    for (nx, nw, _), y in ((CORD_NOTCH, SHELL_D / 2), (WIRE_NOTCH, -SHELL_D / 2)):
        body = body.cut(cq.Workplane("XY").center(nx, y).rect(nw, 12).extrude(seat + 0.1))
    # Power tab: the back wall rises into a rounded tab carrying the panel jack.
    tx, tw, th, jz = POWER_TAB
    tab = (cq.Workplane("XZ", origin=(0, back, 0)).center(tx, th / 2).rect(tw, th).extrude(t)
           .edges("|Y and >Z").fillet(tw / 2 - 0.5))
    body = body.union(tab)
    body = body.cut(cq.Workplane("XZ", origin=(0, back + 1, 0)).center(tx, jz).circle(JACK_HOLE / 2).extrude(t + 2))

    # The shelf (console top, with the ball dish) is part of the frame: cut the console down to its
    # seat and fuse the shelf in. Print open side down with supports under the shelf.
    body = body.cut(cq.Workplane("XY").workplane(offset=SHELF_SEAT).center(0, (front - 1 + _shelf_back()) / 2)
                    .rect(w + 2, _shelf_back() - front + 1).extrude(CONSOLE_HIGH - SHELF_SEAT + 1))
    body = body.union(shelf())

    # Buttons on the console's vertical face.
    face = cq.Plane(origin=(0, front, BUTTON_Z), xDir=(1, 0, 0), normal=(0, -1, 0))
    return body.cut(cq.Workplane(face).pushPoints([(-BUTTON_PITCH / 2, 0), (BUTTON_PITCH / 2, 0)])
                    .circle(BUTTON_D / 2).extrude(-2 * t, both=True))


def liner() -> cq.Workplane:
    """Optional solid inner shell: hides the mechanism and damps motor noise.

    Slides up inside the lattice shell; trapped between the frame ledge and the
    shell's top. Print with a gyroid infill for a little extra damping.
    """
    seat = CRADLE_H - SEAT
    w = SHELL_W - 2 * (SHELL_T + LINER_FIT)
    d = SHELL_D - 2 * (SHELL_T + LINER_FIT)
    r = SHELL_R - SHELL_T - LINER_FIT
    top = SHELL_H - SHELL_T - LINER_FIT
    body = _rounded_box(w, d, top - seat, r).edges(">Z").fillet(2).translate((0, 0, seat))
    body = body.cut(_rounded_box(w - 2 * LINER_T, d - 2 * LINER_T, top - seat - LINER_T, r - LINER_T)
                    .translate((0, 0, seat - 0.01)))
    for x, y, a in _rod_slots():
        body = body.cut(cq.Workplane("XY").workplane(offset=top - LINER_T - 1).center(x, y)
                        .transformed(rotate=(0, 0, a)).slot2D(ROD_SLOT[0] + 4, ROD_SLOT[1] + 4, 0).extrude(LINER_T + 2))
    body = body.cut(_bottom_notches(LINER_T, grow=2))
    return body.cut(_cable_slot(top - LINER_T - 1, LINER_T + 2, grow=4))


def dish_geometry():
    """(centre y, half-width at the top, depth) of the shelf's ball dish."""
    t = CRADLE_WALL
    front_in = -(SHELL_D / 2 + FIT + CONSOLE_DEPTH)          # inside face of the console's front wall
    ball_r = 0.75 * 25.4
    yc = -SHELL_D / 2 - BALL_CLEAR - ball_r                   # parked ball just clears the shell
    half_w = yc - (front_in - t) - SHELF_RIM                  # out to a rim at the shelf's front edge
    depth = DISH_R - math.sqrt(DISH_R**2 - half_w**2)
    return yc, half_w, depth


def _shelf_back():
    """The shelf's back edge: where the console meets the shell's pocket."""
    return -SHELL_D / 2 - FIT


def shelf() -> cq.Workplane:
    """The console's top, fused into frame(): flush with the frame, with a lip inside the
    console walls, with a shallow stadium dish (the shape the 1.5" ball sweeps rolling along a
    line) running nearly edge to edge and following the rounded front corners.
    Print flat, dish up. World coordinates (it sits where it is drawn).
    """
    t = CRADLE_WALL
    w = SHELL_W + 2 * (FIT + t)
    front = -(SHELL_D / 2 + FIT + CONSOLE_DEPTH + t)
    back = _shelf_back()
    r = SHELL_R + FIT + t
    yc_frame = (SHELL_D / 2 + FIT + t + front) / 2
    d_frame = SHELL_D / 2 + FIT + t - front
    outline = _rounded_box(w, d_frame, SHELF_T, r).translate((0, yc_frame, SHELF_SEAT))
    lid = outline.intersect(cq.Workplane("XY").center(0, (front + back) / 2).rect(w + 2, back - front)
                            .extrude(SHELF_T).translate((0, 0, SHELF_SEAT)))
    try:
        lid = lid.faces(">Z").edges().fillet(1.2)
    except Exception:
        pass
    # Locating lip just inside the front and end walls.
    lip = (_rounded_box(w - 2 * t - 0.6, d_frame - 2 * t - 0.6, 2.0, r - t - 0.3).translate((0, yc_frame, SHELF_SEAT - 2.0))
           .cut(_rounded_box(w - 2 * t - 4.6, d_frame - 2 * t - 4.6, 2.0, r - t - 2.3).translate((0, yc_frame, SHELF_SEAT - 2.0)))
           .intersect(cq.Workplane("XY").center(0, (front + back) / 2).rect(w, back - front - 1).extrude(-3)
                      .translate((0, 0, SHELF_SEAT))))
    lid = lid.union(lip)

    # Stadium dish: radius matched to the ball, ends pulled in to follow the rounded corners,
    # sitting forward so a parked ball clears the shell.
    yc, half_w, depth = dish_geometry()
    corner = (w / 2 - r, front + r)                                    # front corner arc centre
    dy = abs(yc - corner[1])
    reach = max(0.0, (r - SHELF_RIM - half_w) ** 2 - dy**2)
    ends = corner[0] + math.sqrt(reach)                                # end-sphere centres stay inside the corners
    zc = CONSOLE_HIGH + DISH_R - depth
    sweep = (cq.Workplane("YZ").workplane(offset=-ends).center(yc, zc).circle(DISH_R).extrude(2 * ends)
             .union(cq.Workplane("XY").sphere(DISH_R).translate((-ends, yc, zc)))
             .union(cq.Workplane("XY").sphere(DISH_R).translate((ends, yc, zc))))
    return lid.cut(sweep)
