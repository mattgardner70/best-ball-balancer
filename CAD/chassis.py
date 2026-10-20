"""White base plate: the chassis. Every motor and every piece of electronics
mounts here; the frame and the lattice shell are covers that lift off.

Seen through the lattice when the liner is out, so it is kept tidy:
components on standoffs, wiring on zip-tie loops along clean routes.
(Optional graded Voronoi floor lightening: FLOOR_LATTICE.)

Units: mm. Origin: robot centre, on the desk.
"""

import math

import cadquery as cq
import numpy as np
from shapely.geometry import LineString, Point, Polygon, box

from sim.params import GEOMETRY

from .base import _ring
from .enclosure import (CABLE_PASS, SEAT, SHELL_D, SHELL_W, _poisson, _rounded_box, _voronoi_cells)
from . import hardware as hw

PLATE_T = 4.0
PLATE_W, PLATE_D = SHELL_W - 9.0, SHELL_D - 9.0   # inside the frame's ledge ring
PLATE_R = 14.0
FLOOR_LATTICE = False                # graded Voronoi lightening in the open floor (Matt: not needed)
INSERT_M3 = (4.0, 7.0)               # ruthex M3 x 5.7 heat-set insert: hole dia, depth (one insert size for the whole build)

# Motor saddles (see sim/model.py for the stack along each shaft)
MOTOR_FACE = -23.05                  # gearbox face, along the leg tangent from the rod plane (2 mm past the coupling flange)
AXIS_Z = GEOMETRY.motor_height * 1000
BODY_Z = AXIS_Z - hw.GA37_SHAFT_OFFSET
CRADLE_CLEAR = 0.3
INSERT_WALL = 2.6                    # plastic all round each heat-set insert
STRAP_T, EAR_T = 3.0, 4.0
STRAP_PRELOAD = 1.2                  # the arch is flattened this much: snug on the motor, the ears still float
                                     # 1.2 mm over the saddle, so the screws pull the strap down and it flexes tight
SCREW_HEAD_R = 3.1                   # M3 socket head + clearance
GEARBOX_SADDLE = (MOTOR_FACE - 3.0, MOTOR_FACE - hw.GA37_GEARBOX[1] + 2.0)      # t0, t1 along the tangent
MOTOR_SADDLE = (MOTOR_FACE - hw.GA37_GEARBOX[1] - 0.3, MOTOR_FACE - hw.GA37_GEARBOX[1] - 14.0)

# Electronics layout (from a free-space search between the motors and arm swings)
MEGA_AT = (88.0, 6.0)                # standing on edge, board along y, parts facing -x
MEGA_FLIP = True                     # USB end toward +y (back corner); the digital header is the bottom edge
MEGA_LIFT = 9.0                      # board's bottom edge above the plate: room for the Dupont plugs below it
                                     # (the liner's inside top, ~67.4 mm, caps it: board top 66.3)
MEGA = (101.6, 53.3)
MEGA_HOLES = [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5), (90.2, 50.7), (96.5, 2.5)]  # (I) Arduino drawing
# Two Cytron MDD3A drivers, back to back on one upright wall in the front-right corner, beside the Mega.
MDD3A = (48.26, 41.66)               # board: terminals-to-header length, width (datasheet)
MDD3A_HOLES = (43.18, 36.58)         # hole spacing, same axes; 3 mm holes -> M2.5 x 8 screws
DRIVER_WALL_X = 76.5                 # wall centre line (runs along y)
DRIVER_Y = -72.5                     # boards' centre along y
DRIVER_STANDOFF = 6.0
POWER_SWITCH_AT = (82.0, 85.0)       # Pololu #2813: upright in a slot bracket on the back edge, over the Mega's USB plug
POWER_SWITCH = (20.3, 25.4)          # 0.8" x 1.0" (Pololu); terminal blocks on a 0.8" edge
SWITCH_Z = 39.5                      # board's bottom edge: above the USB plug
from .enclosure import POWER_TAB  # noqa: E402  (the jack lives in the frame's back tab)
BUCK_AT = (-82.0, -56.0)             # 12 V -> 5 V buck: DROK B07P663XJV, sealed case with wire leads
BUCK = (51.0, 25.0)                  # over the mounting feet, body width (DROK drawing); body 34 long, 21 tall
BUCK_FOOT_W = 18.0                   # mounting feet width
BUCK_ROT = 90

# Zip-tie loops: (x, y, route direction in degrees)
TIE_LOOPS = [
    (-55.0, 64.0, 90), (-75.0, 60.0, 0),               # jack leads down from the frame tab, toward the switch
    (-30.0, 78.0, 0), (5.0, 60.0, 0), (40.0, 40.0, 60),  # back run to the Mega
    (60.0, -40.0, 90), (60.0, 20.0, 90),                # along the Mega
    (-35.0, -60.0, 0), (25.0, -40.0, 0),                # driver 1 run
    (-74.0, 20.0, 90),                                  # touch-panel ribbon drop
    (0.0, -82.0, 0),                                    # button wires from the front notch
    (-62.0, -56.0, 90),                                 # buck converter leads out to the Wagos
]


def _leg_frames():
    for a in GEOMETRY.leg_angles_deg:
        r = math.radians(a)
        yield a, np.array([math.cos(r), math.sin(r)]), np.array([-math.sin(r), math.cos(r)])


def _insert_offset(radius):
    """Distance from the motor axis to each strap screw: the screw head clears the strap's arch."""
    return radius + CRADLE_CLEAR + STRAP_T + SCREW_HEAD_R + 0.6


def _saddle_width(radius):
    return 2 * (_insert_offset(radius) + INSERT_M3[0] / 2 + INSERT_WALL)


def _saddle(u, t, t0, t1, radius, rs):
    """Cradle block under a motor section spanning t0..t1 along the tangent, with two
    M3 inserts fully enclosed in plastic for the strap screws."""
    c = rs * u + (t0 + t1) / 2 * t
    length = abs(t1 - t0)
    w = _saddle_width(radius)
    plane = cq.Plane(origin=(c[0], c[1], PLATE_T), xDir=(t[0], t[1], 0), normal=(0, 0, 1))
    block = cq.Workplane(plane).rect(length, w).extrude(BODY_Z - PLATE_T).edges("|Z").fillet(2.5)
    cradle = cq.Workplane(cq.Plane(origin=(c[0] - t[0] * length, c[1] - t[1] * length, BODY_Z),
                                   xDir=(u[0], u[1], 0), normal=(t[0], t[1], 0))).circle(radius + CRADLE_CLEAR).extrude(2 * length)
    block = block.cut(cradle)
    for sgn in (-1, 1):
        p = c + sgn * _insert_offset(radius) * u
        block = block.cut(cq.Workplane("XY").workplane(offset=BODY_Z - INSERT_M3[1]).center(*p)
                          .circle(INSERT_M3[0] / 2).extrude(INSERT_M3[1] + 1))
    return block


def motor_saddles():
    rs = GEOMETRY.motor_radius * 1000
    gd, gl = hw.GA37_GEARBOX
    md, _ = hw.GA37_MOTOR
    out = None
    for ang, u, t in _leg_frames():
        # Gearbox saddle with a lip on the gearbox face; motor saddle against the gearbox's back shoulder:
        # together they trap the motor axially.
        a = _saddle(u, t, *GEARBOX_SADDLE, gd / 2, rs)
        lip_c = rs * u + (MOTOR_FACE + 1.5) * t
        lip = (cq.Workplane(cq.Plane(origin=(lip_c[0], lip_c[1], PLATE_T), xDir=(t[0], t[1], 0), normal=(0, 0, 1)))
               .rect(3, _saddle_width(gd / 2) - 12).extrude(BODY_Z - PLATE_T - 8))
        b = _saddle(u, t, *MOTOR_SADDLE, md / 2, rs)
        part = a.union(lip).union(b)
        out = part if out is None else out.union(part)
    return out


def motor_strap(radius, length) -> cq.Workplane:
    """Flattened arched strap matching one saddle, M3 x 10 screws into the saddle inserts.

    The arch is an ellipse, as wide as the motor plus clearance but STRAP_PRELOAD lower at the crown,
    so resting on the motor its ears float STRAP_PRELOAD above the saddle; tightening the screws
    bends the arch down onto the motor. Origin on the motor axis at the saddle top; arc axis along y.
    """
    a_in, b_in = radius + CRADLE_CLEAR, radius - STRAP_PRELOAD      # half-width, crown height (ears' underside = 0)
    a_out, b_out = a_in + STRAP_T, b_in + STRAP_T
    half = _saddle_width(radius) / 2
    arc = (cq.Workplane("XZ").ellipse(a_out, b_out).extrude(length / 2, both=True)
           .cut(cq.Workplane("XZ").ellipse(a_in, b_in).extrude(length, both=True))
           .cut(cq.Workplane("XY").rect(4 * a_out, 4 * a_out).extrude(-2 * a_out)))
    ears = (cq.Workplane("XY").rect(2 * half, length).extrude(EAR_T)
            .cut(cq.Workplane("XZ").ellipse(a_in, b_in).extrude(length, both=True)))
    body = arc.union(ears).translate((0, 0, STRAP_PRELOAD))
    try:
        body = body.edges("|Y").fillet(0.8)
    except Exception:
        pass
    for sgn in (-1, 1):
        x = sgn * _insert_offset(radius)
        body = body.cut(cq.Workplane("XY").center(x, 0).circle(1.7).extrude(20))                                  # M3 clearance
        body = body.cut(cq.Workplane("XY").workplane(offset=STRAP_PRELOAD + EAR_T - 1.2).center(x, 0)
                        .circle(SCREW_HEAD_R).extrude(5))                                                           # head seat
    return body


def strap_gearbox():
    """Print 3."""
    return motor_strap(hw.GA37_GEARBOX[0] / 2, abs(GEARBOX_SADDLE[1] - GEARBOX_SADDLE[0]))


def strap_motor():
    """Print 3."""
    return motor_strap(hw.GA37_MOTOR[0] / 2, abs(MOTOR_SADDLE[1] - MOTOR_SADDLE[0]))


# Saddle centres along each leg tangent (for the straps and the viewer)
GEARBOX_SADDLE_T = sum(GEARBOX_SADDLE) / 2
MOTOR_SADDLE_T = sum(MOTOR_SADDLE) / 2


def _standoffs(points, h, od=7.0, hole=INSERT_M3):
    w = cq.Workplane("XY").workplane(offset=PLATE_T).pushPoints(points).circle(od / 2).extrude(h)
    return w.cut(cq.Workplane("XY").workplane(offset=PLATE_T + h - hole[1]).pushPoints(points)
                 .circle(hole[0] / 2).extrude(hole[1] + 0.1))


def _buck_holder():
    """Low tray for the DROK buck: sized over its mounting feet, open ends wide enough for the feet
    and leads, and two zip-tie slots through the floor-level side rails to strap the body down."""
    L, W = BUCK[0] + 1.0, BUCK[1] + 1.0                # inside, with clearance
    wall, h = 2.0, 5.0
    pl = cq.Workplane("XY").workplane(offset=PLATE_T).center(*BUCK_AT).transformed(rotate=(0, 0, BUCK_ROT))
    tray = pl.rect(L + 2 * wall, W + 2 * wall).extrude(h).cut(pl.rect(L, W).extrude(h))
    tray = tray.cut(pl.rect(L + 2 * wall + 1, BUCK_FOOT_W + 2).extrude(h))       # open ends: feet and leads
    for dx in (-8.0, 8.0):                                                        # zip-tie slots under the body
        tray = tray.cut(pl.center(dx, 0).rect(3.0, W + 2 * wall + 1).extrude(2.2))
    return tray


def _tie_loop(x, y, ang):
    """A bridge a zip tie threads under. Route runs along `ang`; the tie wraps across it."""
    post = cq.Workplane("XY").workplane(offset=PLATE_T).center(x, y).transformed(rotate=(0, 0, ang))
    arch = post.rect(5, 12).extrude(5).edges(">Z").fillet(1.5)
    return arch.cut(post.rect(6, 5).extrude(2.6))


def _mega_holes():
    """(along y from the board's -y end, up from its bottom edge) for each hole, flipped if MEGA_FLIP."""
    L, H = MEGA
    return [(L - hx, H - hy) if MEGA_FLIP else (hx, hy) for hx, hy in MEGA_HOLES]


def _mega_cradle():
    """Back wall with six standoffs; the board stands on edge MEGA_LIFT above the plate, parts facing
    the centre. The wall stops at the board's top edge (the liner's inside top is just above)."""
    x0, y0 = MEGA_AT
    wall_x = x0 + 10.0
    L, H = MEGA
    z_board = PLATE_T + MEGA_LIFT
    top = z_board + H
    wall = (cq.Workplane("XY").workplane(offset=PLATE_T).center(wall_x + 1.5, y0).rect(3, L + 8).extrude(top - PLATE_T)
            .edges("|X and >Z").fillet(2))
    # Gussets.
    for dy in (-L / 2 - 2, 0, L / 2 + 2):
        g = (cq.Workplane("XZ", origin=(0, y0 + dy, 0)).polyline([(wall_x + 3, PLATE_T), (wall_x + 3, PLATE_T + 30),
                                                                  (wall_x + 3 + 0.01, PLATE_T + 30), (wall_x + 10, PLATE_T)])
             .close().extrude(1.5, both=True))
        wall = wall.union(g)
    standoff = 6.0
    for hx, hy in _mega_holes():
        y, z = y0 - L / 2 + hx, z_board + hy
        s = (cq.Workplane("YZ", origin=(wall_x, 0, 0)).center(y, z).circle(3.0).extrude(-standoff)
             .cut(cq.Workplane("YZ", origin=(wall_x - standoff, 0, 0)).center(y, z).circle(INSERT_M3[0] / 2)
                  .extrude(INSERT_M3[1])))
        wall = wall.union(s)
    wall = wall.intersect(cq.Workplane("XY").rect(1000, 1000).extrude(top))      # standoffs near the top edge: flat at the top
    # A ledge the board's bottom edge rests on while you start the screws.
    ledge = cq.Workplane("XY").workplane(offset=PLATE_T).center(wall_x - standoff - 1.5, y0).rect(3, L - 10).extrude(MEGA_LIFT)
    return wall.union(ledge)


def _driver_wall():
    """Upright wall along y carrying both MDD3As back to back, one on each face, on M2.5 standoffs.
    Each board stands with its terminal blocks along the top edge and its input header along the
    bottom edge, MEGA_LIFT above the plate like the Mega."""
    L, W = MDD3A
    hl, hw_ = MDD3A_HOLES
    z0 = PLATE_T + MEGA_LIFT                       # boards' bottom edge
    top = z0 + L + 1.0
    wall = (cq.Workplane("XY").workplane(offset=PLATE_T).center(DRIVER_WALL_X, DRIVER_Y).rect(3, W + 6).extrude(top - PLATE_T)
            .edges("|X and >Z").fillet(1.5))
    # Feet along the wall's base for stiffness (low, under the boards' bottom edge).
    wall = wall.union(cq.Workplane("XY").workplane(offset=PLATE_T).center(DRIVER_WALL_X, DRIVER_Y).rect(9, W + 6).extrude(3))
    for side in (1, -1):
        x_face = DRIVER_WALL_X + side * 1.5
        for dy in (-hw_ / 2, hw_ / 2):
            for dz in (-hl / 2, hl / 2):
                y, z = DRIVER_Y + dy, z0 + L / 2 + dz
                post = (cq.Workplane("YZ", origin=(x_face, 0, 0)).center(y, z).circle(3.0).extrude(side * DRIVER_STANDOFF)
                        .cut(cq.Workplane("YZ", origin=(x_face, 0, 0)).center(y, z).circle(1.05).extrude(side * (DRIVER_STANDOFF + 0.1))))
                wall = wall.union(post)
    return wall


def _switch_bracket():
    """Pololu bracket on the back edge, above the Mega's USB plug: a short wall on two posts with a
    grooved rail above and below. The board slides in from the -x end (stop at +x), standing upright
    6 mm off the wall, terminal blocks on its -x end facing the centre."""
    bw, bh = POWER_SWITCH[1], POWER_SWITCH[0]      # lying on its side: 1.0" along x, 0.8" tall
    x0, y_board = POWER_SWITCH_AT
    y_wall = y_board + 6.0                          # wall's inner face
    z0 = SWITCH_Z
    xs = (x0 - bw / 2 - 4, x0 + bw / 2 + 3)
    body = (cq.Workplane("XY").workplane(offset=z0 - 3).center(sum(xs) / 2, y_wall + 1.25)
            .rect(xs[1] - xs[0], 2.5).extrude(bh + 7))
    for x in xs:                                    # posts down to the plate
        body = body.union(cq.Workplane("XY").workplane(offset=PLATE_T).center(x, y_wall + 1.25).rect(3, 2.5).extrude(z0 + bh + 4 - PLATE_T))
    for z, sgn in ((z0, -1), (z0 + bh, 1)):         # rails with a groove the board's long edges ride in
        rail = (cq.Workplane("XY").workplane(offset=z - 1.5 if sgn < 0 else z - 1.0).center(sum(xs) / 2, (y_wall + y_board - 1.5) / 2)
                .rect(xs[1] - xs[0], y_wall - y_board + 1.5).extrude(2.5))
        groove = (cq.Workplane("XY").workplane(offset=z - 0.6 if sgn < 0 else z - 1.0).center(sum(xs) / 2 - 1.5, y_board)
                  .rect(xs[1] - xs[0], 1.9).extrude(1.6))
        body = body.union(rail.cut(groove))
    body = body.union(cq.Workplane("XY").workplane(offset=z0 - 1.5).center(x0 + bw / 2 + 1.2, y_board).rect(2.4, 4).extrude(bh + 3))   # end stop
    return body


def _keepouts():
    """Areas the lightening lattice must avoid."""
    rs = GEOMETRY.motor_radius * 1000
    keep = []
    for _, u, t in _leg_frames():
        keep.append(LineString([rs * u + (MOTOR_FACE + 3) * t, rs * u + (MOTOR_FACE - 40) * t])
                    .buffer(_saddle_width(hw.GA37_GEARBOX[0] / 2) / 2 + 4))
    x0, y0 = MEGA_AT
    keep.append(box(x0 - 14, y0 - MEGA[0] / 2 - 8, x0 + 16, y0 + MEGA[0] / 2 + 8))
    keep.append(box(DRIVER_WALL_X - 22, DRIVER_Y - 27, DRIVER_WALL_X + 22, DRIVER_Y + 27))
    keep.append(Point(POWER_SWITCH_AT).buffer(20))
    keep.append(box(BUCK_AT[0] - 16, BUCK_AT[1] - 30, BUCK_AT[0] + 16, BUCK_AT[1] + 30))
    keep += [Point(x, y).buffer(10) for x, y, _ in TIE_LOOPS]
    keep.append(LineString([(POWER_TAB[0], PLATE_D / 2), (POWER_TAB[0], 60)]).buffer(10))
    return keep


def base_plate() -> cq.Workplane:
    w, d = PLATE_W, PLATE_D
    plate = _rounded_box(w, d, PLATE_T, PLATE_R).edges(">Z").fillet(1.2)

    if FLOOR_LATTICE:
        plate = _lighten(plate, w, d)
    return _mount_everything(plate)


def _lighten(plate, w, d):
    """Graded Voronoi lightening in the open floor."""
    region = box(-w / 2 + 8, -d / 2 + 8, w / 2 - 8, d / 2 - 8)
    for k in _keepouts():
        region = region.difference(k)
    spacing = lambda x, y: 7.0 + 7.0 * min(1.0, min(x, w - 16 - x, y, d - 16 - y) / 40)
    pts = _poisson(w - 16, d - 16, spacing, seed=21) - ((w - 16) / 2, (d - 16) / 2)
    cells = [c for c in _voronoi_cells(pts, region) if not c.buffer(-2.2).is_empty]
    top = cq.Plane(origin=(0, 0, PLATE_T), xDir=(1, 0, 0), normal=(0, 0, 1))
    cutters = [cq.Workplane(top).workplane(offset=0.5).polyline(_ring(c, 32)).close().extrude(-(PLATE_T + 1)).val()
               for c in cells]
    return cq.Workplane("XY").add(plate.val().cut(*cutters))


def _mount_everything(plate):
    plate = plate.union(motor_saddles()).union(_mega_cradle()).union(_driver_wall()).union(_switch_bracket())
    plate = plate.union(_buck_holder())
    for x, y, a in TIE_LOOPS:
        plate = plate.union(_tie_loop(x, y, a))
    return plate


PARTS = {"base_plate": base_plate, "strap_gearbox": strap_gearbox, "strap_motor": strap_motor}
