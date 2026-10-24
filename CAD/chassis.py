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
STRAP_PRELOAD = 2.5                  # the arch is flattened this much: snug on the motor, the ears still float
                                     # 2.5 mm over the saddle, so the screws pull the strap down and it springs tight
SCREW_HEAD_R = 3.1                   # M3 socket head + clearance
WRENCH_CLEAR = 7.5                   # screw centre to the arch: room for an open-end wrench on an M3 nut/hex head
GEARBOX_SADDLE = (MOTOR_FACE - 3.0, MOTOR_FACE - hw.GA37_GEARBOX[1] + 2.0)      # t0, t1 along the tangent
MOTOR_SADDLE = (MOTOR_FACE - hw.GA37_GEARBOX[1] - 0.3, MOTOR_FACE - hw.GA37_GEARBOX[1] - 14.0)

# Electronics layout (from a free-space search between the motors and arm swings)
MEGA_AT = (88.0, 20.0)               # standing on edge, board along y, parts facing -x; toward the back wall
MEGA_LIFT = 9.0                      # board's bottom edge above the plate: room for the Dupont plugs below it
                                     # (the liner's inside top, ~67.4 mm, caps it: board top 66.3)
MEGA = (101.6, 53.3)
MEGA_HOLES = [(13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56), (90.17, 50.8), (96.52, 2.54)]
# Arduino Mega 2560 R3 holes, mm, seen from the component side: x from the USB / power-jack edge, y up from
# the analog / power header edge (verified against SparkFun's ARDUINO_MEGA footprint).
# Two Cytron MDD3A drivers, back to back on one upright wall in the front-right corner, beside the Mega.
MDD3A = (48.26, 41.66)               # board: terminals-to-header length, width (datasheet)
MDD3A_HOLES = (43.18, 36.58)         # hole spacing, same axes; 3 mm holes: top two used, M3 x 6 into heat-set inserts
DRIVER_WALL_X = 76.5                 # wall centre line (runs along y)
DRIVER_Y = -72.5                     # boards' centre along y
DRIVER_STANDOFF = 6.0
POWER_SWITCH_AT = (-96.0, 60.0)      # Pololu #2813: on a small upright plate (foam tape + zip tie) by the left wall,
                                     # facing the centre so its terminals are easy to reach
POWER_SWITCH = (20.3, 25.4)          # 0.8" x 1.0" (Pololu); terminal blocks on a 0.8" edge
# Bolt-on holders: each sits on the plate and screws (M3 x 8) through tabs into raised pads with inserts.
PAD_H, TAB_T, TAB_W = 4.0, 4.0, 9.0
from .enclosure import POWER_TAB  # noqa: E402  (the jack lives in the frame's back tab)
BUCK_AT = (-84.0, -61.0)             # 12 V -> 5 V buck: DROK B07P663XJV, sealed case with wire leads
BUCK = (51.0, 25.0)                  # over the mounting feet, body width (DROK drawing); body 34 long, 21 tall
BUCK_FOOT_W = 18.0                   # mounting feet width
BUCK_ROT = 90

# Zip-tie loops: (x, y, route direction in degrees)
TIE_LOOPS = [
    (-75.0, 60.0, 0), (-20.0, 74.0, 0),                # jack and switch leads
    (5.0, 60.0, 0), (40.0, 40.0, 60),                   # back run to the Mega
    (60.0, 20.0, 90),                                   # along the Mega
    (-35.0, -60.0, 0),                                  # front run
    (-74.0, 20.0, 90),                                  # touch-panel ribbon drop
    (0.0, -82.0, 0),                                    # button wires from the front notch
    (-62.0, -56.0, 90),                                 # buck converter leads out to the Wagos
]
MOTOR_TIE_T = (-68.0, -86.0)          # extra loops beside every motor (along its length), for its cable


def _leg_frames():
    for a in GEOMETRY.leg_angles_deg:
        r = math.radians(a)
        yield a, np.array([math.cos(r), math.sin(r)]), np.array([-math.sin(r), math.cos(r)])


def _insert_offset(radius):
    """Distance from the motor axis to each strap screw: a wrench fits round it, clear of the arch."""
    return radius + CRADLE_CLEAR + STRAP_T + WRENCH_CLEAR


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


def _tab(x, y, z0, hole=None):
    """A screw tab sitting on a pad: M3 clearance with the head sunk in. `hole` puts the screw off the
    tab's centre (to land on an already printed pad while the tab stays flush with a flat back)."""
    hx, hy = hole if hole else (x, y)
    t = (cq.Workplane("XY").workplane(offset=z0).center(x, y).rect(TAB_W, TAB_W).extrude(TAB_T).edges("|Z").fillet(2))
    t = t.cut(cq.Workplane("XY").workplane(offset=z0).center(hx, hy).circle(1.7).extrude(TAB_T))
    return t.cut(cq.Workplane("XY").workplane(offset=z0 + TAB_T - 1.8).center(hx, hy).circle(3.1).extrude(2))


def _web(a, tab):
    """A 3 mm web from attach point a to the near edge of the tab at `tab`, stopping short of its screw hole."""
    a, tab = np.array(a, float), np.array(tab, float)
    d = tab - a
    n = np.linalg.norm(d)
    b = tab - d / n * (TAB_W / 2)
    length = np.linalg.norm(b - a)
    if length < 0.5:
        return None
    c = (a + b) / 2
    ang = math.degrees(math.atan2(d[1], d[0]))
    return (cq.Workplane("XY").workplane(offset=PLATE_T).center(*c).transformed(rotate=(0, 0, ang))
            .rect(length + 1.0, 3).extrude(PAD_H + TAB_T))


def _attach(body, a, tab, hole=None):
    web = _web(a, tab)
    body = body if web is None else body.union(web)
    return body.union(_tab(tab[0], tab[1], PLATE_T + PAD_H, hole))


def _mega_holes():
    """(along y from the board's -y end, up from its bottom edge) for each hole. The board stands with its
    component side facing the centre (-x): seen from there, +y is to the LEFT, so the drawing's x runs
    toward -y. USB / power-jack edge at the +y (back) end, analog / power header along the bottom."""
    L, H = MEGA
    return [(L - hx, hy) for hx, hy in MEGA_HOLES]


def _mega_tabs():
    """Two tabs off the wall's ends, four low in front of it (under the board, below its plugs)."""
    x0, y0 = MEGA_AT
    ends = [(x0 + 11.5, y0 + s * (MEGA[0] / 2 + 4 + TAB_W / 2 + 1)) for s in (-1, 1)]   # screw = the printed pads
    front = [(x0 - 3.0, y0 + dy) for dy in (-40.0, -18.0, 18.0, 40.0)]
    return ends + front


def mega_mount() -> cq.Workplane:
    """Upright 4 mm wall for the Mega (standing on edge, parts facing the centre), four insert standoffs,
    a ledge to rest the board on while starting the screws, and two screw tabs. World position;
    print lying on its back (standoffs up)."""
    x0, y0 = MEGA_AT
    wall_x = x0 + 10.0
    L, H = MEGA
    z_board = PLATE_T + MEGA_LIFT
    top = z_board + H
    wall = (cq.Workplane("XY").workplane(offset=PLATE_T).center(wall_x + 2, y0).rect(4, L + 8).extrude(top - PLATE_T)
            .edges("|X and >Z").fillet(2))          # 4 mm thick: no gussets behind it
    standoff = 6.0
    for hx, hy in _mega_holes():
        if hy > H - 6.0:      # the two holes at the top edge: their inserts would break out of the wall's top
            continue          # (the liner caps the height), so the board takes 4 screws plus the ledge
        y, z = y0 - L / 2 + hx, z_board + hy
        st = (cq.Workplane("YZ", origin=(wall_x, 0, 0)).center(y, z).circle(3.5).extrude(-standoff)
              .cut(cq.Workplane("YZ", origin=(wall_x - standoff, 0, 0)).center(y, z).circle(INSERT_M3[0] / 2)
                   .extrude(INSERT_M3[1])))
        wall = wall.union(st)
    wall = wall.intersect(cq.Workplane("XY").rect(1000, 1000).extrude(top))
    # The ledge the board's bottom edge rests on: a solid block back to the wall (it used to float in
    # front of it, held only by the base plate). All below the board, so its plugs stay clear.
    ledge_x0 = wall_x - standoff - 3.0
    ledge = (cq.Workplane("XY").workplane(offset=PLATE_T).center((ledge_x0 + wall_x) / 2 + 0.25, y0)
             .rect(wall_x - ledge_x0 + 0.5, L - 10).extrude(MEGA_LIFT))
    wall = wall.union(ledge)
    for x, y in _mega_tabs()[:2]:                    # tabs off the wall's ends: body flush with the back, screw on the pad
        xb = wall_x + 4.0 - TAB_W / 2
        wall = _attach(wall, (xb, y0 + (1 if y > y0 else -1) * (L / 2 + 4)), (xb, y), hole=(x, y))
    for x, y in _mega_tabs()[2:]:                    # low front tabs off the ledge (all below the board, z < 13)
        wall = _attach(wall, (wall_x - standoff - 3.0, y), (x, y))
    return wall


def _driver_tabs():
    # At the wall's ends, where the printed base's pads are (the bottom board posts are gone, so a
    # screwdriver reaches them straight down).
    return [(DRIVER_WALL_X + sx * 7.0, y) for sx in (-1, 1) for y in (DRIVER_Y + 21.5, DRIVER_Y - 17.5)]


def driver_mount() -> cq.Workplane:
    """Upright wall carrying both MDD3As back to back, one on each face, each on its two top holes
    (M3 heat-set inserts in the posts, M3 x 6 screws). Terminal blocks along the top edge, input header along the bottom, MEGA_LIFT above
    the plate like the Mega. Four screw tabs. World position; print standing up."""
    L, W = MDD3A
    hl, hw_ = MDD3A_HOLES
    z0 = PLATE_T + MEGA_LIFT
    top = z0 + L + 1.0
    wall = (cq.Workplane("XY").workplane(offset=PLATE_T).center(DRIVER_WALL_X, DRIVER_Y).rect(3, W + 6).extrude(top - PLATE_T)
            .edges("|X and >Z").fillet(1.5))
    for side in (1, -1):
        x_face = DRIVER_WALL_X + side * 1.5
        for dy in (-hw_ / 2, hw_ / 2):
            dz = hl / 2                            # top posts only: the bottom two crowded the base tabs
            y, z = DRIVER_Y + dy, z0 + L / 2 + dz
            post = (cq.Workplane("YZ", origin=(x_face, 0, 0)).center(y, z).circle(3.5)
                    .extrude(side * DRIVER_STANDOFF))
            hole = (cq.Workplane("YZ", origin=(x_face + side * DRIVER_STANDOFF, 0, 0)).center(y, z)
                    .circle(INSERT_M3[0] / 2).extrude(-side * (DRIVER_STANDOFF + 0.2)))   # M3 heat-set insert
            wall = wall.union(post).cut(hole)
    for x, y in _driver_tabs():                     # tabs on both sides, between the board standoffs
        wall = _attach(wall, (DRIVER_WALL_X + (1.5 if x > DRIVER_WALL_X else -1.5), y), (x, y))
    return wall


def _switch_tabs():
    x0, y0 = POWER_SWITCH_AT
    return [(x0 - 1.5, y0 + s * 20.5) for s in (-1, 1)]   # screw = the printed pads


def switch_mount() -> cq.Workplane:
    """Small upright plate for the Pololu switch (no mounting holes on the board): stick the board to the
    plate's face toward the centre (+x) with double-sided foam tape, terminal blocks out, then a zip tie
    through the two slots holds it. Two screw tabs. World position; print standing up."""
    x0, y0 = POWER_SWITCH_AT
    w, h = 32.0, 30.0
    plate = (cq.Workplane("XY").workplane(offset=PLATE_T).center(x0 - 2, y0).rect(4, w).extrude(h)
             .edges("|X and >Z").fillet(2))
    for sy in (-1, 1):                               # zip-tie slots beside the board
        plate = plate.cut(cq.Workplane("XY").workplane(offset=PLATE_T + h / 2 - 2).center(x0 - 1.5, y0 + sy * 14.0)
                          .rect(4, 2.6).extrude(4))
    for x, y in _switch_tabs():                      # body flush with the back, screw on the pad
        xb = x0 - 4.0 + TAB_W / 2
        plate = _attach(plate, (xb, y0 + (1 if y > y0 else -1) * w / 2), (xb, y), hole=(x, y))
    return plate


def _motor_numbers():
    """Raised 1 / 2 / 3 on the floor under each motor's back end, reading from outside."""
    rs = GEOMETRY.motor_radius * 1000
    out = None
    for k, (a, u, t) in enumerate(_leg_frames()):
        p = rs * u + (MOTOR_SADDLE[1] - 14.0) * t
        n = (cq.Workplane("XY").workplane(offset=PLATE_T).center(*p).transformed(rotate=(0, 0, a - 90))
             .text(str(k + 1), 12.0, 0.8, halign="center", valign="center"))
        out = n if out is None else out.union(n)
    return out


def _pads():
    """Raised pads with an M3 insert under every holder tab."""
    pts = _mega_tabs() + _driver_tabs() + _switch_tabs()
    pads = cq.Workplane("XY").workplane(offset=PLATE_T).pushPoints(pts).rect(TAB_W, TAB_W).extrude(PAD_H).edges("|Z").fillet(2)
    return pads.cut(cq.Workplane("XY").workplane(offset=PLATE_T + PAD_H - INSERT_M3[1]).pushPoints(pts)
                    .circle(INSERT_M3[0] / 2).extrude(INSERT_M3[1] + 0.1))


def _motor_tie_loops():
    """Two loops beside each motor, clear of the saddles, for its encoder and power cable."""
    rs = GEOMETRY.motor_radius * 1000
    out = []
    for a, u, t in _leg_frames():
        for tt in MOTOR_TIE_T:
            for sgn in (-1, 1):
                p = rs * u + tt * t + sgn * (hw.GA37_MOTOR[0] / 2 + 8.0) * u
                out.append((p[0], p[1], a + 90))
    return out


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
    plate = plate.union(motor_saddles()).union(_pads()).union(_buck_holder()).union(_motor_numbers())
    clear = _clear_area()
    for x, y, a in TIE_LOOPS + _motor_tie_loops():
        if clear.contains(Point(x, y).buffer(7)):
            plate = plate.union(_tie_loop(x, y, a))
    return plate


def _clear_area():
    """Floor not taken by saddles, holders, pads or the plate's rounded edge (for placing tie loops)."""
    w, d = PLATE_W, PLATE_D
    area = box(-w / 2 + 6, -d / 2 + 6, w / 2 - 6, d / 2 - 6)
    rs = GEOMETRY.motor_radius * 1000
    for _, u, t in _leg_frames():
        for (t0, t1), rad in ((GEARBOX_SADDLE, hw.GA37_GEARBOX[0] / 2), (MOTOR_SADDLE, hw.GA37_MOTOR[0] / 2)):
            h = _saddle_width(rad) / 2
            area = area.difference(Polygon([rs * u + t0 * t - h * u, rs * u + t0 * t + h * u,
                                            rs * u + t1 * t + h * u, rs * u + t1 * t - h * u]).buffer(1))
    x0, y0 = MEGA_AT
    area = area.difference(box(x0 - 20, y0 - MEGA[0] / 2 - 12, x0 + 22, y0 + MEGA[0] / 2 + 12))
    area = area.difference(box(DRIVER_WALL_X - 22, DRIVER_Y - 26, DRIVER_WALL_X + 22, DRIVER_Y + 27))
    sx, sy = POWER_SWITCH_AT
    area = area.difference(box(sx - 8, sy - 26, sx + 16, sy + 26))
    bx, by = BUCK_AT
    return area.difference(box(bx - 16, by - 29, bx + 16, by + 29))


PARTS = {"base_plate": base_plate, "strap_gearbox": strap_gearbox, "strap_motor": strap_motor,
         "mega_mount": mega_mount, "driver_mount": driver_mount, "switch_mount": switch_mount}
