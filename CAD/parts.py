"""Printed parts, modelled in CadQuery. Units: mm.

Geometry comes from sim/params.py so the CAD and the sim never disagree.
Bought-part dimensions marked PLACEHOLDER wait on the bought-parts research.
"""

import math

import cadquery as cq

from sim.params import GEOMETRY

IN = 25.4

# --- touch panel and plate holder -------------------------------------------
PANEL_W, PANEL_H = 187.0, 141.0      # 8.4" resistive panel outline
PANEL_T = 3.0                        # Matt measured ~3 mm (glass); about 198 g at 187 x 141 x 3
PANEL_CLEAR = 1.0                    # per side, pocket around the panel (Matt: 1 mm looser than the first print)
LIP_ABOVE_PANEL = 0.25 * IN          # Matt: 0.25" lip keeps the ball on
FLOOR_T = 2.6                        # the glass ledge's thickness
WALL = 6.0                           # lip width
CORNER_R = 10.0

# --- joints (bought parts: cad/hardware.py) -----------------------------------
from . import hardware as hw  # noqa: E402

INSERT_M3_HOLE = (4.0, 7.0)          # ruthex M3 x 5.7 heat-set insert: hole dia, depth (rod top, arm tip)
PRESS_FIT = -0.08                    # bore undersize for pressing in bushings
HOLDER_JOINT_DROP = 13.0             # ball centre below the holder underside (sim: model.HOLDER_Z)

# Elbow stack along the shaft axis, measured from the rod's centre plane (mm):
SHOULDER_END = hw.SHOULDER[1] / 2    # the shoulder bottoms on the arm's face here (arm side)
# The rod eye is asymmetric: it runs from the arm's boss (0.1 mm gap) to where the single bushing's
# flange starts on the bolt-head side (0.1 mm gap under the head): 0.2 mm end play in all.
EYE_ARM = SHOULDER_END - 0.1                                 # eye face toward the arm, from the rod plane
EYE_HEAD = SHOULDER_END - hw.BUSHING["flange_t"] - 0.1       # eye face toward the bolt head (flange outside it)
EYE_HALF = EYE_HEAD                                          # (sim: where the bushing flange sits)
ARM_NEAR = 6.35                      # arm face from the rod plane: kept where it was so the motor stack doesn't move
COLLAR_POCKET = hw.HUB_COLLAR[1]     # the coupling's collar sinks into the arm; its flange bolts to the far face
SET_SCREW_ACCESS = 6.0               # vertical hole onto the collar's set screw: the M3 set screw and its key both fit through

# --- arm ----------------------------------------------------------------------
ARM_T = 0.5 * IN                     # Matt: about 0.5" thick
HUB_R, TIP_R = 13.0, 7.5             # hub boss carries the 22 mm flange coupling
ARM_WAIST_R = 40.0                   # concave sides: soft, organic profile

# --- rod ----------------------------------------------------------------------
ROD_EYE_OD = 17.0
ROD_W = 8.0                          # shaft section, radial
ROD_T = 8.8                          # shaft thickness along the pin: room for the M3 insert at the top
SPOT_FACE = hw.BUSHING["flange_od"] + 1.0   # flat recess round the eye on the head side for the flange
ROD_TOP = 33.0                       # elbow centre to the rod's top face (M3 stud's insert here)


def _holder_dims():
    pocket_w, pocket_h = PANEL_W + 2 * PANEL_CLEAR, PANEL_H + 2 * PANEL_CLEAR
    return pocket_w, pocket_h, pocket_w + 2 * WALL, pocket_h + 2 * WALL, PANEL_T + LIP_ABOVE_PANEL


LEDGE = 6.0                          # (used to place the joints' reach; the insert itself is solid)
FPC_SLOT = (4.0, 30.0)               # the panel's tail goes down through the insert here, by the -x short side
RIBBON_CLIP = (18.0, 1.8)            # bridges under the insert that hold the tail flat: opening width, gap (measure your tail)
RIBBON_CLIP_X = (22.0, 48.0)         # their distance in from the tail slot, along +x
INSERT_FIT = 0.2                     # per side, insert inside the rim
STEP = 1.5                           # the rim's inward step the insert sits on (glued)


def _joint_layout():
    """Per leg: (tab centre, arm end on the ledge, leg unit vector, tangent) in holder coordinates."""
    pocket_w, pocket_h, _, _, _ = _holder_dims()
    r = GEOMETRY.plate_joint_radius * 1000
    face = hw.BALL_D / 2 + 0.5 + hw.STUD_STANDOFF       # tab face, from the ball centre along the stud
    out = []
    for a in GEOMETRY.leg_angles_deg:
        rad = math.radians(a)
        u, t = (math.cos(rad), math.sin(rad)), (-math.sin(rad), math.cos(rad))
        tab = (r * u[0] + (face + 2.5) * t[0], r * u[1] + (face + 2.5) * t[1])
        # Arm end: straight out from the tab along the leg, to the middle of the ledge band (under the glass).
        bx, by = pocket_w / 2 - INSERT_FIT - LEDGE / 2, pocket_h / 2 - INSERT_FIT - LEDGE / 2
        kx = (bx - abs(tab[0])) / abs(u[0]) if abs(u[0]) > 1e-6 else 1e9
        ky = (by - abs(tab[1])) / abs(u[1]) if abs(u[1]) > 1e-6 else 1e9
        k = min(kx, ky)
        out.append((tab, (tab[0] + k * u[0], tab[1] + k * u[1]), u, t))
    return out


def plate_rim() -> cq.Workplane:
    """Outer wall and the 0.25" lip, with a STEP-wide ledge at the bottom the insert sits on (glued).
    Nothing else shows from the side. Print flat."""
    pocket_w, pocket_h, outer_w, outer_h, depth = _holder_dims()
    h = FLOOR_T + depth
    body = (cq.Workplane("XY").workplane(offset=-STEP).sketch().rect(outer_w, outer_h).vertices().fillet(CORNER_R)
            .finalize().extrude(h + STEP))
    body = body.cut(cq.Workplane("XY").sketch().rect(pocket_w, pocket_h).vertices().fillet(1.5).finalize().extrude(h))
    body = body.cut(cq.Workplane("XY").workplane(offset=-STEP).sketch()
                    .rect(pocket_w - 2 * STEP, pocket_h - 2 * STEP).vertices().fillet(1.5).finalize().extrude(STEP))
    return body.edges(">Z").fillet(1.2)


def plate_insert() -> cq.Workplane:
    """The plate the glass sits on: a solid rectangle that slots into the rim onto its step (a little
    super glue), with three nubs underneath for the ball joints, a slot for the panel's tail, and two
    low bridges to tuck the tail flat under the plate.
    The stud runs sideways (tangential) through each nub, so the socket hangs from the ball with the
    rod below it. Print flat, nubs up."""
    pocket_w, pocket_h, _, _, _ = _holder_dims()
    w, h = pocket_w - 2 * INSERT_FIT, pocket_h - 2 * INSERT_FIT
    body = cq.Workplane("XY").sketch().rect(w, h).vertices().fillet(1.3).finalize().extrude(FLOOR_T)
    body = body.cut(cq.Workplane("XY").center(-w / 2 + 2.0 + FPC_SLOT[0] / 2, 0)
                    .slot2D(FPC_SLOT[1], FPC_SLOT[0], 90).extrude(FLOOR_T))        # tail slot
    # Ribbon clips: low bridges on the underside, in line with the tail slot, to tuck the tail flat under the plate.
    slot_x = -w / 2 + 2.0 + FPC_SLOT[0] / 2
    ow, gap = RIBBON_CLIP
    for dx in RIBBON_CLIP_X:
        clip = cq.Workplane("XY").center(slot_x + dx, 0).rect(6, ow + 6).extrude(-(gap + 1.6))
        clip = clip.cut(cq.Workplane("XY").center(slot_x + dx, 0).rect(7, ow).extrude(-gap))
        body = body.union(clip.edges("|X and <Z").fillet(0.6))
    zb = -HOLDER_JOINT_DROP
    tab_t, tab_w = 5.0, 12.0
    for tab, _, u, t in _joint_layout():
        a = math.degrees(math.atan2(u[1], u[0]))
        body = body.union(cq.Workplane("XY").center(*tab).transformed(rotate=(0, 0, a)).rect(tab_w, tab_t)
                          .extrude(zb - 6).edges("<Z").fillet(2.0))
        body = body.cut(cq.Workplane(cq.Plane(origin=(tab[0], tab[1], zb), xDir=(u[0], u[1], 0), normal=(t[0], t[1], 0)))
                        .circle(1.65).extrude(5, both=True))                       # M3 clearance for the ball screw
    return body


def plate_holder() -> cq.Workplane:
    """Both pieces as assembled (for the sim)."""
    return plate_rim().union(plate_insert())


def _waisted_link(L, r1, r2, rw) -> cq.Wire:
    """Outline of two bosses joined by concave arcs of radius rw tangent to both."""
    x = ((r1 + rw) ** 2 - (r2 + rw) ** 2 + L**2) / (2 * L)
    y = math.sqrt((r1 + rw) ** 2 - x**2)

    def tangent(cx, cy, r, sgn):  # point on boss (cx,0,r) toward the arc centre (x, sgn*y)
        dx, dy = x - cx, sgn * y
        n = math.hypot(dx, dy)
        return (cx + r * dx / n, r * dy / n)

    def concave_mid(sgn):  # midpoint of the waist arc
        a1 = math.atan2(tangent(0, 0, r1, sgn)[1] - sgn * y, tangent(0, 0, r1, sgn)[0] - x)
        a2 = math.atan2(tangent(L, 0, r2, sgn)[1] - sgn * y, tangent(L, 0, r2, sgn)[0] - x)
        a = (a1 + a2) / 2
        return (x + rw * math.cos(a), sgn * y + rw * math.sin(a))

    h_top, h_bot = tangent(0, 0, r1, 1), tangent(0, 0, r1, -1)
    t_top, t_bot = tangent(L, 0, r2, 1), tangent(L, 0, r2, -1)
    return (cq.Workplane("XY").moveTo(*h_top)
            .threePointArc(concave_mid(1), t_top)
            .threePointArc((L + r2, 0), t_bot)
            .threePointArc(concave_mid(-1), h_bot)
            .threePointArc((-r1, 0), h_top)
            .close().wire().val())


def arm() -> cq.Workplane:
    """Crank on a 6 mm rigid flange coupling (collar sunk in the arm, flange bolted with 4x M3);
    elbow tip carries an M3 insert for the shoulder bolt.

    Origin at the shaft axis, arm along +x, thickness along z (the shaft axis).
    z = 0 is the face toward the rod; the coupling's collar sits in a pocket in the z = ARM_T face.
    """
    L = GEOMETRY.arm_length * 1000
    body = cq.Workplane("XY").add(_waisted_link(L, HUB_R, TIP_R, ARM_WAIST_R)).toPending().extrude(ARM_T)
    body = body.faces(">Z or <Z").edges().fillet(2.5)

    od, _, bc = hw.HUB
    cd = hw.HUB_COLLAR[0]
    body = body.cut(cq.Workplane("XY").workplane(offset=ARM_T - COLLAR_POCKET).circle(cd / 2 + 0.2).extrude(COLLAR_POCKET))
    body = body.cut(cq.Workplane("XY").circle(3.3).extrude(ARM_T))                       # shaft tip clearance
    holes = cq.Workplane("XY").polarArray(bc / 2, 0, 360, 4)       # bolts on +-x and +-y: the set screws sit in line with two of them
    body = body.cut(holes.circle(1.65).extrude(ARM_T))                                   # M3 clearance
    body = body.cut(holes.circle(3.0).extrude(3.0))                                      # screw-head counterbores, rod side
    # The coupling has two set screws, 180 deg apart, in line with two of its flange holes (measured):
    # with the bolt pattern square to the arm they point along +-y, at these access holes. Each access
    # hole passes one flange bolt; tighten the set screws before bolting the arm on (or take that bolt
    # out to reach one later). Turn them to point up and down at assembly:
    # the top one is reached from above (plate off, shell lifted), the bottom one from below with
    # the short leg of an L-key (about 23 mm clear above the base plate). +y here is up.
    for side in (1, -1):
        body = body.cut(cq.Workplane("XZ", origin=(0, side * (HUB_R + 1), 0)).center(0, ARM_T - COLLAR_POCKET / 2)
                        .circle(SET_SCREW_ACCESS / 2).extrude(side * (HUB_R - cd / 2 + 1.2)))
    # Boss on the rod-side face of the tip: the shoulder bottoms on it, closing the gap from the arm
    # face (ARM_NEAR) to the end of the 10 mm shoulder.
    boss_h = ARM_NEAR - SHOULDER_END
    body = body.union(cq.Workplane("XY").center(L, 0).circle(4.0).extrude(-boss_h))
    body = body.cut(cq.Workplane("XY").workplane(offset=-boss_h).center(L, 0).circle(INSERT_M3_HOLE[0] / 2)
                    .extrude(INSERT_M3_HOLE[1]))
    return body


def rod() -> cq.Workplane:
    """Elbow eye (one pressed flanged bushing, from the bolt-head side) up to an M3 insert in the top face.

    An M3 grub screw (stud) goes into the insert with a jam nut against the top face, and the
    uxcell tie-rod-end socket threads onto its top with its own nut: no glue. Turning the
    socket (0.5 mm per turn) sets each leg's length for levelling.

    Origin at the elbow pin centre, rod along +z, eye axis along y.
    """
    b = hw.BUSHING
    eye = cq.Workplane("XZ", origin=(0, -EYE_ARM, 0)).circle(ROD_EYE_OD / 2).extrude(-(EYE_ARM + ROD_T / 2))
    shaft = (cq.Workplane("XY").sketch().rect(ROD_W, ROD_T).vertices().fillet(2.5).finalize()
             .extrude(ROD_TOP))
    body = eye.union(shaft)
    # Spot face on the head side down to EYE_HEAD, where the bushing flange sits.
    body = body.cut(cq.Workplane("XZ", origin=(0, EYE_HEAD, 0)).circle(SPOT_FACE / 2).extrude(-ROD_T))
    # Bushing bore from the head side (press fit, the 6 mm body), then pin clearance through the rest.
    body_len = b["length"] - b["flange_t"]
    body = body.cut(cq.Workplane("XZ", origin=(0, EYE_HEAD, 0)).circle((b["od"] + PRESS_FIT) / 2).extrude(body_len))
    body = body.cut(cq.Workplane("XZ", origin=(0, EYE_HEAD + 1, 0)).circle(hw.SHOULDER[0] / 2 + 0.3).extrude(20))
    # M3 heat-set insert in the top face; an M3 grub-screw stud carries the tie-rod-end socket.
    body = body.cut(cq.Workplane("XY").workplane(offset=ROD_TOP - INSERT_M3_HOLE[1]).circle(INSERT_M3_HOLE[0] / 2)
                    .extrude(INSERT_M3_HOLE[1] + 1))
    return body


PARTS = {"plate_rim": plate_rim, "plate_insert": plate_insert, "arm": arm, "rod": rod}
