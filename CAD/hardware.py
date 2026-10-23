"""Bought parts, modelled from their listings (bought-parts research, ticket 014).

Visual and fit models only: enough to check clearances and to see the real
linkage in the viewer. Dimensions marked (I) are inferred, not published;
measure them on arrival. Units: mm.
"""

import cadquery as cq

IN = 25.4

# GA37-520 30:1 with hall encoder (research ticket 002)
GA37_GEARBOX = (37.0, 22.0)       # diameter, length
GA37_MOTOR = (33.0, 31.0)         # (I)
GA37_ENCODER = (33.0, 15.0)       # (I)
GA37_SHAFT = (6.0, 15.0)          # D-shaft diameter, length from the face
GA37_SHAFT_OFFSET = 7.0           # shaft axis is off the gearbox axis

# 6 mm rigid flange coupling, 4-pack (Amazon B08334N261, ~$7): listing drawing gives flange 22,
# collar 10, bore 6, M3 set screw, 4x 2.85 mm holes. Bolt circle 16 mm is (I) from sellers of the
# same design: measure before printing the arm.
HUB = (22.0, 12.0, 16.0)          # flange OD, overall height, bolt circle (4 holes)
HUB_FLANGE_T = 2.0
HUB_COLLAR = (10.0, 10.0)         # diameter, height
HUB_HOLE = 2.85

# Elbow pin: uxcell 304 stainless shoulder bolt, 4 mm x 10 mm shoulder, M3 x 6 mm thread, 12-pack (B0DP71MPP7)
SHOULDER = (4.0, 10.0)            # diameter, length
BOLT_HEAD = (7.0, 3.5)            # diameter, height
BOLT_THREAD = (3.0, 6.0)          # M3, length

# HARFINGTON flanged sintered-bronze bushing, 4 mm bore x 8 mm OD x 8 mm total length (6 mm body),
# 12 mm x 2 mm flange, 6-pack (B0CGDT9YYW). One per elbow, pressed in from the bolt-head side of the rod eye.
BUSHING = dict(bore=4.0, od=8.0, length=8.0, flange_od=12.0, flange_t=2.0)

# APIELE 16 mm momentary push button, 12 V LED angel-ring head, 1NO1NC, wire socket plug
# (Amazon B091B26J4V). Listing gives the 16 mm hole only; the rest is (I), typical of this style.
BUTTON_HOLE = 16.0
BUTTON_BEZEL = (18.5, 2.0)        # head diameter, height proud of the panel
BUTTON_CAP = 12.0                 # actuator diameter inside the LED ring
BUTTON_THREAD = (16.0, 19.0)      # M16 body behind the panel
BUTTON_NUT = (19.0, 3.0)          # hex across flats, thickness
BUTTON_BLOCK = (14.0, 9.0)        # switch block / terminals
BUTTON_PLUG = (10.0, 7.0, 12.0)   # harness plug: width, height, length

# uxcell 10Pcs M3 3.0xL29mm tie-rod end ball head, with screws and nuts (Amazon B07QPXWDCR).
# Plastic socket on a steel ball; M3 thread. Ball size, swivel angle and socket proportions are (I):
# check one when they arrive (need >= 20 deg of swivel).
BALL_D = 4.8
STUD_HEX = 5.5                    # across flats (I)
STUD_STANDOFF = 3.0
STUD_THREAD = (3.0, 8.0)          # M3, length (I)
SOCKET_BODY = (9.0, 8.0)          # width across the ball, thickness along the stud (I)
SOCKET_SHANK = (6.0, 24.0)        # diameter, length below the ball centre (L29 overall, I)
LINK_LENGTH = 29.0

def button(panel_t=3.0) -> cq.Workplane:
    """Push button body. Origin on the panel's outer face, +z out of the panel (the head),
    -z into the console (thread, nut on the panel's inner face, switch block, harness plug)."""
    bz, bh = BUTTON_BEZEL
    head = cq.Workplane("XY").circle(bz / 2).circle(BUTTON_CAP / 2 + 1.6).extrude(bh).edges(">Z").fillet(0.6)
    cap = cq.Workplane("XY").circle(BUTTON_CAP / 2).extrude(bh - 0.4)
    thread = cq.Workplane("XY").circle(BUTTON_THREAD[0] / 2 - 0.1).extrude(-BUTTON_THREAD[1])
    nut = (cq.Workplane("XY").workplane(offset=-panel_t).polygon(6, BUTTON_NUT[0] / 0.866)
           .circle(BUTTON_THREAD[0] / 2).extrude(-BUTTON_NUT[1]))
    block = cq.Workplane("XY").workplane(offset=-BUTTON_THREAD[1]).circle(BUTTON_BLOCK[0] / 2).extrude(-BUTTON_BLOCK[1])
    w, h, l = BUTTON_PLUG
    plug = cq.Workplane("XY").workplane(offset=-BUTTON_THREAD[1] - BUTTON_BLOCK[1]).rect(w, h).extrude(-l)
    return head.union(cap).union(thread).union(nut).union(block).union(plug)


def button_ring() -> cq.Workplane:
    """The LED ring between the bezel and the cap (rendered lit)."""
    return cq.Workplane("XY").circle(BUTTON_CAP / 2 + 1.6).circle(BUTTON_CAP / 2).extrude(BUTTON_BEZEL[1] - 0.2)


def ga37() -> cq.Workplane:
    """Shaft along +z from the origin (shaft axis); body along -z."""
    gd, gl = GA37_GEARBOX
    md, ml = GA37_MOTOR
    ed, el = GA37_ENCODER
    off = GA37_SHAFT_OFFSET
    body = cq.Workplane("XY").center(0, -off).circle(gd / 2).extrude(-gl)
    body = body.union(cq.Workplane("XY").workplane(offset=-gl).center(0, -off).circle(md / 2).extrude(-ml))
    body = body.union(cq.Workplane("XY").workplane(offset=-gl - ml).center(0, -off).circle(ed / 2).extrude(-el))
    shaft = (cq.Workplane("XY").circle(GA37_SHAFT[0] / 2).extrude(GA37_SHAFT[1])
             .cut(cq.Workplane("XY").center(GA37_SHAFT[0] / 2 + 2.5, 0).rect(6, 8).extrude(GA37_SHAFT[1])))
    return body.union(shaft)


def hub() -> cq.Workplane:
    """Flange face at z = 0, collar along +z; M3 set screw radial in the collar."""
    od, _, bc = HUB
    flange = cq.Workplane("XY").circle(od / 2).circle(3.0).extrude(HUB_FLANGE_T)
    collar = cq.Workplane("XY").workplane(offset=HUB_FLANGE_T).circle(HUB_COLLAR[0] / 2).circle(3.0).extrude(HUB_COLLAR[1])
    body = flange.union(collar)
    body = body.cut(cq.Workplane("XY").polarArray(bc / 2, 0, 360, 4).circle(HUB_HOLE / 2).extrude(HUB_FLANGE_T))
    # Two set screws, 180 deg apart, along y: in line with the flange holes on +-y (as on the real part).
    return body.cut(cq.Workplane("XZ").center(0, HUB_FLANGE_T + HUB_COLLAR[1] / 2).circle(1.25)
                    .extrude(HUB_COLLAR[0], both=True))


def shoulder_bolt() -> cq.Workplane:
    """Head at the origin, shoulder along -z, then the M3 thread."""
    head = cq.Workplane("XY").circle(BOLT_HEAD[0] / 2).extrude(BOLT_HEAD[1]).edges(">Z").fillet(0.8)
    head = head.cut(cq.Workplane("XY").workplane(offset=BOLT_HEAD[1] - 2.5).polygon(6, 2.9).extrude(2.6))
    shoulder = cq.Workplane("XY").circle(SHOULDER[0] / 2).extrude(-SHOULDER[1])
    thread = cq.Workplane("XY").workplane(offset=-SHOULDER[1]).circle(BOLT_THREAD[0] / 2).extrude(-BOLT_THREAD[1])
    return head.union(shoulder).union(thread)


def bushing() -> cq.Workplane:
    """Flange face at the origin, body along -z."""
    b = BUSHING
    flange = cq.Workplane("XY").circle(b["flange_od"] / 2).circle(b["bore"] / 2).extrude(b["flange_t"])
    body = cq.Workplane("XY").circle(b["od"] / 2).circle(b["bore"] / 2).extrude(-(b["length"] - b["flange_t"]))
    return flange.union(body)


def ball_stud() -> cq.Workplane:
    """Ball centre at the origin; stud along +x (through the plate tab)."""
    ball = cq.Workplane("XY").sphere(BALL_D / 2)
    neck = cq.Workplane("YZ").circle(1.6).extrude(BALL_D / 2 + 0.5)
    hexa = cq.Workplane("YZ").workplane(offset=BALL_D / 2 + 0.5).polygon(6, STUD_HEX / 0.866).extrude(STUD_STANDOFF)
    thread = (cq.Workplane("YZ").workplane(offset=BALL_D / 2 + 0.5 + STUD_STANDOFF)
              .circle(STUD_THREAD[0] / 2).extrude(STUD_THREAD[1]))
    return ball.union(neck).union(hexa).union(thread)


def ball_socket() -> cq.Workplane:
    """Nylon socket: ball centre at the origin, shank hanging along -z."""
    w, t = SOCKET_BODY
    body = cq.Workplane("XZ").circle(w / 2).extrude(t / 2, both=True)
    shank = cq.Workplane("XY").circle(SOCKET_SHANK[0] / 2).extrude(-SOCKET_SHANK[1])
    return body.union(shank)


def stud_m3(length=16.0) -> cq.Workplane:
    """M3 grub screw used as the rod's threaded stud, with a jam nut at its base.

    Origin at the rod's top face: 5 mm runs down into the M3 insert, the rest sticks up.
    """
    stud = cq.Workplane("XY").workplane(offset=-5.0).circle(1.5).extrude(length)
    nut = cq.Workplane("XY").polygon(6, 5.5 / 0.866).circle(1.5).extrude(2.4)
    return stud.union(nut)


PARTS = {
    "button": button, "button_ring": button_ring,
    "ga37": ga37, "hub": hub, "shoulder_bolt": shoulder_bolt, "bushing": bushing,
    "ball_stud": ball_stud, "ball_socket": ball_socket, "stud_m3": stud_m3,
}
