"""Builds the MuJoCo model (MJCF) of the 3-RRS ball balancer from params."""

from pathlib import Path

import numpy as np

from .kinematics import leg_directions
from .params import BALL, GEOMETRY, MOTOR, PLATE

PANEL = (0.187, 0.141)  # 8.4" resistive panel outline
MESHES = Path(__file__).resolve().parent / "meshes"   # written by `python -m CAD.build`
# Where each CAD part sits in its MuJoCo body frame (metres / degrees).
HOLDER_Z = 0.013          # CAD origin (holder underside) above the rod-end centres


def _f(*v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def build_xml() -> str:
    g, p, b, m = GEOMETRY, PLATE, BALL, MOTOR
    h0 = g.home_plate_height
    u = leg_directions(g)
    plate_top = p.joint_drop + p.thickness
    ball_z = h0 + plate_top + b.radius + 0.001

    have = lambda n: (MESHES / f"{n}.stl").exists()
    real = have("arm") and have("rod")          # CAD meshes present: hide the stick shapes
    hide = 'rgba="0 0 0 0"' if real else ""
    mm = lambda *v: _f(*(x / 1000 for x in v))
    mesh = lambda name, pos, extra="", cls="moving", rgba="": (
        f'<geom type="mesh" mesh="{name}" pos="{pos}" {extra} class="{cls}" mass="0" {rgba}/>' if have(name) else "")
    steel = 'rgba="0.55 0.57 0.6 1"'
    bronze = 'rgba="0.72 0.5 0.25 1"'
    nylon = 'rgba="0.15 0.15 0.15 1"'  # tie-rod-end socket
    alu = 'rgba="0.78 0.79 0.8 1"'
    assets = "".join(f'<mesh name="{n}" file="{n}.stl" scale="0.001 0.001 0.001"/>'
                     for n in ("arm", "rod", "plate_rim", "plate_insert", "lattice_shell", "frame", "liner", "ga37", "hub",
                               "shoulder_bolt", "bushing", "ball_stud", "ball_socket", "stud_m3",
                               "base_plate", "strap_gearbox", "strap_motor", "mega_mount", "driver_mount", "switch_mount", "button", "button_ring") if have(n))
    # Stack along each shaft, from the rod's plane (y = 0 in the arm frame), in mm.
    from CAD.chassis import MOTOR_FACE
    from CAD.parts import ARM_NEAR, ARM_T, EYE_HALF, SHOULDER_END
    eye_half, shoulder_end, arm_near, arm_t, flange_t = EYE_HALF, SHOULDER_END, ARM_NEAR, ARM_T, 2.0
    hub_y = -(arm_near + arm_t + flange_t)         # coupling flange's outer face; collar runs back into the arm
    motor_face = MOTOR_FACE
    legs, motors_vis, connects, actuators, studs = [], [], [], [], []
    chassis = _chassis_geoms(have)
    for i, ang in enumerate(g.leg_angles_deg, start=1):
        ux, uy = u[i - 1]
        sx, sy = g.motor_radius * ux, g.motor_radius * uy
        tx, ty = -uy, ux
        if have("ga37"):
            mx, my = sx + motor_face / 1000 * tx, sy + motor_face / 1000 * ty
            motors_vis.append(f'<geom type="mesh" mesh="ga37" pos="{_f(mx, my, g.motor_height)}" '
                              f'xyaxes="{_f(-ux, -uy, 0, 0, 0, 1)}" class="fixed" rgba="0.35 0.37 0.4 1"/>')
        else:
            motors_vis.append(
                f'<geom type="cylinder" size="0.0185 0.032" class="fixed" '
                f'fromto="{_f(sx - 0.026 * ux, sy - 0.026 * uy, g.motor_height, sx - 0.026 * ux + 0.064 * tx, sy - 0.026 * uy + 0.064 * ty, g.motor_height)}"/>'
            )
        L, R = g.arm_length * 1000, g.rod_length * 1000
        legs.append(f"""
    <body name="arm{i}" pos="{_f(sx, sy, g.motor_height)}" euler="0 0 {ang:.6g}">
      <joint name="arm{i}" type="hinge" axis="0 -1 0" range="{np.degrees(g.arm_min):.4g} {np.degrees(g.arm_max):.4g}"
             limited="true" armature="3e-4" frictionloss="0.01" damping="0.0005"/>
      <geom type="capsule" size="0.005" fromto="0 0 0 {g.arm_length:.6g} 0 0" class="moving" mass="0.012" {hide}/>
      <geom type="cylinder" size="0.008 0.006" euler="90 0 0" class="moving" mass="0.004" {hide}/>
      {mesh("arm", mm(0, -arm_near, 0), 'euler="90 0 0"')}
      {mesh("hub", mm(0, hub_y, 0), 'euler="-90 0 0"', rgba=steel)}
      {mesh("shoulder_bolt", mm(L, shoulder_end + 0.1, 0), 'euler="-90 0 0"', rgba=steel)}
      <body name="rod{i}" pos="{g.arm_length:.6g} 0 0">
        <joint name="elbow{i}" type="hinge" axis="0 -1 0" damping="0.0002"/>
        <geom type="capsule" size="0.003" fromto="0 0 0 0 0 {g.rod_length:.6g}" class="moving" mass="0.010" {hide}/>
        {mesh("rod", "0 0 0")}
        {mesh("bushing", mm(0, eye_half, 0), 'euler="-90 0 0"', rgba=bronze)}
        {mesh("stud_m3", mm(0, 0, 33.0), rgba=steel)}
        {mesh("ball_socket", mm(0, 0, R), rgba=nylon)}
      </body>
    </body>""")
        studs.append(mesh("ball_stud", mm(g.plate_joint_radius * 1000 * ux, g.plate_joint_radius * 1000 * uy, 0),
                          f'euler="0 0 {ang + 90:.6g}"', rgba=steel))
        connects.append(
            f'<connect name="rodend{i}" body1="rod{i}" body2="plate" anchor="0 0 {g.rod_length:.6g}" solref="0.002 1"/>'
        )
        actuators.append(
            f'<general name="motor{i}" joint="arm{i}" ctrlrange="{-m.supply_v} {m.supply_v}" ctrllimited="true" '
            f'gainprm="{m.torque_per_volt:.6g}" biastype="affine" biasprm="0 0 {-m.back_emf_damping:.6g}"/>'
        )

    lip_h, lip_t = 0.00635, 0.004  # 0.25" lip above the panel
    lx, ly = PANEL[0] / 2 + lip_t / 2, PANEL[1] / 2 + lip_t / 2
    lz = plate_top + lip_h / 2
    lips = [
        f'<geom type="box" class="plate" size="{_f(lip_t / 2, ly + lip_t / 2, lip_h / 2)}" pos="{_f(lx, 0, lz)}"/>',
        f'<geom type="box" class="plate" size="{_f(lip_t / 2, ly + lip_t / 2, lip_h / 2)}" pos="{_f(-lx, 0, lz)}"/>',
        f'<geom type="box" class="plate" size="{_f(lx + lip_t / 2, lip_t / 2, lip_h / 2)}" pos="{_f(0, ly, lz)}"/>',
        f'<geom type="box" class="plate" size="{_f(lx + lip_t / 2, lip_t / 2, lip_h / 2)}" pos="{_f(0, -ly, lz)}"/>',
    ]

    return f"""<mujoco model="ball_balancer_3rrs">
  <compiler angle="degree" autolimits="true" meshdir="{MESHES}"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <default>
    <default class="fixed"><geom contype="0" conaffinity="0" rgba="0.62 0.66 0.63 1" density="0"/></default>
    <default class="moving"><geom contype="0" conaffinity="0" rgba="0.25 0.42 0.85 1"/></default>
    <default class="plate"><geom contype="2" conaffinity="0" rgba="{'0 0 0 0' if have('plate_rim') else '0.78 0.84 0.95 1'}" density="0" friction="0.6 0.005 0.0001"/></default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.88" rgb2="0.82 0.83 0.8" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
    <material name="led" emission="0.8"/>
    {assets}
  </asset>
  <worldbody>
    <light pos="0.3 -0.4 0.8" dir="-0.3 0.4 -0.8" diffuse="0.8 0.8 0.8"/>
    <light pos="-0.3 0.3 0.6" dir="0.3 -0.3 -0.6" diffuse="0.3 0.3 0.3"/>
    <geom name="desk" type="plane" size="0.5 0.5 0.01" material="grid" contype="1" conaffinity="1"/>
    {'<geom type="mesh" mesh="lattice_shell" class="fixed" group="4" rgba="0.08 0.08 0.09 1"/>' if have("lattice_shell") else ''}
    {'<geom type="mesh" mesh="liner" class="fixed" group="3" rgba="0.95 0.95 0.93 1"/>' if have("liner") else ''}
    {'<geom type="mesh" mesh="frame" class="fixed" group="2" rgba="0.95 0.95 0.93 1"/>' if have("frame") else ''}
    {''.join(motors_vis)}
    {chassis}
    {''.join(legs)}
    <body name="plate" pos="0 0 {h0:.6g}">
      <freejoint name="plate"/>
      <inertial pos="0 0 {p.joint_drop + p.thickness / 2:.6g}" mass="{p.mass}"
                diaginertia="{_f(p.mass * p.size_y**2 / 12, p.mass * p.size_x**2 / 12, p.mass * (p.size_x**2 + p.size_y**2) / 12)}"/>
      <geom name="plate" type="box" class="plate" size="{_f(p.size_x / 2, p.size_y / 2, p.thickness / 2)}"
            pos="0 0 {p.joint_drop + p.thickness / 2:.6g}"/>
      <geom type="box" size="{_f(PANEL[0] / 2, PANEL[1] / 2, 0.0005)}" pos="0 0 {plate_top - 0.0004:.6g}"
            contype="0" conaffinity="0" rgba="0.2 0.24 0.26 1" density="0"/>
      {"".join(f'<geom type="mesh" mesh="{n}" pos="0 0 {HOLDER_Z}" contype="0" conaffinity="0" density="0" rgba="0.93 0.93 0.91 1"/>' for n in ("plate_rim", "plate_insert") if have(n))}
      {''.join(lips)}
      {''.join(studs)}
      <site name="plate_origin" pos="0 0 {plate_top:.6g}" size="0.003"/>
    </body>
    <body name="ball" pos="0 0 {ball_z:.6g}">
      <freejoint name="ball"/>
      <geom name="ball" type="sphere" size="{b.radius}" mass="{b.mass}" contype="1" conaffinity="3"
            rgba="0.75 0.77 0.8 1" friction="0.6 0.005 0.0001"/>
    </body>
  </worldbody>
  <equality>
    {''.join(connects)}
  </equality>
  <actuator>
    {''.join(actuators)}
  </actuator>
</mujoco>
"""


def save(path: str = "sim/ball_balancer.xml") -> str:
    xml = build_xml()
    with open(path, "w") as f:
        f.write(xml)
    return path


def _chassis_geoms(have) -> str:
    """Base plate, motor straps, electronics stand-ins and the parked ball (all fixed)."""
    from CAD import chassis as ch
    from CAD import enclosure as en

    out = []
    white = 'rgba="0.95 0.95 0.93 1"'
    if have("base_plate"):
        out.append(f'<geom type="mesh" mesh="base_plate" class="fixed" {white}/>')
    for n in ("mega_mount", "driver_mount", "switch_mount"):          # bolt-on holders (world coordinates)
        if have(n):
            out.append(f'<geom type="mesh" mesh="{n}" class="fixed" rgba="0.85 0.87 0.9 1"/>')
    rs = GEOMETRY.motor_radius
    for a in GEOMETRY.leg_angles_deg:
        r = np.radians(a)
        u, t = np.array([np.cos(r), np.sin(r)]), np.array([-np.sin(r), np.cos(r)])
        for name, tc in (("strap_gearbox", ch.GEARBOX_SADDLE_T), ("strap_motor", ch.MOTOR_SADDLE_T)):
            if have(name):
                p = rs * u + tc / 1000 * t
                out.append(f'<geom type="mesh" mesh="{name}" pos="{_f(p[0], p[1], ch.BODY_Z / 1000)}" '
                           f'xyaxes="{_f(u[0], u[1], 0, t[0], t[1], 0)}" class="fixed" {white}/>')
    box = lambda c, half, rgba: f'<geom type="box" pos="{_f(*(v / 1000 for v in c))}" size="{_f(*(v / 1000 for v in half))}" class="fixed" rgba="{rgba}"/>'
    mx, my = ch.MEGA_AT
    L, H = ch.MEGA
    bx = mx + 10.0 - 6.0 - 0.8
    zb = ch.PLATE_T + ch.MEGA_LIFT
    out.append(box((bx, my, zb + H / 2), (0.8, L / 2, H / 2), "0.05 0.45 0.55 1"))                    # Mega, on edge
    hz = zb + 4 if ch.MEGA_FLIP else zb + H - 4                                                        # digital header row
    out.append(box((bx - 5, my - 10 if ch.MEGA_FLIP else my + 10, hz), (4, 35, 2.5), "0.1 0.1 0.1 1"))
    usb_y = my + L / 2 + 3 if ch.MEGA_FLIP else my - L / 2 - 3
    out.append(box((bx - 6, usb_y, zb + (H - 38.4 if ch.MEGA_FLIP else 38.4)), (5.5, 8, 6), "0.7 0.7 0.72 1"))   # USB-B
    # MDD3A drivers, back to back on the driver wall (terminals along the top edge).
    dl, dw = ch.MDD3A
    for side in (1, -1):
        x = ch.DRIVER_WALL_X + side * (1.5 + ch.DRIVER_STANDOFF + 0.8)
        z0 = ch.PLATE_T + ch.MEGA_LIFT
        out.append(box((x, ch.DRIVER_Y, z0 + dl / 2), (0.8, dw / 2, dl / 2), "0.08 0.08 0.1 1"))
        out.append(box((x + side * 6, ch.DRIVER_Y, z0 + dl - 6), (5.5, dw / 2 - 3, 5), "0.1 0.1 0.1 1"))   # terminal blocks
    px, py = ch.POWER_SWITCH_AT
    out.append(box((px + 0.8, py, ch.PLATE_T + 4 + 12.7), (0.8, 10.15, 12.7), "0.1 0.35 0.15 1"))      # the green Pololu
    bx2, by2 = ch.BUCK_AT
    out.append(box((bx2, by2, ch.PLATE_T + 7), (13.5, 23.0, 7.0), "0.2 0.2 0.22 1"))                   # 5 V buck (sealed case)
    jx, _, _, jz = en.POWER_TAB
    jb = en.SHELL_D / 2 + en.FIT + en.CRADLE_WALL      # outside face of the back wall
    out.append(f'<geom type="cylinder" fromto="{_f(jx / 1000, (jb - 20) / 1000, jz / 1000, jx / 1000, (jb + 1.5) / 1000, jz / 1000)}" size="0.0055" class="fixed" rgba="0.1 0.1 0.1 1"/>')
    if have("button"):
        # POWER (left) and MODE (right) on the console face; CAD +z points out of the panel (-y here).
        face_y = -(en.SHELL_D / 2 + en.FIT + en.CONSOLE_DEPTH + en.CRADLE_WALL)
        for k, x in enumerate((-en.BUTTON_PITCH / 2, en.BUTTON_PITCH / 2)):
            pos = _f(x / 1000, face_y / 1000, en.BUTTON_Z / 1000)
            out.append(f'<geom type="mesh" mesh="button" pos="{pos}" xyaxes="1 0 0 0 0 1" class="fixed" group="2" rgba="0.12 0.12 0.13 1"/>')
            out.append(f'<geom type="mesh" mesh="button_ring" pos="{pos}" xyaxes="1 0 0 0 0 1" class="fixed" group="2" '
                       f'rgba="0.2 0.55 1.0 1" material="led"/>')
    if have("frame"):
        # A spare 1.5" ball parked in the dish on the frame's console top (visual only).
        yc, _, depth = en.dish_geometry()
        r = BALL.radius * 1000
        bottom = en.CONSOLE_HIGH - depth
        out.append(f'<geom type="sphere" size="{r / 1000:.6g}" pos="{_f(0.03, yc / 1000, (bottom + r) / 1000)}" '
                   f'class="fixed" group="2" rgba="0.75 0.77 0.8 1"/>')
    return "\n    ".join(out)
