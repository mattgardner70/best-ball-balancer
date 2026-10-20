# Assembly guide

Seven stages, from a pile of parts to a balancing ball. Keep the [bill of materials](BOM/BOM.md) and the [wiring diagram](wiring/wiring.pdf) open alongside.

| Stage | What | Time |
|---|---|---|
| [1](#1-print-the-parts) | Print the parts | ~30 h (unattended) |
| [2](#2-set-the-heat-set-inserts) | Set the heat-set inserts | 20 min |
| [3](#3-build-the-three-legs) | Build the three legs | 45 min |
| [4](#4-mount-the-motors) | Mount the motors | 30 min |
| [5](#5-mount-the-electronics) | Mount the electronics | 30 min |
| [6](#6-wire-it) | Wire it | ~5 h |
| [7](#7-bring-it-to-life) | Flash, calibrate, tune | 1–5 h (integration hell) |

---

## 1. Print the parts

All files are in [`stl/`](stl/). PLA is fine everywhere; PETG is better for the arms and rods.

| Part | Qty | Orientation |
|---|---|---|
| [`base_plate`](stl/base_plate.stl) | 1 | Flat, as modelled |
| [`mega_mount`](stl/mega_mount.stl) | 1 | Lying on its back, standoffs up |
| [`driver_mount`](stl/driver_mount.stl) | 1 | Standing up, on its tabs |
| [`switch_mount`](stl/switch_mount.stl) | 1 | Standing up, on its tabs |
| [`strap_motor`](stl/strap_motor.stl), [`strap_gearbox`](stl/strap_gearbox.stl) | 3 each | Arch up |
| [`arm`](stl/arm.stl) | 3 | Flat |
| [`rod`](stl/rod.stl) | 3 | Standing up |
| [`plate_holder_base`](stl/plate_holder_base.stl) | 1 | Upside down, tabs up |
| [`plate_holder_rim`](stl/plate_holder_rim.stl) | 1 | Flat |
| [`frame`](stl/frame.stl) | 1 | Open side down, supports under the console top |
| [`lattice_shell`](stl/lattice_shell.stl) | 1 | Top face down |
| [`liner`](stl/liner.stl) (optional) | 1 | Top face down |

## 2. Set the heat-set inserts

Press **36 M3 inserts** with a soldering iron and insert tip, flush with the surface:

- **Base plate:** 12 in the motor saddles (two beside each strap) and 12 in the raised pads the board holders screw onto.
- **Mega holder:** 6 in its standoffs.
- **Arms:** 1 in each elbow tip.
- **Rods:** 1 in each top face.

The MDD3A standoffs on the driver holder don't take inserts: their M2.5 screws tap straight into the plastic.

## 3. Build the three legs

![One leg](images/leg.png)

For each leg:

1. **Coupling onto the arm.** Seat the flange coupling's collar in the arm's pocket and bolt the flange on with four **M3 × 12** screws.
2. **Bushing into the rod.** Press a flanged bronze bushing into the rod's elbow eye, from the side the bolt head will be on.
3. **Elbow.** Pass a **shoulder bolt** through the bushing and thread it into the arm's elbow insert. The rod should swing freely with no play.
4. **Stud.** Thread an **M3 × 16 grub screw** into the rod's top insert, and lock it with an **M3 nut** against the rod face.
5. **Ball joint.** Screw a **tie-rod end** socket onto the stud. You'll set its height when the plate goes on.

## 4. Mount the motors

1. Lay each motor in its two saddles, gearbox face against the lip, so the shaft points along its leg. Each motor's number (**1**, **2**, **3**) is moulded into the floor under it.
2. Fit the **gearbox strap** and the **motor strap**, each with two **M3 × 12** screws.
3. Tighten until the strap's ears touch the saddle. The straps are deliberately flat: when they're snug on the motor, the ears still sit about 2.5 mm high, so tightening springs them down and clamps the motor firmly. A motor that can shift in its saddle makes the plate imprecise. The screws sit far enough from the arch to get a wrench on them.
4. Slide each arm's coupling onto its motor's D-shaft and tighten both set screws onto the flat.

> **Tip:** draw a line across the shaft, coupling and arm with a marker. If an arm ever slips on its shaft, you'll see it immediately.

## 5. Mount the electronics

![The driver wall beside the Mega](images/driver_wall.png)

Every board has its own printed holder, so you can mount and wire each board on the bench, then screw the holder down with **M3 × 8** screws into the pads on the base plate.

1. **Arduino Mega:** screw it to its holder with six **M3 × 6** screws, USB end toward the back. It stands 9 mm off the floor so the plugs on its bottom header have room. Use a **right-angle USB-B cable**: there isn't room for a straight plug with the shell on.
2. **Motor drivers:** stick **4 heatsinks** on each MDD3A, one on each of the four small 8-pin chips beside the terminal blocks (the tape comes on them). Then screw the boards back to back onto the driver holder, one on each face, with four **M2.5 × 8** screws each: terminal blocks up, input header down.
3. **Power switch:** stick the **Pololu** board onto its holder plate with double-sided foam tape, terminals facing out, and loop a zip tie through the two slots and across the board. It sits by the left wall, facing the centre, close to the power jack.
4. **Buck converter:** drop it into its tray and strap it down with a zip tie through the two slots.
5. **Plate holder:** glue the rim onto the plate-holder base, then drop the touch panel in.
6. **Buttons and jack:** fit the two buttons in the console face (red POWER on the left, blue MODE on the right) and the barrel jack in the frame's back tab.

## 6. Wire it

Follow the [wiring diagram](wiring/wiring.pdf). It shows every connection on one page per subsystem, with a wire list by part at the end.

- **Power and ground** run through lever connectors, one bus each for +12 V, +5 V and ground.
- **Signals** are Dupont leads straight onto the Mega's pins.
- **Soldering** is only needed on the Pololu power switch (its two terminal blocks and a few pins).

Before connecting the motors, power it once and check with a meter: 12 V on the +12 V bus and 5.0 V on the +5 V bus.

> **Never** connect 12 V to a driver's or the Mega's 5 V pin. That's how the first motor driver died.

## 7. Bring it to life

### Flash the firmware

See [Quick start](README.md#quick-start). Optional first: the four [bring-up sketches](code/firmware/bringup/) test the encoders, motor directions, touch panel and homing one at a time.

### First power-up

Put the plate holder on the three ball joints and power on. The balancer will:

1. **Home:** each arm creeps down until it stops against the base, then all three push down together to seat the plate.
2. **Raise:** the arms lift the plate to level.
3. **Balance:** the moment a ball touches the panel, it starts steering the ball (circle mode by default).

### Talk to it

Connect over USB at 115200 baud. The easiest way is the console: it finds the Mega, unlocks the keys, shows the live status and logs it to `console.log`. Type a key and press Return.

```sh
uv run python code/tools/console.py
```

In the Arduino Serial Monitor instead, set **No line ending** and send `~balancer-hello~` once to unlock the keys.

| Key | Does |
|---|---|
| `n` | Next pattern: centre, circle, figure-8, square, wander |
| `P` `p` / `I` `i` / `D` `d` | Raise / lower each gain by 15% (`v` shows them) |
| `1` `2` `3` / `!` `@` `#` | Raise / lower one leg's trim by 0.5° |
| `a` `f` / `w` `z` | Move the target centre ±3 mm (toward / away from motor 1, toward motor 2 / motor 3) |
| `l` / `b` / `s` | Level the plate / balance / stop all motors |
| `home` | Home again (in the Serial Monitor: send a newline) |

### Calibrate

1. **Panel axes:** rest the ball (by its own weight, not held down) over motor 1 and send `k1`; repeat for `k2` and `k3`. This sets which way is which and where the centre is.
2. **Level:** with the ball parked at the centre, watch the tilt in the status line. If the plate has to lean to hold it there, trim the legs with `1` `2` `3` until it doesn't.
3. **Gains:** start from the saved values. If it overshoots, raise D; if it's lazy at the edges, raise P; if it settles off-centre, raise I a little.

Once you like a setting, write it into [`balance.ino`](code/firmware/balance/balance.ino) and upload: live changes last until power-off.

### When things go wrong

| Symptom | Likely cause |
|---|---|
| An arm runs to full extension | Loose encoder wire: reseat it, then glue it |
| An arm goes the wrong way | Motor leads swapped on the driver (M+ / M−) |
| The plate leans with the ball at centre | Leg trims (`1` `2` `3`), or an arm slipping on its shaft |
| Jitter near the edges | Ball is beyond the panel's active area; keep patterns inside it |
| The ball "isn't seen" | Panel tail wires: A0–A3, see the wiring diagram |
| A motor driver runs hot | Check its heatsinks are on; run without the liner for more airflow |
