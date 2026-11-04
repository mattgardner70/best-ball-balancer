# Best Ball Balancer

**I set out to build the best ball-balancing plate out there:** beautiful, perfectly silent, robust enough to run for years, and full of features. Just as important, I wanted anyone to be able to build one. Every part is editable, so you can swap in different motors or drivers, and every step is documented so you can follow along and build your own.

Three geared motors tilt an 8.4" touch panel; the panel feels where the ball is; an Arduino Mega closes the loop a hundred times a second. It holds the ball dead centre, or steers it around a circle, a figure-8, a square, or a slow random wander.

![The balancer, shell on](images/hero.png)

| | |
|---|---|
| **Build** | [Assembly guide](assembly.md) (animated, step by step) · [Bill of materials](BOM/BOM.md) · [Wiring guide](wiring/README.md) with a [wire-by-wire video](https://github.com/user-attachments/assets/843bd77f-8e50-462b-997f-23ea67aadc6e) |
| **Files** | [3D-printable parts](stl/) · [CAD source](CAD/) · [Firmware](code/firmware/balance/balance.ino) · [Simulation](code/sim/) |
| **Tune** | [Tuning guide](tuning.md): PID, centre and paths over the console, with a live top-down visualizer |
| **Power** | 12–20 W from a 12 V wall adapter |
| **Time** | About 40 hours, most of it printing ([breakdown](#time-estimate)) |

---

## Designed to be silent

Most ball balancers use hobby servos or steppers, and you can hear them from across the room: servos buzz while holding position, and steppers whine. This one is designed for a desk you sit at:

- **Geared DC motors** (GA37-520, 30:1) instead of servos or steppers. A gearmotor holding still is quiet.
- **20 kHz PWM**, above the range of human hearing, so the motor drivers don't sing.
- **Encoders on every motor**, so each arm moves smoothly to an exact angle instead of hunting.
- **A damped enclosure:** a Voronoi lattice shell with an optional solid liner that hides the mechanism and soaks up what noise is left.
- **A soft power button** (Pololu pushbutton switch) instead of a clicking toggle: a press lowers the arms slowly onto their stops before the power goes off.

## How it works

![Inside: three motors, three arms, one plate](images/mechanism.png)

The plate rides on three legs spaced 120° apart: a **3-RRS parallel mechanism**. Each leg is a motor-driven arm, a rod, and a ball joint under the plate. Raising and lowering the three arms tilts the plate in any direction.

Two control loops run on the Mega:

| Loop | Rate | Job |
|---|---|---|
| **Ball loop** | ~70 Hz | Read the ball's position from the resistive panel, filter it, and decide how far to tilt the plate (PID + feed-forward for moving targets). |
| **Motor loop** | 500 Hz | Turn that tilt into three arm angles (inverse kinematics), and drive each motor to its angle using its encoder. |

![One leg: motor, arm, rod and ball joint](images/leg.png)

## Modes

The balancer doesn't just hold the ball still: it can steer it along a path. Each mode moves a target point around the plate, and the controller makes the ball follow it. A feed-forward term tilts the plate for each turn slightly before it's due, so the ball tracks the path instead of lagging behind it.

| Mode | What the ball does |
|---|---|
| **Centre** | Sits still in the middle of the plate. |
| **Circle** *(default)* | Rolls a 90 mm circle, one lap every 3.5 s. |
| **Figure-8** | Traces a figure-8, 120 mm wide and 47 mm tall, one loop every 5.5 s. |
| **Square** | Dashes between the corners of a 90 mm square, holding 1.5 s at each corner. |
| **Wander** | Drifts slowly around the plate on a smooth, random-looking path. |

It powers up in **circle**, and starts the moment a ball touches the panel. Each new mode glides out from the centre over 2 seconds rather than jumping.

**Switching modes:** press the blue MODE button to step to the next mode, or type `mode figure-8` (or `n`) in the [console](code/tools/console.py).

**The ball doesn't follow a path exactly.** Left alone it cuts inside: the circle comes out smaller and slightly triangular, and the figure-8's loops pull in. Each pattern's target therefore runs a hand-tuned path, reshaped where the ball cuts in, so that the ball traces the shape you want. See the [tuning guide](tuning.md).

## Tuning

Everything is tuned live over USB while the ball rolls, and kept with `save`: the PID gains, where the plate's centre is, the leg trims, and the shape of each pattern's path. A live top-down visualizer shows the ball, its target and the goal shape. The [tuning guide](tuning.md) walks through it.

![Figure-8 with a tuned path: the target (black) runs a reshaped path so that the ball (blue) traces the goal (orange)](images/tuning/figure-8-after.png)

## The story

### 1. Simulate first

Before buying a single part, the whole machine existed in [MuJoCo](https://mujoco.org/): plate, legs, motors, ball and controller. The simulation picked the geometry (arm length, rod length, leg radius), sized the motors (holding torque, worst-case current), and tuned the first controller. The CAD is generated from the same parameters, so the model and the parts can't drift apart.

### 2. Design every part in code

Every printed part is a [CadQuery](https://cadquery.readthedocs.io/) script. Change a dimension, rerun the build, and every STL, preview and simulation mesh updates together. The parts were iterated many times this way: a two-piece plate (rim and insert) that prints in about an hour instead of eight, spring-preloaded motor straps, bolt-on holders for every board so each can be wired on the bench, and a raised Mega with room for its plugs underneath.

### 3. Bring it up, one piece at a time

Small test sketches proved each subsystem on its own before anything ran together: [encoders](code/firmware/bringup/encoder_monitor/), [motor drive](code/firmware/bringup/drive_test/), [touch panel](code/firmware/bringup/touch_test/), and [homing](code/firmware/bringup/jog_test/).

### 4. Integration hell

Things always go wrong. Some highlights:

- **A dead motor driver.** 12 V went into the 5 V pin of the original TB6612 driver. It was replaced by the sturdier Cytron MDD3A.
- **Motors that went the wrong way, sometimes.** `799 * 45` overflows a 16-bit integer on the Arduino, so "45% counter-clockwise" quietly became "37% clockwise". One `(long)` cast fixed hours of confusion.
- **A homing routine that lied.** The three arms are linked through the plate, so one arm could stop against the plate's weight and look "home" while another was still high. Homing now finishes with a joint push-down.
- **A loose encoder wire.** It came loose twice and sent an arm to full extension. It's glued now, and the firmware rests a motor whose encoder goes silent under full power.
- **"It only shakes when USB is unplugged."** A flight recorder in EEPROM, read back after each unplugged run, ruled out resets, phantom touches and serial noise one by one, then caught the real bug: a "full power" mode meant only for the start-up raise was still switched on while balancing. Fixed, it runs the same with or without USB.
- **A board that froze.** With the encoders' 5 V off, their inputs picked up noise and fired interrupts so fast the Mega locked up mid-start. The firmware now spots an encoder flooding, switches that motor off and says so in the log, and checks every pin at boot.
- **The jitter hunt.** A 100 Hz data stream showed three culprits: single-frame glitches from the touch panel (now rejected), a loop that ran 50% slower than it assumed (now measured), and EEPROM writes stalling the motors once a second (removed).

### 5. Tune it

Gains were tuned live over serial, then saved. Leg trims level the plate, a feed-forward term with a small lead lets the ball follow moving targets, and a rotation trim cancels sideways drift. The result holds the ball within a few millimetres of centre.

Moving patterns were harder: however the gains were set, the ball cut inside the path. A learning controller (it corrects the target lap by lap) fixed the shape but made the plate loud. Hand-tuned paths, drawn and corrected against the live visualizer, did it quietly: the figure-8 now runs 3 mm off its goal on average.

![The finished build](images/built.jpg)

---

## Requirements

**Hardware:** everything is in the [bill of materials](BOM/BOM.md): about $306 of parts including the three GA37-520 gearmotors and two Cytron MDD3A drivers, and about 650 g of PLA/PETG.

**Tools:** a 3D printer (every part fits a 256 mm bed), a soldering iron (about 10 joints), a heat-set insert tip, a multimeter and hex keys.

**Software:**

| For | You need |
|---|---|
| Firmware | [Arduino IDE](https://www.arduino.cc/en/software) or [arduino-cli](https://arduino.github.io/arduino-cli/), board **Arduino Mega 2560** |
| CAD and simulation (optional) | Python 3.12 and [uv](https://docs.astral.sh/uv/) |

## Quick start

**Flash the firmware** (recommended: arduino-cli, which allows a bigger serial buffer so status printing never stalls the motors):

```sh
arduino-cli compile --fqbn arduino:avr:mega \
  --build-property "build.extra_flags=-DSERIAL_TX_BUFFER_SIZE=256" code/firmware/balance
arduino-cli upload --fqbn arduino:avr:mega -p /dev/cu.usbmodem1101 code/firmware/balance
```

From the Arduino IDE, just open [`code/firmware/balance/balance.ino`](code/firmware/balance/balance.ino) and upload; it works without the bigger buffer.

**Power it on.** It homes the arms, raises the plate level, and starts balancing (circle mode) the moment a ball touches the panel.

**Watch the simulation:**

```sh
uv sync
cd code && uv run mjpython -m sim.view      # macOS needs mjpython; elsewhere: python
```

**Rebuild every printed part** after changing a dimension in [`CAD/`](CAD/):

```sh
uv run python -m CAD.build
```

## Time estimate

| Stage | Hours | |
|---|---|---|
| 3D printing | **30** | Mostly unattended. The base plate, frame and shell are the long prints. Note: The bulk of the time is mostly the shells and covers. The base + plate + arms + board mounts is ~8 hours of printing time. |
| Wiring | **5** | Lever connectors and Dupont leads; about 10 solder joints in total. Follow the [wiring video](https://github.com/user-attachments/assets/843bd77f-8e50-462b-997f-23ea67aadc6e) and the [wiring PDF](wiring/wiring.pdf). |
| Integration hell | **5** | Things always go wrong: a loose wire, a reversed motor, a calibration that needs redoing. Budget for it. |
| **Total** | **~40** | |

## What's in this repo

| Folder | Contents |
|---|---|
| [`stl/`](stl/) | Every printed part, ready to slice |
| [`CAD/`](CAD/) | CadQuery source for every part ([`build.py`](CAD/build.py) exports them all) |
| [`code/firmware/`](code/firmware/) | The balancer firmware, plus four bring-up test sketches |
| [`code/sim/`](code/sim/) | MuJoCo model, controller and interactive viewer |
| [`code/tools/`](code/tools/) | The serial console, the live visualizer and the path tool for [tuning](tuning.md), and the scripts that render the assembly animations and the wiring video |
| [`BOM/`](BOM/BOM.md) | Bill of materials with links |
| [`wiring/`](wiring/README.md) | Wiring guide, and the diagram and wire list (PDF) |
| [`assembly.md`](assembly.md) | Step-by-step build guide, with an animation for each stage |
