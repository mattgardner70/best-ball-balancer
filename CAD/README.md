# CAD

Every printed part in this project is written as code, in [CadQuery](https://cadquery.readthedocs.io/).

![The mechanism, generated from these scripts](../images/mechanism.png)

## What is CadQuery?

CadQuery is a Python library for building 3D models. Instead of clicking and dragging in a CAD program, you describe a part in a few lines of Python:

```python
import cadquery as cq

# A 40 x 20 x 5 mm plate with a 4 mm hole in the middle and rounded corners
plate = (cq.Workplane("XY").rect(40, 20).extrude(5)
         .edges("|Z").fillet(3)
         .faces(">Z").workplane().hole(4))
```

Under the hood it uses OpenCascade, the same geometry kernel as FreeCAD, so the output is real solid geometry: exact fillets, holes and booleans, exported to STL (for printing) or STEP (for other CAD tools).

## Why we used it

**It's much faster for an AI to model in code.** This project was designed together with an AI assistant, and code-based CAD is a natural fit:

- **The AI can write and edit it directly.** A part is text, so "make the strap 1 mm flatter" or "move the Mega 4 mm toward the back" is a one-line change, not a session of clicking through a GUI.
- **Every dimension has a name and a reason.** Constants like `MEGA_LIFT = 9.0` or `STRAP_PRELOAD = 1.2` carry a comment saying why. The design explains itself, and nothing is a mystery number buried in a sketch.
- **One source of truth.** The geometry (arm length, rod length, leg spacing) comes from the same parameters the [simulation](../code/sim/) uses, so the printed parts and the physics model can never disagree.
- **Changes ripple automatically.** Change one value, rebuild, and every affected STL, preview and simulation mesh updates together. The base plate went through many revisions this way: the driver wall, the raised Mega and the preloaded straps were each a few edited lines.
- **It works with normal software tools.** Diffs, version history and code review all apply to the CAD, the same as the firmware.

## Set it up

You need **Python 3.12** and [**uv**](https://docs.astral.sh/uv/) (a fast Python package manager).

1. **Install uv** (once):

   ```sh
   curl -LsSf https://astral.sh/uv/install.sh | sh        # macOS / Linux
   # Windows: powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
   ```

2. **Install the project's dependencies** from the repo root. This pulls CadQuery, MuJoCo and the rest into a local `.venv`:

   ```sh
   uv sync
   ```

3. **Build every part:**

   ```sh
   uv run python -m CAD.build
   ```

   This writes the printable files to [`stl/`](../stl/), coarser copies for the simulation to `code/sim/meshes/`, and a preview image of each part to `CAD/previews/`. It also checks that each part fits the printer bed.

4. **Build just one part** while you iterate:

   ```sh
   uv run python -m CAD.build base_plate
   ```

5. **See it in context** in the simulation viewer:

   ```sh
   cd code && uv run mjpython -m sim.view     # macOS needs mjpython; elsewhere use python
   ```

> **Want to click around a model?** [CQ-editor](https://github.com/CadQuery/CQ-editor) is a small GUI that runs CadQuery scripts with a live 3D view, and the [OCP CAD Viewer](https://marketplace.visualstudio.com/items?itemName=bernhard-42.ocp-cad-viewer) extension does the same inside VS Code.

## Making changes

Want different motors, another motor driver, a bigger touch panel, or a different microcontroller? The easiest way to swap a component is to let an AI assistant like **Claude** or **ChatGPT** do the CAD work for you. Because every part is code, it can read the whole design, change it, and rebuild it.

1. **Open this repo with the assistant.** A coding assistant that can read and run files works best (for example Claude Code). A chat assistant works too if you paste in the relevant files.
2. **Say what you're replacing, and with what.** Give it the part's datasheet or listing, especially the dimension drawing. For example:

   > Replace the Cytron MDD3A motor drivers with two DRV8871 boards. Here's the datasheet: … Draw the new board from its dimension drawing, update the driver holder so it fits, and make sure it clears the motors and the Mega.

3. **Let it model the new part from the datasheet,** then adjust everything around it: holders, mounting holes, clearances, cable routes.
4. **Review it in the simulation.** Rebuild and look at it from every side before you print:

   ```sh
   uv run python -m CAD.build
   cd code && uv run mjpython -m sim.view
   ```

   Ask for changes in plain words ("move it 5 mm toward the back wall", "make the clip a little looser") until it's right.
5. **Print it.** The new STLs are already in [`stl/`](../stl/).

Designing with an LLM and code-based CAD is **much easier, faster and more precise** than the old-fashioned way. You describe the change in a sentence, the assistant edits a few named dimensions, and every part that depends on them updates together. You never have to redraw a sketch or hunt down every place a dimension was used. Most of this project was designed exactly this way.

> **Wiring and firmware too:** if a new part changes the pins or how it's driven (a different motor driver, for example), ask the assistant to update the [firmware](../code/firmware/balance/balance.ino) and the [wiring guide](../wiring/README.md) at the same time.

## What's in here

| File | Parts |
|---|---|
| [`chassis.py`](chassis.py) | Base plate (motor saddles, insert pads, buck tray, tie loops), motor straps, and the bolt-on holders for the Mega, the motor drivers and the power switch |
| [`parts.py`](parts.py) | Arms, rods, the plate's rim and insert |
| [`enclosure.py`](enclosure.py) | Frame with the button console, Voronoi lattice shell, liner |
| [`hardware.py`](hardware.py) | Bought parts (motors, couplings, bolts, ball joints), modelled for the simulation only |
| [`base.py`](base.py) | Shared geometry helpers |
| [`build.py`](build.py) | Exports everything to STL and renders the previews |

The key dimensions sit at the top of each file as named constants. Change one, rerun the build, and the parts follow.
