"""Interactive MuJoCo viewer running the full controller in real time.

    uv run mjpython -m sim.view        (macOS needs mjpython)

Keys (chosen to avoid MuJoCo's own single-letter shortcuts):
  Page Down (Fn+Down on a Mac) = next mode
  Home      (Fn+Left)          = reset ball
  End       (Fn+Right)         = park: motors off, plate drops
  2 = show/hide the base (frame, shelf)   3 = noise liner   4 = Voronoi shell (hidden at start)
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from . import modes
from .sim import Sim

KEY_M, KEY_R, KEY_P = 267, 268, 269  # GLFW: PAGE_DOWN, HOME, END


def main():
    state = {"mode": 0, "reset": False, "park": False}

    def on_key(key):
        if key == KEY_M:
            state["mode"] = (state["mode"] + 1) % len(modes.ALL)
            state["reset"] = True
            print("Mode:", modes.ALL[state["mode"]].name, flush=True)
        elif key == KEY_R:
            state["reset"] = True
        elif key == KEY_P:
            state["park"] = not state["park"]
            print("Parked" if state["park"] else "Running", flush=True)

    print("Page Down (Fn+Down): next mode   Home (Fn+Left): reset   End (Fn+Right): park   "
          "2: base   3: noise liner   4: Voronoi shell", flush=True)
    sim = Sim(modes.ALL[0]())
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as v:
        v.cam.lookat[:] = (0, 0, 0.12)
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.55, -30, 120
        while v.is_running():
            if state["reset"]:
                with v.lock():
                    sim.reset(modes.ALL[state["mode"]]())
                state["reset"] = False
            start = time.perf_counter()
            with v.lock():
                if state["park"]:
                    sim.data.ctrl[:] = 0
                    for _ in range(sim.substeps):
                        mujoco.mj_step(sim.model, sim.data)
                    r = None
                else:
                    r = sim.step()
                # Draw the setpoint as a small marker on the plate.
                v.user_scn.ngeom = 0
                if r is not None:
                    rot = sim.data.xmat[sim.plate_id].reshape(3, 3)
                    top = sim.data.site("plate_origin").xpos
                    p = top + rot @ np.array([*r["setpoint"], 0.001])
                    mujoco.mjv_initGeom(v.user_scn.geoms[0], mujoco.mjtGeom.mjGEOM_CYLINDER,
                                        np.array([0.006, 0.0006, 0]), p, rot.flatten(),
                                        np.array([0.9, 0.3, 0.1, 1], dtype=np.float32))
                    v.user_scn.geoms[0].label = modes.ALL[state["mode"]].name
                    v.user_scn.ngeom = 1
                # Don't call v.set_texts() every frame: on mujoco 3.15.0 / macOS it hangs
                # the viewer and leaks memory without bound (~100 GB in minutes).
            v.sync()
            time.sleep(max(0, 1 / 200 - (time.perf_counter() - start)))


if __name__ == "__main__":
    main()
