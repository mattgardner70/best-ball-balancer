"""Per-motor output-shaft torque in every mode (and a couple of disturbances).

    uv run python -m sim.torque [seconds]
"""

import sys

import numpy as np

from . import modes
from .params import MOTOR
from .sim import Sim

KGCM = 0.0980665  # N*m per kg*cm


def measure(mode_cls, seconds, ball_start=(0.03, -0.02), kick=None):
    sim = Sim(mode_cls(), ball_start=ball_start)
    if kick is not None:
        dof = sim.model.jnt_dofadr[sim.model.joint("ball").id]
        sim.data.qvel[dof:dof + 2] = kick
    log = [sim.step() for _ in range(int(seconds * 200))]
    keep = [r for r in log if r["t"] > (0.0 if kick is not None else 1.0)]
    tau = np.array([r["torque"] for r in keep]) / KGCM          # kg*cm, per motor
    return tau


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    stall = MOTOR.stall_torque / KGCM
    limit = MOTOR.driver_current_limit * MOTOR.torque_per_amp / KGCM
    print(f"Output-shaft torque per motor, kg*cm ({seconds:.0f} s per mode). "
          f"GA37 stall {stall:.1f}; TB6612 continuous limit about {limit:.2f}.\n")
    print(f"{'case':<28}{'motor':>6}{'peak':>8}{'RMS':>8}{'mean':>8}{'% of limit (RMS)':>18}")
    rows = [(cls.name, cls, None, (0.03, -0.02)) for cls in modes.ALL]
    rows += [("Drop at corner (recovery)", modes.CenterHold, (0.0, 0.0), (0.07, 0.045)),
             ("Rolled in at 0.25 m/s", modes.CenterHold, (0.25, 0.0), (-0.07, 0.0))]
    for name, cls, kick, start in rows:
        tau = measure(cls, seconds if kick is None else 4, start, kick)
        for k in range(3):
            t = tau[:, k]
            print(f"{name if k == 0 else '':<28}{k + 1:>6}{np.abs(t).max():>8.2f}{np.sqrt((t**2).mean()):>8.2f}"
                  f"{t.mean():>8.2f}{100 * np.sqrt((t**2).mean()) / limit:>17.0f}%")
        print()


if __name__ == "__main__":
    main()
