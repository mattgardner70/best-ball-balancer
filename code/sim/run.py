"""Headless run of every mode; prints the numbers the torque budget needs.

    uv run python -m sim.run [seconds]
"""

import sys

import numpy as np

from . import modes
from .params import MOTOR
from .sim import Sim


def run(mode_cls, seconds):
    sim = Sim(mode_cls())
    log = [sim.step() for _ in range(int(seconds * 200))]
    keep = [r for r in log if r["t"] > 1.0]  # skip start-up transient
    arr = lambda k: np.array([r[k] for r in keep])
    tilt = np.degrees(arr("tilt"))
    tilt_rate = np.abs(np.diff(tilt, axis=0)) * 200
    torque, current = arr("torque"), arr("current")
    err = np.linalg.norm(arr("ball") - arr("setpoint"), axis=1) * 1000
    fell = (arr("ball_z") < 0.05).any()
    return dict(
        mode=mode_cls.name,
        tilt_max=np.abs(tilt).max(),
        tilt_rate_p95=np.percentile(tilt_rate, 95),
        arm_speed_max=np.degrees(np.abs(arr("arm_speed")).max()),
        torque_peak=np.abs(torque).max() / 0.0981,
        torque_rms=np.sqrt((torque**2).mean(axis=0)).max() / 0.0981,
        current_peak=np.abs(current).max(),
        current_rms=np.sqrt((current**2).mean(axis=0)).max(),
        err_median=np.median(err),
        err_p95=np.percentile(err, 95),
        fell=fell,
    )


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    print(f"{seconds:.0f} s per mode; torque in kg*cm per arm; TB6612 limit {MOTOR.driver_current_limit} A\n")
    hdr = f"{'mode':<12}{'tilt°':>7}{'°/s p95':>9}{'arm°/s':>8}{'τ pk':>7}{'τ rms':>7}{'I pk':>7}{'I rms':>7}{'err50':>7}{'err95':>7}  ball"
    print(hdr)
    for cls in modes.ALL:
        r = run(cls, seconds)
        print(f"{r['mode']:<12}{r['tilt_max']:>7.1f}{r['tilt_rate_p95']:>9.1f}{r['arm_speed_max']:>8.0f}"
              f"{r['torque_peak']:>7.2f}{r['torque_rms']:>7.2f}{r['current_peak']:>7.2f}{r['current_rms']:>7.2f}"
              f"{r['err_median']:>6.1f}m{r['err_p95']:>6.1f}m  {'FELL OFF' if r['fell'] else 'ok'}")


if __name__ == "__main__":
    main()
