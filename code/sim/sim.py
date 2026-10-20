"""Simulation harness: model + controller stepping, shared by run.py and view.py."""

import mujoco
import numpy as np

from .controller import Controller
from .model import build_xml
from .params import CONTROL, MOTOR


class Sim:
    def __init__(self, mode, ball_start=(0.03, -0.02), seed=0):
        self.model = mujoco.MjModel.from_xml_string(build_xml())
        self.data = mujoco.MjData(self.model)
        self.substeps = round(1 / CONTROL.rate_hz / self.model.opt.timestep)
        m, d = self.model, self.data
        self.arm_q = [m.jnt_qposadr[m.joint(f"arm{i}").id] for i in (1, 2, 3)]
        self.arm_v = [m.jnt_dofadr[m.joint(f"arm{i}").id] for i in (1, 2, 3)]
        self.plate_id = m.body("plate").id
        self.ball_id = m.body("ball").id
        self.ball_q = m.jnt_qposadr[m.joint("ball").id]
        self.reset(mode, ball_start, seed)

    def reset(self, mode, ball_start=(0.03, -0.02), seed=0):
        """Back to home with a fresh controller; the model is kept."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.ball_q:self.ball_q + 2] = ball_start
        mujoco.mj_forward(self.model, self.data)
        self.ctrl = Controller(seed=seed)
        self.mode = mode
        self.target = np.zeros(3)

    def ball_on_plate(self) -> np.ndarray:
        """Ball centre in plate coordinates (what the panel measures)."""
        d = self.data
        rot = d.xmat[self.plate_id].reshape(3, 3)
        rel = d.xpos[self.ball_id] - d.xpos[self.plate_id]
        return (rot.T @ rel)[:2]

    def plate_tilt(self) -> np.ndarray:
        rot = self.data.xmat[self.plate_id].reshape(3, 3)
        n = rot[:, 2]
        return np.array([np.arctan2(-n[1], n[2]), np.arctan2(n[0], n[2])])

    def step(self):
        """One control period. Returns a dict of measurements."""
        d = self.data
        ball = self.ball_on_plate()
        setpoint = self.mode(d.time, ball)
        volts, self.target = self.ctrl.step(ball, setpoint, d.qpos[self.arm_q])
        d.ctrl[:] = volts
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, d)
        torque = d.actuator_force.copy()
        return dict(
            t=d.time, ball=ball, setpoint=setpoint, volts=volts, torque=torque,
            current=torque / MOTOR.torque_per_amp, arm=d.qpos[self.arm_q].copy(),
            arm_speed=d.qvel[self.arm_v].copy(), tilt=self.plate_tilt(),
            ball_z=d.xpos[self.ball_id][2],
        )
