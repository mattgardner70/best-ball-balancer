"""Firmware-like cascaded controller: ball position -> tilt -> arm angle -> volts.

Runs at a fixed control rate on what the Mega actually sees: noisy panel
readings and quantized encoder counts. Smoothness matters as much as
accuracy here: every jitter in the tilt command is gear noise.
"""

import numpy as np

from .kinematics import arm_angles
from .params import CONTROL, MOTOR, Control, Motor

G_ROLL = 5 / 7 * 9.81  # solid ball rolling: a = 5/7 g sin(tilt)


class AlphaBeta:
    """Position/velocity tracker for a noisy position signal."""

    def __init__(self, dt, alpha=0.12, beta=0.004):
        self.dt, self.alpha, self.beta = dt, alpha, beta
        self.x = None
        self.v = 0.0

    def update(self, z):
        if self.x is None:
            self.x = np.array(z, dtype=float)
            self.v = np.zeros_like(self.x)
            return self.x, self.v
        pred = self.x + self.v * self.dt
        r = z - pred
        self.x = pred + self.alpha * r
        self.v = self.v + self.beta / self.dt * r
        return self.x, self.v


class Controller:
    def __init__(self, c: Control = CONTROL, m: Motor = MOTOR, seed: int = 0):
        self.c, self.m = c, m
        self.dt = 1 / c.rate_hz
        self.rng = np.random.default_rng(seed)
        self.ball = AlphaBeta(self.dt)
        self.ball_int = np.zeros(2)
        self.tilt = np.zeros(2)          # (roll, pitch) after smoothing
        self.sp_hist = []                # recent setpoints, for feedforward
        self.rad_per_count = 2 * np.pi / m.counts_per_rev
        self.prev_counts = None
        self.arm_speed = np.zeros(3)
        self.arm_int = np.zeros(3)

    # --- sensors -----------------------------------------------------------
    def read_panel(self, true_xy):
        return true_xy + self.rng.normal(0, self.c.panel_noise_std, 2)

    def read_encoders(self, true_angle):
        counts = np.round(true_angle / self.rad_per_count)
        if self.prev_counts is None:
            self.prev_counts = counts
        raw = (counts - self.prev_counts) * self.rad_per_count / self.dt
        self.prev_counts = counts
        self.arm_speed += 0.25 * (raw - self.arm_speed)
        return counts * self.rad_per_count, self.arm_speed

    # --- loops -------------------------------------------------------------
    def setpoint_motion(self, setpoint):
        """Setpoint velocity and acceleration, for feedforward."""
        self.sp_hist = (self.sp_hist + [setpoint])[-21:]
        if len(self.sp_hist) < 21:
            return np.zeros(2), np.zeros(2)
        h = np.array(self.sp_hist)
        span = 10 * self.dt  # differences over 50 ms; setpoints are noise-free
        return (h[-1] - h[-11]) / span, (h[-1] - 2 * h[-11] + h[-21]) / span**2

    def ball_loop(self, reading, setpoint):
        c = self.c
        pos, vel = self.ball.update(reading)
        err = setpoint - pos
        self.ball_int = np.clip(self.ball_int + err * self.dt, -0.03, 0.03)
        sp_vel, sp_acc = self.setpoint_motion(setpoint)
        acc = (sp_acc + c.ball_kp * err + c.ball_ki * self.ball_int
               + c.ball_kd * (sp_vel - vel))
        s = np.clip(acc / G_ROLL, -np.sin(c.max_tilt), np.sin(c.max_tilt))
        # +pitch drops the +x edge (ball goes +x); +roll lifts the +y edge (ball goes -y).
        want = np.array([-np.arcsin(s[1]), np.arcsin(s[0])])
        # Low-pass and rate-limit the tilt command: quiet gears.
        step = np.clip(0.3 * (want - self.tilt), -c.max_tilt_rate * self.dt, c.max_tilt_rate * self.dt)
        self.tilt = self.tilt + step
        return self.tilt

    def arm_loop(self, target, angle, speed):
        c, m = self.c, self.m
        err = target - angle
        self.arm_int = np.clip(self.arm_int + err * self.dt, -0.02, 0.02)
        volts = c.arm_kp * err + c.arm_ki * self.arm_int - c.arm_kd * speed
        volts = np.clip(volts, -m.supply_v, m.supply_v)
        # Firmware current cap: keep the TB6612 inside its continuous rating.
        torque = m.torque_per_volt * volts - m.back_emf_damping * speed
        current = torque / m.torque_per_amp
        over = np.abs(current) > m.driver_current_limit
        if over.any():
            cap = np.sign(current) * m.driver_current_limit * m.torque_per_amp
            volts = np.where(over, (cap + m.back_emf_damping * speed) / m.torque_per_volt, volts)
            volts = np.clip(volts, -m.supply_v, m.supply_v)
        return volts

    def step(self, ball_xy, setpoint, arm_angle):
        roll, pitch = self.ball_loop(self.read_panel(ball_xy), setpoint)
        target = arm_angles(roll, pitch)
        angle, speed = self.read_encoders(arm_angle)
        return self.arm_loop(target, angle, speed), target
