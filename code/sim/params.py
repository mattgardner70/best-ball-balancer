"""Design parameters shared by the simulation (and later the CAD).

All lengths in metres, masses in kg, angles in radians unless noted.
Geometry follows the 3-RRS layout: three legs at 120 deg, each
motor -> arm -> elbow pin -> rod -> rod end -> plate.
"""

from dataclasses import dataclass
from math import radians


@dataclass(frozen=True)
class Geometry:
    motor_radius: float = 0.030      # motor shaft axis distance from centre (GA37 packaging)
    motor_height: float = 0.040      # motor shaft height above the desk
    arm_length: float = 0.035        # shaft to elbow pin
    rod_length: float = 0.059        # elbow pin to rod-end centre (plate within 1" of the base when parked)
    plate_joint_radius: float = 0.065  # ball-link centres on the plate
    leg_angles_deg: tuple = (0.0, 120.0, 240.0)  # leg 1 points +x, along the base's long side
    arm_min: float = radians(-25)    # hard stop (park)
    arm_max: float = radians(45)

    @property
    def home_plate_height(self) -> float:
        """Rod-end plane height with arms horizontal and rods vertical."""
        return self.motor_height + self.rod_length


@dataclass(frozen=True)
class Plate:
    # 8.4" panel 187 x 141 mm sits on a printed plate.
    size_x: float = 0.200
    size_y: float = 0.200
    thickness: float = 0.0056        # holder floor 2.6 + panel 3.0, as one slab (cad/parts.py)
    joint_drop: float = 0.013        # rod-end centre below the holder underside
    mass: float = 0.33               # printed holder ~80-150 g + 3 mm glass panel ~198 g + hardware ~10 g
    # Area the ball centre can reach (lip at the panel outline).
    reach_x: float = 0.149
    reach_y: float = 0.103


@dataclass(frozen=True)
class Ball:
    radius: float = 0.01905          # 1.5"
    mass: float = 0.227              # steel


@dataclass(frozen=True)
class Motor:
    """GA37-520 12 V, 30:1, at the output shaft (research ticket 002)."""
    supply_v: float = 12.0
    stall_torque: float = 0.39       # N*m  (~4.0 kg*cm, conservative)
    stall_current: float = 3.2       # A
    no_load_speed: float = 37.7      # rad/s (360 rpm)
    driver_current_limit: float = 1.2  # TB6612 continuous, per channel
    counts_per_rev: int = 1320       # 11 PPR x4 x 30:1

    @property
    def torque_per_volt(self) -> float:
        return self.stall_torque / self.supply_v

    @property
    def back_emf_damping(self) -> float:
        """Torque lost per rad/s of shaft speed at full voltage."""
        return self.stall_torque / self.no_load_speed

    @property
    def torque_per_amp(self) -> float:
        return self.stall_torque / self.stall_current


@dataclass(frozen=True)
class Control:
    rate_hz: float = 200.0
    max_tilt: float = radians(12)
    max_tilt_rate: float = radians(40)  # per second, for quiet
    panel_noise_std: float = 0.0003  # m, research ticket 003
    # Ball loop: desired ball acceleration from position error (m/s^2 per m)
    ball_kp: float = 4.0
    ball_ki: float = 0.4
    ball_kd: float = 3.2
    # Arm loop: volts per rad
    arm_kp: float = 220.0
    arm_ki: float = 600.0
    arm_kd: float = 1.5


GEOMETRY = Geometry()
PLATE = Plate()
BALL = Ball()
MOTOR = Motor()
CONTROL = Control()
