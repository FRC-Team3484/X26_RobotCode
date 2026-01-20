from frc3484.motion import AngularPositionMotor
from commands2 import subsystem
from src.constants import TurretSubsystemConstants
from enum import Enum
from wpimath.geometry import Translation2d
from wpimath.units import degrees, inchesToMeters

# setting the current state of turret (is it homed?)
class Turret_State (Enum):
    HOME = 0 
    READY = 1
class TurretSubsystem(subsystem) :
    def __init__(self, external_encoder: None = None, )-> None:
        self._motor = AngularPositionMotor(TurretSubsystemConstants.MOTOR_CONFIG, TurretSubsystemConstants.PID_CONFIG, TurretSubsystemConstants.FEED_FORWARD_CONFIG, TurretSubsystemConstants.TRAPEZOID_CONFIG, 0, gear_ratio= TurretSubsystemConstants.GEAR_RATIO, external_encoder= external_encoder)
        self._state: Turret_State = Turret_State.HOME
        self._target: Translation2d = Translation2d()
        self._tolerance: degrees = 0

# declaring aimed means within 6in error
    def aim(self, target: Translation2d) -> None:
        self._target = target
        self._tolerance = Translation2d(target.distance(), inchesToMeters(TurretSubsystemConstants.AIM_TOLERANCE)).angle().degrees()
        self._motor.set_target_position(target.angle().degrees())

# sets power needed to aim the turret
    def set_power(self, power: float) -> None:
        self._motor.set_power(power)

# checks if the turret is aimed correctly and within acceptable percent error
    def is_aimed(self) -> bool:
        if self._target.X() == 0 and self._target.Y() == 0 :
            return False
        error: degrees = abs(self._motor.get_position() - self._target.angle().degrees())
        return error <= self._tolerance