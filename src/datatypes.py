from enum import Enum
from dataclasses import dataclass

from wpimath.units import degrees
from wpimath.geometry import Translation2d

from frc3484.datatypes import SC_LauncherSpeed

class TargetType(Enum):
    NONE = ""
    HUB = "hub"
    TARGET_1 = "target 1"
    TARGET_2 = "target 2"

@dataclass(frozen=True)
class IntakePosition:
    pivot_angle: degrees
    roller_power: float
    disable_pivot: bool = False

@dataclass(frozen=True)
class FeederSpeed:
    top_speed: SC_LauncherSpeed
    bottom_speed: SC_LauncherSpeed

class LauncherTarget:
    def __init__(self, turret_target: Translation2d = Translation2d(), flywheel_speed: SC_LauncherSpeed = SC_LauncherSpeed(0, 0)) -> None:
        self.turret_target: Translation2d = turret_target
        self.flywheel_speed: SC_LauncherSpeed = flywheel_speed