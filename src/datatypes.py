from enum import Enum
from dataclasses import dataclass

from wpimath.units import degrees
from wpimath.geometry import Translation2d

from frc3484.motion import SC_SpeedRequest

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
    top_speed: SC_SpeedRequest
    bottom_speed: SC_SpeedRequest

class LauncherTarget:
    def __init__(self, turret_target: Translation2d = Translation2d(), flywheel_speed: SC_SpeedRequest = SC_SpeedRequest(0, 0)) -> None:
        self.turret_target: Translation2d = turret_target
        self.flywheel_speed: SC_SpeedRequest = flywheel_speed