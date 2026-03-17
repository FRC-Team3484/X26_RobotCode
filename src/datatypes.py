from dataclasses import dataclass

from wpimath.units import degrees

from frc3484.datatypes import SC_LauncherSpeed

@dataclass(frozen=True)
class IntakePosition:
    pivot_angle: degrees
    roller_power: float
    disable_pivot: bool = False

@dataclass(frozen=True)
class FeederSpeed:
    top_speed: SC_LauncherSpeed
    bottom_speed: SC_LauncherSpeed