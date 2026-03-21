from typing import Literal
from commands2 import Command
from wpimath.geometry import Translation2d

from src.datatypes import TargetType
from src.subsystems.launcher_subsystem import LauncherSubsystem
from src.subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem

class AutonScoreCommand(Command):
    """
    A command for scoring while in auton

    Parameters:
        - launcher ('LauncherSubsystem'): subsytem for launcher operation
        - target ('Translation2d'): gets the target for aiming the feed target in auton
        - target_type ('Literal["hub", "feed"]'): gets the type of target
    """
    def __init__(self, launcher: LauncherSubsystem | TurretlessLauncherSubsystem, target: Translation2d, target_type: Literal["hub", "feed"]):
        super().__init__()
        self._launcher: LauncherSubsystem | TurretlessLauncherSubsystem = launcher
        self._target: Translation2d = target
        self._target_type: Literal["hub", "feed"] = target_type

        self.addRequirements(launcher)

    def initialize(self):
        pass

    def execute(self) -> None:
        self._launcher.fire_at(TargetType.HUB)

    def end(self, interrupted: bool) ->  None:
        self._launcher.stop()

    def isFinished(self) -> bool:
        return False