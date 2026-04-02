from commands2 import Command

from src.datatypes import TargetType
from src.subsystems.launcher_subsystem import LauncherSubsystem
from src.subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem

class AutonScoreCommand(Command):
    """
    A command for scoring while in auton

    Parameters:
        - launcher ('LauncherSubsystem'): subsytem for launcher operation
        - target_type ('TargetType'): gets the type of target
    """
    def __init__(self, launcher: LauncherSubsystem | TurretlessLauncherSubsystem, target: TargetType):
        super().__init__()
        self._launcher: LauncherSubsystem | TurretlessLauncherSubsystem = launcher
        self._target_type: TargetType = target

        self.addRequirements(*launcher.getSubsystems())

    def initialize(self):
        pass

    def execute(self) -> None:
        self._launcher.fire_at(self._target_type)

    def end(self, interrupted: bool) ->  None:
        self._launcher.stop()

    def isFinished(self) -> bool:
        return False