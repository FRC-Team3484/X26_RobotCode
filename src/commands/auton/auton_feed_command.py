from commands2 import Command
from subsystems.launcher_subsystem import LauncherSubsystem
from wpimath.geometry import Translation2d
class FeedAutonCommand(Command):
    """
    A command for operating feeding while in auton

    Parameters:
        - launcher ('LauncherSubsystem'): subsytem for launcher operation
        - target ('Translation2d'): gets the target for aiming the feed target in auton
    """
    def __init__(self, launcher: LauncherSubsystem, target: Translation2d):
        super().__init__()
        self._launcher = launcher
        self._target = target
        self.addRequirements(launcher)

    def initialize(self):
        pass

    def execute(self) -> None:
        self._launcher.fire_at(self._target, 'feed')

    def end(self, interrupted: bool) ->  None:
        self._launcher.stop()

    def isFinished(self) -> bool:
        return False