from commands2 import Command

from subsystems.launcher_subsystem import LauncherSubsystem
from constants import LauncherSubsystemConstants

from constants import 

class LauncherAutonCommand(Command):
    def __init__(self, launcher: LauncherSubsystem, launcher_constants: LauncherSubsystemConstants):
        super().__init__()
        self.launcher = launcher
        self.launcher_constants = launcher_constants

    def execute(self):
        # self.launcher.aim_at()
        pass
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()