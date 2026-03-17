from commands2 import Command

from src.constants import IntakeSubsystemConstants  
from src.subsystems.intake_subsystem import IntakeSubsystem

class AutonIntakeCommand(Command):
    """
    This handles the intake in auton

    Parameters:
        - intake (IntakeSubsystem): the intake subsystem 
    """
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self._intake_subsystem = intake_subsystem
        self.addRequirements(self._intake_subsystem)

    def initialize(self):
        self._intake_subsystem.set_pivot(IntakeSubsystemConstants.DEPLOY_POSITION)

    def end(self, interrupted: bool):
        self._intake_subsystem.set_pivot(IntakeSubsystemConstants.HOME_POSITION)

    def isFinished(self) -> bool:
        return False 
