from commands2 import Command
from subsystems.intake_subsystem import IntakeSubsystem
from constants import IntakeSubsystemConstants  

class AutoIntakeCommand(Command):
    """
    This makes the intake flips out and flip back in during auton

    Parameters :
        -intake (IntakeSubsystem): the intake subsystem 
    """
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self._intake_subsystem = intake_subsystem
    def initialize(self):
        self._intake_subsystem.set_pivot_angle(IntakeSubsystemConstants.PIVOT_DEPLOY_POSITION)
        self._intake_subsystem.set_roller_power(IntakeSubsystemConstants.PIVOT_INTAKE_POWER)
    def end(self, interrupted: bool):
        self._intake_subsystem.set_pivot_angle(IntakeSubsystemConstants.PIVOT_HOME_POSITION)
        self._intake_subsystem.set_roller_power(IntakeSubsystemConstants.PIVOT_INTAKE_STOP)
    def isFinished(self) -> bool:
        return False 

    
    

