from typing import override

from commands2 import Command

from src.oi import OperatorInterface
from src.subsystems.intake_subsystem import IntakeSubsystem
from src.constants import IntakeSubsystemConstants

class TeleopIntakeCommand(Command):
    """
    Teleop command for the intake

    Upon pressing the intake button, the intake will deploy

    Parameters:
        - intake (`IntakeSubsystem`): the intake subsystem
        - driver_oi (`OperatorInterface`): the oi test interface for controller bindings
    """
    def __init__(self, intake: IntakeSubsystem, driver_oi: OperatorInterface) -> None:
        super().__init__()
        self.addRequirements(intake) 
        
        self._intake: IntakeSubsystem = intake 
        self._oi: OperatorInterface = driver_oi

    @override
    def execute(self) -> None:
        if self._oi.get_intake():
            self._intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_DEPLOY_POSITION)  
        else: 
            self._intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_HOME_POSITION)  

    @override
    def isFinished(self) -> bool:
            return False
    
    @override
    def end(self, interrupted: bool) -> None:
        self._intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_HOME_POSITION)