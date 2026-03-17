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

        self._has_deployed: bool = False

    @override
    def execute(self) -> None:
        if self._oi.get_intake():
            self._intake.set_pivot(IntakeSubsystemConstants.DEPLOY_POSITION)
            self._has_deployed = True
        elif self._oi.get_retract_intake():
            self._intake.set_pivot(IntakeSubsystemConstants.HOME_POSITION)
            self._has_deployed = False
        elif self._has_deployed:
            self._intake.set_pivot(IntakeSubsystemConstants.STOW_POSITION)

    @override
    def isFinished(self) -> bool:
            return False
    
    @override
    def end(self, interrupted: bool) -> None:
        self._intake.stop_motors()