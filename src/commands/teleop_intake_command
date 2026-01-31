
from commands2 import Command
from src.oi import OperatorInterface
from src.subsystems.intake_subsystem import IntakesubSystem
from src.constants import IntakeSubsystemConstants

class TeleopDriveCommand(Command):
    def __init__(self, intake: IntakesubSystem, driver_oi: OperatorInterface|None = None) -> None:
        super().__init__()
        self.addRequirements(intake) 
        
        self._intake = intake 
        self._oi = driver_oi

    def execute(self) -> None:
        if self._oi.get_intake_button():
            self._intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_DEPLOY_POSITION)  
        else: 
            self._intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_HOME_POSITION)  

    def isFinished(self) -> bool:
            return False
    
    def end(self, interrupted: bool) -> None:
        self._intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_HOME_POSITION)

    
    