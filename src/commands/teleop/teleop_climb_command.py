from commands2 import Command

from src.subsystems.climber_subsystem import ClimberSubsystem
from src.constants import ClimberSubsystemConstants
from src.oi import OperatorInterface


class TeleopClimbCommand(Command):
    """
    Teleop command for the climmber

    Makes the climmber move up and down when button pressed and not move when not pressed
    
    Parameters: 
        - oi (`OperatorInterface`): this is the operatorintface for the command
        - climber_subystem (`ClimberSubystem`): the climmber subystem
    """

    def __init__(self, oi: OperatorInterface, climber_subystem: ClimberSubsystem):
        self._climber_subsystem: ClimberSubsystem = climber_subystem
        self._oi: OperatorInterface = oi

    def execute(self):
        if self._oi.get_climber_extend():
            self._climber_subsystem.set_power(ClimberSubsystemConstants.UP_POWER)
        elif self._oi.get_climber_retract():
            self._climber_subsystem.set_power(ClimberSubsystemConstants.DOWN_POWER)
        else: 
            self._climber_subsystem.set_power(0)

    def end(self, interrupted: bool):
        self._climber_subsystem.set_power(0)

    def isFinished(self) -> bool:
        return False 
