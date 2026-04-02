from commands2 import Subsystem, InstantCommand

from frc3484.motion import PowerMotor

from src.constants import IndexerSubsystemConstants
from src.config import LOGGING_ENABLED

class IndexerSubsystem(Subsystem):
    """
    Handles the movement of the indexer motor, which is used to agitate the pieces towards the turret
    """
    def __init__(self) -> None:
        super().__init__()

        self._motor: PowerMotor = PowerMotor(IndexerSubsystemConstants.MOTOR_CONFIG, LOGGING_ENABLED)

        self.setDefaultCommand(InstantCommand(lambda: self.set_power(0), self))

    def set_power(self, power: float) -> None:
        self._motor.set_power(power)
    
    def periodic(self) -> None:
        if LOGGING_ENABLED:
            self._motor.log_diagnostics()