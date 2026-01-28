from frc3484.motion import PowerMotor
from constants import IndexerSubsystemConstants
from commands2 import Subsystem

class IndexerSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._motor: PowerMotor = PowerMotor(IndexerSubsystemConstants.MOTOR_CONFIG)

    def set_power(self, power: float) -> None:
        self._motor.set_power(power)
    
    def periodic(self) -> None:
        pass