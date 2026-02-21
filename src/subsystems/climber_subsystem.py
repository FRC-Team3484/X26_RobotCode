from typing import override

from commands2 import Subsystem

from frc3484.motion import PowerMotor

from src.constants import ClimberSubsystemConstants

class ClimberSubsystem(Subsystem):
    """
    Climber Subsystem

    Handles the movement of the climber motor
    """
    def __init__(self) -> None:
        super().__init__()

        self._motor: PowerMotor = PowerMotor(ClimberSubsystemConstants.MOTOR_CONFIG)
    
    def set_power(self, power: float) -> None:
        """
        Sets the power of the climber motor

        Args:
            power (`float`): the power to set the climber motor to
        """
        self._motor.set_power(power)

    @override
    def periodic(self) -> None:
        pass