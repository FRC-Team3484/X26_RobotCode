from src.constants import IntakeSubsystemConstants
from commands2 import Subsystem 
from frc3484.motion import PowerMotor, AngularPositionMotor


class IntakesubSystem(Subsystem):
    def __init__(self)-> None:
        self._rollermotor = PowerMotor(IntakeSubsystemConstants.MOTOR_CONFIG)
        self._deploymotor = AngularPositionMotor()

    def set_power(self, power: float)->None:
        self._rollermotor.set_power(power)

   

         