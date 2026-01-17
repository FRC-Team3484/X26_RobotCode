from commands2 import Subsystem

from frc3484.motion import SC_AngularFeedForwardConfig, VelocityMotor, SC_MotorConfig, SC_PIDConfig
from wpilib import DigitalInput

from constants import IndexerSubsystenConstants

class IndexerSubsystem(Subsystem):
    def __init__(self, motor_config: SC_MotorConfig, pid_config: SC_PIDConfig, feed_forward_config: SC_AngularFeedForwardConfig, gear_ratio: float, tolerance: float) -> None:
        super().__init__()

        self._motor: VelocityMotor = VelocityMotor(motor_config, pid_config, feed_forward_config, gear_ratio, tolerance)
        self._piece_sensor: DigitalInput = DigitalInput(IndexerSubsystenConstants.PIECE_SENSOR_ID)