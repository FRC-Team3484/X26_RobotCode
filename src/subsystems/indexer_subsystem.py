from typing import override

from commands2 import Subsystem
from wpilib import DigitalInput, SmartDashboard
from frc3484.motion import VelocityMotor, SC_LauncherSpeed

from constants import IndexerSubsystenConstants

class IndexerSubsystem(Subsystem):
    """
    Indexer Subsystem

    Handles moving game pieces from the intake to the funnel or launcher
    """
    def __init__(self) -> None:
        super().__init__()

        self._motor: VelocityMotor = VelocityMotor(
            IndexerSubsystenConstants.MOTOR_CONFIG, 
            IndexerSubsystenConstants.PID_CONFIG, 
            IndexerSubsystenConstants.FEED_FORWARD_CONFIG, 
            IndexerSubsystenConstants.GEAR_RATIO, 
            IndexerSubsystenConstants.TOLERANCE
        )
        self._piece_sensor: DigitalInput = DigitalInput(
            IndexerSubsystenConstants.PIECE_SENSOR_ID
        )

        self._target_velocity: SC_LauncherSpeed

    @override
    def periodic(self) -> None:
        """
        Runs every loop. If the piece sensor is true and the indexer is not moving, remove the piece

        Also prints diagnostics if enabled
        """
        if self._piece_sensor.get() and self._target_velocity.speed == 0 and self._target_velocity.power == 0:
            self._motor.set_speed(IndexerSubsystenConstants.REMOVE_PIECE_VELOCITY)

        if SmartDashboard.getBoolean("Indexer Diagnostics", False):
            self.print_diagnostics()

    def set_velocity(self, velocity: SC_LauncherSpeed) -> None:
        """
        Sets the velocity of the indexer

        Paramaters:
            velocity: SC_LauncherSpeed
        """
        self._target_velocity = velocity
        self._motor.set_speed(self._target_velocity)
    
    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Indexer Target Velocity", self._target_velocity.speed)
        _ = SmartDashboard.putNumber("Indexer Target Power", self._target_velocity.power)
        _ = SmartDashboard.putBoolean("Indexer Piece Sensor", self._piece_sensor.get())