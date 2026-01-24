from typing import override

from commands2 import Subsystem
from wpilib import DigitalInput, SmartDashboard
from frc3484.motion import VelocityMotor, SC_LauncherSpeed

from constants import FeederSubsystemConstants

class FeederSubsystem(Subsystem):
    """
    Feeder Subsystem

    Handles moving game pieces from the intake to the funnel or launcher
    """
    def __init__(self) -> None:
        super().__init__()

        self._motor: VelocityMotor = VelocityMotor(
            FeederSubsystemConstants.MOTOR_CONFIG, 
            FeederSubsystemConstants.PID_CONFIG, 
            FeederSubsystemConstants.FEED_FORWARD_CONFIG, 
            FeederSubsystemConstants.GEAR_RATIO, 
            FeederSubsystemConstants.TOLERANCE
        )
        self._piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.PIECE_SENSOR_ID
        )

        self._target_velocity: SC_LauncherSpeed

    @override
    def periodic(self) -> None:
        """
        Runs every loop. If the piece sensor is true and the feeder is not moving, remove the piece

        Also prints diagnostics if enabled
        """
        if self._piece_sensor.get() and self._target_velocity.speed == 0 and self._target_velocity.power == 0:
            self._motor.set_speed(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY)

        if SmartDashboard.getBoolean("Indexer Diagnostics", False):
            self.print_diagnostics()

    def set_velocity(self, velocity: SC_LauncherSpeed) -> None:
        """
        Sets the velocity of the feeder

        Args:
            velocity (`SC_LauncherSpeed`): the velocity and power to set the indexer to
        """
        self._target_velocity = velocity
        self._motor.set_speed(self._target_velocity)

    def set_power(self, power: float) -> None:
        """
        Sets the power of the feeder

        Args:
            power (`float`): the power to set the feeder to
        """
        self._motor.set_power(power)
    
    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Indexer Target Velocity", self._target_velocity.speed)
        _ = SmartDashboard.putNumber("Indexer Target Power", self._target_velocity.power)
        _ = SmartDashboard.putBoolean("Indexer Piece Sensor", self._piece_sensor.get())