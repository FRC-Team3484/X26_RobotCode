from typing import Literal, override

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import DigitalInput, SmartDashboard
from frc3484.motion import VelocityMotor, SC_LauncherSpeed
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts

from src.constants import FeederSubsystemConstants

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

        self._sys_id_routine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                self._set_voltage,
                self._log_motors,
                self,
                'feeder'
            )
        )

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

    def has_piece(self) -> bool:
        """
        Returns the value of the piece sensor

        Returns:
            `bool`: whether or not the piece sensor is true
        """
        return self._piece_sensor.get()
    
    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Indexer Target Velocity", self._target_velocity.speed)
        _ = SmartDashboard.putNumber("Indexer Target Power", self._target_velocity.power)
        _ = SmartDashboard.putBoolean("Indexer Piece Sensor", self._piece_sensor.get())

    def _set_voltage(self, voltage: volts) -> None:
        '''
        Sets the voltage of the motor

        Parameters:
            - voltage (volts): the voltage to set the motor to
        '''
        self._motor.set_raw_voltage(voltage)

    def _log_motors(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the motor outputs for SysId routines

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        log.motor(f'feeder') \
            .voltage(self._motor.get_raw_voltage()) \
            .angularPosition(self._motor.get_raw_position()) \
            .angularVelocity(self._motor.get_raw_velocity())

    def get_sysid_command(self, mode: Literal['quasistatic', 'dynamic'], direction: SysIdRoutine.Direction) -> Command:
        '''
        Returns a SysID command for the specified mode and direction

        Parameters:
            - mode (Literal['quasistatic', 'dynamic']): the mode to run the SysID routine in
            - direction (SysIdRoutine.Direction): the direction to run the SysID routine in
        '''
        if mode == "quasistatic":
            return self._sys_id_routine.quasistatic(direction)
        elif mode == "dynamic":
            return self._sys_id_routine.dynamic(direction)