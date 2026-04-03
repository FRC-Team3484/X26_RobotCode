from typing import Literal

from commands2 import Command, Subsystem, InstantCommand
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts
from wpilib import DataLogManager, SmartDashboard
from wpiutil.log import DataLog, FloatLogEntry

from frc3484.motion import VelocityMotor, SC_SpeedRequest

from src.constants import FlywheelSubsystemConstants
from src.config import LOGGING_ENABLED

class FlywheelSubsystem(Subsystem):
    """
    Handles the movement of the flywheel
    """
    def __init__(self) -> None:
        
        super().__init__()
        
        self._motor: VelocityMotor = VelocityMotor(
            FlywheelSubsystemConstants.MOTOR_CONFIG,
            FlywheelSubsystemConstants.PID_CONFIG,
            FlywheelSubsystemConstants.FEED_FORWARD_CONFIG,
            FlywheelSubsystemConstants.GEAR_RATIO,
            FlywheelSubsystemConstants.TOLERANCE,
            LOGGING_ENABLED
        )

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
                'flywheel'
            )
        )

        # Logging
        if LOGGING_ENABLED:
            log: DataLog = DataLogManager.getLog()
            self._rpm_log: FloatLogEntry = FloatLogEntry(log, "/flywheel/rpm")

        SmartDashboard.putBoolean("Flywheel Diagnostics", False)

        self.setDefaultCommand(InstantCommand(lambda: self.set_power(0), self))

    def periodic(self) -> None:
        if SmartDashboard.getBoolean("Flywheel Diagnostics", False):
            self.print_diagnostics()

        if LOGGING_ENABLED:
            self._rpm_log.append(self._motor.get_mechanism_speed())
            self._motor.log_diagnostics()

    def set_speed(self, speed: SC_SpeedRequest) -> None:
        """
        Sets the motor speed

        Parameters:
            - speed (SC_LauncherSpeed): the speed and power to set the motor to
        """
        if speed.speed >= FlywheelSubsystemConstants.FULL_POWER_THRESHOLD and speed.power == 0.0:
            self._motor.set_mechanism_speed(SC_SpeedRequest(FlywheelSubsystemConstants.FULL_POWER_THRESHOLD, 1.0))
        else:
            self._motor.set_mechanism_speed(speed)

    def set_power(self, power: float) -> None:
        self._motor.set_power(power)

    def is_at_speed(self) -> bool:
        """
        Checks whether the motor is at the target speed

        Returns:
            - bool: `True` if the motor is at the target speed, `False` otherwise
        """
        return self._motor.mechanism_at_target_speed()

    def print_diagnostics(self) -> None:
        self._motor.print_diagnostics()

        SmartDashboard.putBoolean("Flywheel is at speed", self.is_at_speed())

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
        log.motor(f'flywheel') \
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

r'''
this is a very important comment dont delete
      ________
     /       /\       ______ cube™ :O
    /       /..\     /        so coool :p
   /_______/....\ <-/
   \#######\..../ 
    \#######\../ 
     \#######\/

(c) SOUPOFFICE.AI™: always and forever

yayayyayayyayay
'''
