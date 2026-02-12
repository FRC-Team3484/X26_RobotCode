from typing import Literal
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts

from constants import FlywheelSubsystemConstants
from frc3484.motion import VelocityMotor, SC_LauncherSpeed
from constants import FlywheelSubsystemConstants
from commands2 import Subsystem

class FlywheelSubsystem(Subsystem):
    """
    Handles the movement of the flywheel
    """
    def __init__(self) -> None:
        
        super().__init__()
        
        self._motor: VelocityMotor = VelocityMotor(
            FlywheelSubsystemConstants.motor_config,
            FlywheelSubsystemConstants.pid_config,
            FlywheelSubsystemConstants.feed_forward_config,
            FlywheelSubsystemConstants.gear_ratio,
            FlywheelSubsystemConstants.tolerance
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

    def periodic(self) -> None:
        pass

    def set_speed(self, speed: SC_LauncherSpeed) -> None:
        """
        Sets the motor speed

        Parameters:
            - speed (SC_LauncherSpeed): the speed and power to set the motor to
        """
        self._motor.set_speed(speed)

    def set_power(self, power: float) -> None:
        self._motor.set_power(power)

    def is_at_speed(self) -> bool:
        """
        Checks whether the motor is at the target speed

        Returns:
            - bool: `True` if the motor is at the target speed, `False` otherwise
        """
        return self._motor.at_target_speed()

    def print_diagnostics(self) -> None:
        self._motor.print_diagnostics()

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
     /       /\       ______ cube™ :OO
    /       /..\     /
   /_______/....\ <-/
   \#######\..../ 
    \#######\../ 
     \#######\/

(c) SOUPOFFICE.AI™: always and forever  
'''
