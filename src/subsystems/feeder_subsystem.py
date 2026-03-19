from typing import Literal, override

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import DigitalInput, SmartDashboard
from frc3484.motion import VelocityMotor, SC_LauncherSpeed
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts
from wpilib import DriverStation

from src.constants import FeederSubsystemConstants
from src.datatypes import FeederSpeed

class FeederSubsystem(Subsystem):
    """
    Feeder Subsystem

    Handles moving game pieces from the intake to the funnel or launcher
    """
    def __init__(self) -> None:
        super().__init__()

        # The series of belts that pull the fuel into the turret
        self._bottom_motor: VelocityMotor = VelocityMotor(
            FeederSubsystemConstants.BOTTOM_MOTOR_CONFIG, 
            FeederSubsystemConstants.BOTTOM_PID_CONFIG, 
            FeederSubsystemConstants.BOTTOM_FEED_FORWARD_CONFIG, 
            FeederSubsystemConstants.BOTTOM_MOTOR_GEAR_RATIO, 
            FeederSubsystemConstants.BOTTOM_MOTOR_TOLERANCE
        )
        # The rollers at the top of the hopper that push the pieces up into the turret
        self._top_motor: VelocityMotor = VelocityMotor(
            FeederSubsystemConstants.TOP_MOTOR_CONFIG, 
            FeederSubsystemConstants.TOP_PID_CONFIG, 
            FeederSubsystemConstants.TOP_FEED_FORWARD_CONFIG, 
            FeederSubsystemConstants.TOP_MOTOR_GEAR_RATIO, 
            FeederSubsystemConstants.TOP_MOTOR_TOLERANCE
        )
        self._entry_piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.ENTRY_PIECE_SENSOR_ID
        )
        self._exit_piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.EXIT_PIECE_SENSOR_ID
        )

        SmartDashboard.putBoolean("Indexer Diagnostics", False)

        self._bottom_target_velocity: SC_LauncherSpeed = SC_LauncherSpeed(0, 0)
        self._top_target_velocity: SC_LauncherSpeed = SC_LauncherSpeed(0, 0)

        self._bottom_sys_id_routine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                self._set_bottom_voltage,
                self._log_bottom_motor,
                self,
                'feeder'
            )
        )

        self._top_sys_id_routine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                self._set_top_voltage,
                self._log_top_motor,
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
        if not DriverStation.isTest():
            if self.has_piece() and self._bottom_target_velocity == FeederSubsystemConstants.STOP_VELOCITY.bottom_speed and self._top_target_velocity == FeederSubsystemConstants.STOP_VELOCITY.top_speed:
                self.set_velocity(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY)
            else:
                self.set_velocity(FeederSubsystemConstants.STOP_VELOCITY)

        if SmartDashboard.getBoolean("Indexer Diagnostics", False):
            self.print_diagnostics()

    def set_velocity(self, velocity: FeederSpeed) -> None:
        """
        Sets the velocity of the feeder's motors

        Args:
            velocity (`FeederSpeed`): the velocity and power to set the feeder to

        """
        self._bottom_target_velocity = velocity.bottom_speed
        self._bottom_motor.set_speed(self._bottom_target_velocity)

        self._top_target_velocity = velocity.top_speed
        self._top_motor.set_speed(self._top_target_velocity)

    def set_power(self, power: tuple[float, float]) -> None:  
        """
        Sets the power of the feeder's motors

        Args:
            power (`tup;e[float, float]`): the power to set the feeder to, where the first item is the pull motor and the second item is the push motor
        """
        self._bottom_motor.set_power(power[0])
        self._top_motor.set_power(power[1])

    def stop_motors(self) -> None:
        """
        Stops the feeder's motors
        """
        self.set_velocity(FeederSubsystemConstants.STOP_VELOCITY)

    def has_piece(self) -> bool:
        """
        Returns the value of either of the piece sensors

        Returns:
            `bool`: whether or not either of the piece sensors are true
        """
        return not self._entry_piece_sensor.get() or not self._exit_piece_sensor.get()
    
    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Feeder Bottom Motor Target Velocity", self._bottom_target_velocity.speed)
        _ = SmartDashboard.putNumber("Feeder Bottom Motor Target Power", self._bottom_target_velocity.power)
        _ = SmartDashboard.putNumber("Feeder Top Motor Target Velocity", self._top_target_velocity.speed)
        _ = SmartDashboard.putNumber("Feeder Top Motor Target Power", self._top_target_velocity.power)

        _ = SmartDashboard.putBoolean("Feeder Entry Piece Sensor", not self._entry_piece_sensor.get())
        _ = SmartDashboard.putBoolean("Feeder Exit Piece Sensor", not self._exit_piece_sensor.get())

    def _set_bottom_voltage(self, voltage: volts) -> None:
        '''
        Sets the voltage of the feeder's motors

        Parameters:
            - voltage (`tuple[volts, volts]`): the voltage to set the bottom motor to
        '''
        self._bottom_motor.set_raw_voltage(voltage)

    def _set_top_voltage(self, voltage: volts) -> None:
        '''
        Sets the voltage of the feeder's motors

        Parameters:
            - voltage (`tuple[volts, volts]`): the voltage to set the top motor to
        '''
        self._top_motor.set_raw_voltage(voltage)

    def _log_bottom_motor(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the bottom motor output for the SysId routine

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        log.motor(f'feeder bottom') \
            .voltage(self._bottom_motor.get_raw_voltage()) \
            .angularPosition(self._bottom_motor.get_raw_position()) \
            .angularVelocity(self._bottom_motor.get_raw_velocity())
        
    def _log_top_motor(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the top motor output for the SysId routine

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        log.motor(f'feeder top') \
            .voltage(self._top_motor.get_raw_voltage()) \
            .angularPosition(self._top_motor.get_raw_position()) \
            .angularVelocity(self._top_motor.get_raw_velocity())

    def get_sysid_command(self, motor: Literal['bottom', 'top'], mode: Literal['quasistatic', 'dynamic'], direction: SysIdRoutine.Direction) -> Command:
        '''
        Returns a SysID command for the specified mode and direction

        Parameters:
            - mode (Literal['quasistatic', 'dynamic']): the mode to run the SysID routine in
            - direction (SysIdRoutine.Direction): the direction to run the SysID routine in
        '''
        if motor == "bottom":
            if mode == "quasistatic":
                return self._bottom_sys_id_routine.quasistatic(direction)
            elif mode == "dynamic":
                return self._bottom_sys_id_routine.dynamic(direction)
        elif motor == "top":
            if mode == "quasistatic":
                return self._top_sys_id_routine.quasistatic(direction)
            elif mode == "dynamic":
                return self._top_sys_id_routine.dynamic(direction)