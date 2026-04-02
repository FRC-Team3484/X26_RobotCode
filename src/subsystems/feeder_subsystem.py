from typing import Literal, override

from commands2 import Command, Subsystem, InstantCommand
from commands2.sysid import SysIdRoutine
from wpilib import DigitalInput, SmartDashboard
from frc3484.motion import VelocityMotor, SC_SpeedRequest
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts
from wpilib import DriverStation

from src.constants import FeederSubsystemConstants
from src.datatypes import FeederSpeed
from src.config import LOGGING_ENABLED

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
            FeederSubsystemConstants.BOTTOM_MOTOR_TOLERANCE,
            LOGGING_ENABLED
        )
        # The rollers at the top of the hopper that push the pieces up into the turret
        self._top_motor: VelocityMotor = VelocityMotor(
            FeederSubsystemConstants.TOP_MOTOR_CONFIG, 
            FeederSubsystemConstants.TOP_PID_CONFIG, 
            FeederSubsystemConstants.TOP_FEED_FORWARD_CONFIG, 
            FeederSubsystemConstants.TOP_MOTOR_GEAR_RATIO, 
            FeederSubsystemConstants.TOP_MOTOR_TOLERANCE,
            LOGGING_ENABLED
        )
        self._entry_piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.ENTRY_PIECE_SENSOR_ID
        )
        self._exit_piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.EXIT_PIECE_SENSOR_ID
        )

        SmartDashboard.putBoolean("Indexer Diagnostics", False)

        self._target_velocity: FeederSpeed = FeederSubsystemConstants.STOP_VELOCITY

        self.setDefaultCommand(InstantCommand(lambda: self.set_power((0,0)), self))

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
        if self.has_piece() and self._target_velocity == FeederSubsystemConstants.STOP_VELOCITY:
            velocity: FeederSpeed = FeederSubsystemConstants.REMOVE_PIECE_VELOCITY
        else:
            velocity: FeederSpeed = self._target_velocity

        self._bottom_motor.set_mechanism_speed(velocity.bottom_speed)
        self._top_motor.set_mechanism_speed(velocity.top_speed)

        if SmartDashboard.getBoolean("Indexer Diagnostics", False):
            self.print_diagnostics()

        if LOGGING_ENABLED:
            self._top_motor.log_diagnostics()
            self._bottom_motor.log_diagnostics()

    def set_velocity(self, velocity: FeederSpeed) -> None:
        """
        Sets the velocity of the feeder's motors

        Args:
            velocity (`FeederSpeed`): the velocity and power to set the feeder to

        """
        self._target_velocity = velocity

    def set_power(self, power: tuple[float, float]) -> None:  
        """
        Sets the power of the feeder's motors

        Args:
            power (`tup;e[float, float]`): the power to set the feeder to, where the first item is the pull motor and the second item is the push motor
        """
        self._target_velocity = FeederSpeed(
            SC_SpeedRequest(0, power[0]),
            SC_SpeedRequest(0, power[1])
        )

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
        _ = SmartDashboard.putNumber("Feeder Bottom Motor Target Velocity", self._target_velocity.bottom_speed.speed)
        _ = SmartDashboard.putNumber("Feeder Bottom Motor Target Power", self._target_velocity.bottom_speed.power)
        _ = SmartDashboard.putNumber("Feeder Top Motor Target Velocity", self._target_velocity.bottom_speed.speed)
        _ = SmartDashboard.putNumber("Feeder Top Motor Target Power", self._target_velocity.bottom_speed.power)

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