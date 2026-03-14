from typing import Literal, override

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import DigitalInput, SmartDashboard
from frc3484.motion import VelocityMotor, SC_LauncherSpeed
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts
from wpilib import DriverStation

from src.constants import FeederSubsystemConstants

class FeederSubsystem(Subsystem):
    """
    Feeder Subsystem

    Handles moving game pieces from the intake to the funnel or launcher
    """
    def __init__(self) -> None:
        super().__init__()

        # The series of belts that pull the fuel into the turret
        self._pull_motor: VelocityMotor = VelocityMotor(
            FeederSubsystemConstants.PULL_MOTOR_CONFIG, 
            FeederSubsystemConstants.PULL_PID_CONFIG, 
            FeederSubsystemConstants.PULL_FEED_FORWARD_CONFIG, 
            FeederSubsystemConstants.PULL_MOTOR_GEAR_RATIO, 
            FeederSubsystemConstants.PULL_MOTOR_TOLERANCE
        )
        # The rollers at the top of the hopper that push the pieces up into the turret
        self._push_motor: VelocityMotor = VelocityMotor(
            FeederSubsystemConstants.PUSH_MOTOR_CONFIG, 
            FeederSubsystemConstants.PUSH_PID_CONFIG, 
            FeederSubsystemConstants.PUSH_FEED_FORWARD_CONFIG, 
            FeederSubsystemConstants.PUSH_MOTOR_GEAR_RATIO, 
            FeederSubsystemConstants.PUSH_MOTOR_TOLERANCE
        )
        self._entry_piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.ENTRY_PIECE_SENSOR_ID
        )
        self._exit_piece_sensor: DigitalInput = DigitalInput(
            FeederSubsystemConstants.EXIT_PIECE_SENSOR_ID
        )

        SmartDashboard.putBoolean("Indexer Diagnostics", False)

        self._pull_target_velocity: SC_LauncherSpeed = SC_LauncherSpeed(0, 0)
        self._push_target_velocity: SC_LauncherSpeed = SC_LauncherSpeed(0, 0)

        self._pull_sys_id_routine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                self._set_pull_voltage,
                self._log_pull_motor,
                self,
                'feeder'
            )
        )

        self._push_sys_id_routine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                self._set_push_voltage,
                self._log_push_motor,
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
            if self._entry_piece_sensor.get() and self._pull_target_velocity.speed == 0 and self._pull_target_velocity.power == 0:
                self._pull_motor.set_speed(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY[0])
            if self._entry_piece_sensor.get() and self._push_target_velocity.speed == 0 and self._push_target_velocity.power == 0:
                self._push_motor.set_speed(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY[1])

        if SmartDashboard.getBoolean("Indexer Diagnostics", False):
            self.print_diagnostics()

    def set_velocity(self, velocity: tuple[SC_LauncherSpeed, SC_LauncherSpeed]) -> None:
        """
        Sets the velocity of the feeder's motors

        Args:
            velocity (`tuple[SC_LauncherSpeed, SC_LauncherSpeed]`): the velocity and power to set the indexer to, the pull motor is first, and the second item is the push motor
        """
        self._pull_target_velocity = velocity[0]
        self._pull_motor.set_speed(self._pull_target_velocity)

        self._push_target_velocity = velocity[1]
        self._push_motor.set_speed(self._push_target_velocity)

    def set_power(self, power: tuple[float, float]) -> None:  
        """
        Sets the power of the feeder's motors

        Args:
            power (`tup;e[float, float]`): the power to set the feeder to, where the first item is the pull motor and the second item is the push motor
        """
        self._pull_motor.set_power(power[0])
        self._push_motor.set_power(power[1])

    def has_piece(self) -> bool:
        """
        Returns the value of either of the piece sensors

        Returns:
            `bool`: whether or not either of the peice sensors are true
        """
        return self._entry_piece_sensor.get() or self._exit_piece_sensor.get()
    
    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Indexer Pull Motor Target Velocity", self._pull_target_velocity.speed)
        _ = SmartDashboard.putNumber("Indexer Pull Motor Target Power", self._pull_target_velocity.power)
        _ = SmartDashboard.putNumber("Indexer Push Motor Target Velocity", self._push_target_velocity.speed)
        _ = SmartDashboard.putNumber("Indexer Push Motor Target Power", self._push_target_velocity.power)

        _ = SmartDashboard.putBoolean("Indexer Entry Piece Sensor", self._entry_piece_sensor.get())
        _ = SmartDashboard.putBoolean("Indexer Exit Piece Sensor", self._exit_piece_sensor.get())

    def _set_pull_voltage(self, voltage: volts) -> None:
        '''
        Sets the voltage of the feeder's motors

        Parameters:
            - voltage (`tuple[volts, volts]`): the voltage to set the pull motor to
        '''
        self._pull_motor.set_raw_voltage(voltage)

    def _set_push_voltage(self, voltage: volts) -> None:
        '''
        Sets the voltage of the feeder's motors

        Parameters:
            - voltage (`tuple[volts, volts]`): the voltage to set the push motor to
        '''
        self._push_motor.set_raw_voltage(voltage)

    def _log_pull_motor(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the pull motor output for the SysId routine

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        log.motor(f'feeder pull') \
            .voltage(self._pull_motor.get_raw_voltage()) \
            .angularPosition(self._pull_motor.get_raw_position()) \
            .angularVelocity(self._pull_motor.get_raw_velocity())
        
    def _log_push_motor(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the push motor output for the SysId routine

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        log.motor(f'feeder push') \
            .voltage(self._push_motor.get_raw_voltage()) \
            .angularPosition(self._push_motor.get_raw_position()) \
            .angularVelocity(self._push_motor.get_raw_velocity())

    def get_sysid_command(self, motor: Literal['pull', 'push'], mode: Literal['quasistatic', 'dynamic'], direction: SysIdRoutine.Direction) -> Command:
        '''
        Returns a SysID command for the specified mode and direction

        Parameters:
            - mode (Literal['quasistatic', 'dynamic']): the mode to run the SysID routine in
            - direction (SysIdRoutine.Direction): the direction to run the SysID routine in
        '''
        if motor == "pull":
            if mode == "quasistatic":
                return self._pull_sys_id_routine.quasistatic(direction)
            elif mode == "dynamic":
                return self._pull_sys_id_routine.dynamic(direction)
        elif motor == "push":
            if mode == "quasistatic":
                return self._push_sys_id_routine.quasistatic(direction)
            elif mode == "dynamic":
                return self._push_sys_id_routine.dynamic(direction)