from enum import Enum
from typing import override

from wpilib import DigitalInput, SmartDashboard
from wpimath.units import degrees
from commands2 import Subsystem 

from frc3484.motion import PowerMotor, AngularPositionMotor

from src.constants import IntakeSubsystemConstants

class State(Enum):
    HOMING = 0
    READY = 1
    TESTING = 2

class IntakeSubsystem(Subsystem):
    """
    Handles the intake, for picking up game pieces and moving them to the hopper/indexer
    """
    def __init__(self) -> None:
        super().__init__()

        # Create motor
        self._roller_motor: PowerMotor = PowerMotor(IntakeSubsystemConstants.ROLLER_MOTOR_CONFIG)
        self._pivot_motor: AngularPositionMotor = AngularPositionMotor(
            IntakeSubsystemConstants.PIVOT_MOTOR_CONFIG, 
            IntakeSubsystemConstants.PIVOT_PID_CONFIG, 
            IntakeSubsystemConstants.PIVOT_FEED_FORWARD_CONFIG, 
            IntakeSubsystemConstants.PIVOT_TRAPEZOID_CONFIG, 
            IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE, 
            IntakeSubsystemConstants.PIVOT_GEAR_RATIO, 
        )

        # Create follower
        self._follow_pivot_motor: PowerMotor = PowerMotor(IntakeSubsystemConstants.SECOND_PIVOT_MOTOR_CONFIG)
        self._follow_pivot_motor.follow(self._pivot_motor)

        # Home Sensor
        self._home_sensor: DigitalInput = DigitalInput(IntakeSubsystemConstants.PIVOT_HOME_SENSOR_ID)

        # Variables
        self._state: State = State.HOMING
        self._target_position: degrees = IntakeSubsystemConstants.PIVOT_HOME_POSITION

    def set_roller_power(self, power: float) -> None:
        """
        Sets power to the roller motor

        Parameters:
            - power (`float`): the power to set the roller motor to
        """
        self._state = State.TESTING 
        self._roller_motor.set_power(power)

    def set_pivot_power(self, power: float) -> None:
        """
        Sets power to the pivot motor

        Parameters:
            - power (`float`): the power to set the pivot motor to
        """
        self._state = State.TESTING 
        self._pivot_motor.set_power(power)

    def set_pivot_angle(self, angle: degrees) -> None:
        """
        Sets angle to the pivot motor

        Parameters:
            - angle (`degrees`): the angle to set the pivot motor to
        """
        self._target_position = angle

        if self._state == State.TESTING:
            self._state = State.HOMING

    @override
    def periodic(self) -> None:
        """
        Runs every loop

        Sets the power of the roller motor based on the position of the pivot motor
        """
        if self.get_homed() and self._target_position == IntakeSubsystemConstants.PIVOT_HOME_POSITION:
            self._pivot_motor.set_encoder_position(IntakeSubsystemConstants.PIVOT_HOME_POSITION)
            if self._state == State.HOMING:
                self._state = State.READY

        match self._state:
            case State.HOMING:
                pass

            case State.READY:
                if self._target_position == IntakeSubsystemConstants.PIVOT_HOME_POSITION and self.get_homed():
                    self._pivot_motor.set_power(0)
                    self._roller_motor.set_power(0)

                elif self._pivot_motor.get_position() > IntakeSubsystemConstants.PIVOT_DEPLOY_POSITION - IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE: 
                    self._pivot_motor.set_power(0)
                    self._roller_motor.set_power(0)

                else:
                    self._roller_motor.set_power(IntakeSubsystemConstants.INTAKE_POWER)
                    self._pivot_motor.set_target_position(self._target_position)

            case State.TESTING:
                pass

    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        SmartDashboard.putNumber("Intake Motor Position", self._pivot_motor.get_position())
        SmartDashboard.putBoolean("Intake Home Sensor", self._home_sensor.get())

    def get_homed(self) -> bool:
        """
        Returns if the intake is homed or not
        """
        return not self._home_sensor.get()