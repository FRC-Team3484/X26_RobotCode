from enum import Enum
from typing import override

from wpilib import DigitalInput, SmartDashboard
from wpimath.units import degrees
from commands2 import Subsystem 

from frc3484.motion import PowerMotor, AngularPositionMotor

from src.constants import IntakeSubsystemConstants
from src.datatypes import IntakePosition

class State(Enum):
    TEST = -1
    UNHOMED = 0
    READY = 1

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
            IntakeSubsystemConstants.ANGLE_TOLERANCE, 
            IntakeSubsystemConstants.GEAR_RATIO, 
        )

        # Create follower
        self._follow_pivot_motor: PowerMotor = PowerMotor(IntakeSubsystemConstants.SECOND_PIVOT_MOTOR_CONFIG)
        self._follow_pivot_motor.follow(self._pivot_motor)

        # Home Sensor
        self._homed: bool = False
        self._home_sensor: DigitalInput = DigitalInput(IntakeSubsystemConstants.HOME_SENSOR_ID)

        # Variables
        self._state: State = State.UNHOMED
        self._target_position: IntakePosition = IntakeSubsystemConstants.HOME_POSITION
        self._roller_power: float = IntakeSubsystemConstants.HOME_POSITION.roller_power

    def set_roller_power(self, power: float) -> None:
        """
        Sets power to the roller motor

        Parameters:
            - power (`float`): the power to set the roller motor to
        """
        self._state = State.TEST
        self._roller_motor.set_power(power)

    def set_pivot_power(self, power: float) -> None:
        """
        Sets power to the pivot motor

        Parameters:
            - power (`float`): the power to set the pivot motor to
        """
        self._state = State.TEST 
        self._pivot_motor.set_power(power)

    def set_pivot(self, target: IntakePosition) -> None:
        """
        Sets angle to the pivot motor

        Parameters:
            - target (`IntakePosition`): the target position to set the pivot motor to
        """
        if target.roller_power != 0:
            self._roller_power = target.roller_power
        elif self._target_position.roller_power != 0:
            self._roller_power = self._target_position.roller_power
        else:
            self._roller_power = 0

        self._target_position = target
        
        if self._state == State.TEST:
            self._state = State.UNHOMED

    def stop_motors(self) -> None:
        """
        Stops all motors in the subsystem
        """
        self.set_pivot_power(0)
        self.set_roller_power(0)

    @override
    def periodic(self) -> None:
        """
        Runs every loop

        Sets the power of the roller motor based on the position of the pivot motor
        """
        if not self._homed and self.get_homed():
            self._pivot_motor.set_encoder_position(IntakeSubsystemConstants.HOME_POSITION.pivot_angle / 360.0)
            self._homed = True

        match self._state:
            case State.UNHOMED:
                self._pivot_motor.set_power(0)
                self._roller_motor.set_power(0)
                if self._homed:
                    self._state = State.READY

            case State.READY:
                self._roller_motor.set_power(self._roller_power)
                if self._pivot_motor.at_target_position() and self._target_position.disable_pivot:
                    self._pivot_motor.set_power(0)
                else:
                    self._pivot_motor.set_target_position(self._target_position.pivot_angle)

            case State.TEST:
                pass

    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        SmartDashboard.putNumber("Intake Motor Position", self._pivot_motor.get_position())
        SmartDashboard.putBoolean("Intake Home Sensor", self.get_homed())
        SmartDashboard.putNumber("Intake Target Angle", self._target_position.pivot_angle)

    def get_homed(self) -> bool:
        """
        Returns if the intake is homed or not
        """
        return not self._home_sensor.get()