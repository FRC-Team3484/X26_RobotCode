from typing import override

from wpilib import DigitalInput, SmartDashboard
from wpimath.units import degrees
from commands2 import Subsystem 

from frc3484.motion import PowerMotor, AngularPositionMotor

from src.constants import IntakeSubsystemConstants

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
        self._test_mode: bool = False 
        self._homed: bool = False

    def set_roller_power(self, power: float) -> None:
        """
        Sets power to the roller motor

        Parameters:
            - power (`float`): the power to set the roller motor to
        """
        if self._homed:
            self._roller_motor.set_power(power)
            self._test_mode = True 

    def set_pivot_power(self, power: float) -> None:
        """
        Sets power to the pivot motor

        Parameters:
            - power (`float`): the power to set the pivot motor to
        """
        if self._homed:
            self._pivot_motor.set_power(power)
            self._test_mode = True 

    def set_pivot_angle(self, angle: degrees) -> None:
        """
        Sets angle to the pivot motor

        Parameters:
            - angle (`degrees`): the angle to set the pivot motor to
        """
        if self._homed:
            self._pivot_motor.set_target_position(angle)
            self._test_mode = False

    @override
    def periodic(self) -> None:
        """
        Runs every loop

        Sets the power of the roller motor based on the position of the pivot motor
        """
        if not self._test_mode: 
            if abs(self._pivot_motor.get_position() - IntakeSubsystemConstants.PIVOT_HOME_POSITION) < IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE:
                self._roller_motor.set_power(0)
            else:
                self._roller_motor.set_power(IntakeSubsystemConstants.INTAKE_POWER)

        if self._home_sensor.get():
            self._homed = True

    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        SmartDashboard.putNumber("Intake Motor Position", self._pivot_motor.get_position())