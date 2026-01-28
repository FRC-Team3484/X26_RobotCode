from typing import override

from wpilib import SmartDashboard
from wpimath.units import degrees
from commands2 import Subsystem 
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs import MagnetSensorConfigs
from phoenix6.hardware import CANcoder
from phoenix6.signals import SensorDirectionValue

from frc3484.motion import PowerMotor, AngularPositionMotor
from src.constants import IntakeSubsystemConstants

class IntakeSubsystem(Subsystem):
    """
    Handles the intake, for picking up game pieces and moving them to the hopper/indexer
    """
    def __init__(self) -> None:
        super().__init__()

        # Create encoder and motor
        self._pivot_encoder: CANcoder = CANcoder(IntakeSubsystemConstants.PIVOT_ENCODER_ID, IntakeSubsystemConstants.PIVOT_ENCODER_CANBUS_NAME)

        self._roller_motor: PowerMotor = PowerMotor(IntakeSubsystemConstants.ROLLER_MOTOR_CONFIG)
        self._pivot_motor: AngularPositionMotor = AngularPositionMotor(
            IntakeSubsystemConstants.PIVOT_MOTOR_CONFIG, 
            IntakeSubsystemConstants.PIVOT_PID_CONFIG, 
            IntakeSubsystemConstants.PIVOT_FEED_FORWARD_CONFIG, 
            IntakeSubsystemConstants.PIVOT_TRAPEZOID_CONFIG, 
            IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE, 
            IntakeSubsystemConstants.PIVOT_GEAR_RATIO, 
            self._pivot_encoder
        )
        
        # Configure encoder
        encoder_config: CANcoderConfiguration = CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs() \
            .with_magnet_offset(IntakeSubsystemConstants.PIVOT_ENCODER_OFFSET) \
            .with_sensor_direction(SensorDirectionValue(IntakeSubsystemConstants.PIVOT_ENCODER_REVERSED)) \
            .with_absolute_sensor_discontinuity_point(0.5)
        self._pivot_encoder.configurator.apply(encoder_config)

        # Create follower
        self._follow_pivot_motor: PowerMotor = PowerMotor(IntakeSubsystemConstants.SECOND_PIVOT_MOTOR_CONFIG)
        self._follow_pivot_motor.follow(self._pivot_motor)

        # Variables
        self._test_mode: bool = False 

    def set_roller_power(self, power: float) -> None:
        """
        Sets power to the roller motor

        Parameters:
            - power (`float`): the power to set the roller motor to
        """
        self._roller_motor.set_power(power)
        self._test_mode = True 

    def set_pivot_power(self, power: float) -> None:
        """
        Sets power to the pivot motor

        Parameters:
            - power (`float`): the power to set the pivot motor to
        """
        self._pivot_motor.set_power(power)
        self._test_mode = True 

    def set_pivot_angle(self, angle: degrees) -> None:
        """
        Sets angle to the pivot motor

        Parameters:
            - angle (`degrees`): the angle to set the pivot motor to
        """
        self._pivot_motor.set_target_position(angle)
        self._test_mode = False

    @override
    def periodic(self) -> None:
        """
        Runs every loop

        Sets the power of the roller motor based on the position of the pivot motor
        """
        if not self._test_mode: 
            if abs((self._pivot_encoder.get_absolute_position().value * 360) - IntakeSubsystemConstants.PIVOT_HOME_POSITION) < IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE:
                self._roller_motor.set_power(0)
            else:
                self._roller_motor.set_power(IntakeSubsystemConstants.INTAKE_POWER)

    def print_diagnostics(self) -> None:
        """
        Prints diagnostics to SmartDashboard
        """
        SmartDashboard.putNumber("Intake Position", self._pivot_encoder.get_absolute_position().value * 360)