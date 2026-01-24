from src.constants import IntakeSubsystemConstants
from commands2 import Subsystem 
from frc3484.motion import PowerMotor, AngularPositionMotor
from phoenix6.hardware import CANcoder
from phoenix6 import configs 
from phoenix6.signals import SensorDirectionValue
from wpimath.units import degrees
from wpilib import SmartDashboard



class IntakesubSystem(Subsystem):
    def __init__(self)-> None:
        self._pivotencoder: CANcoder = CANcoder(IntakeSubsystemConstants.PIVOT_ENCODER_ID, IntakeSubsystemConstants.PIVOT_ENCODER_CANBUS_NAME)
        self._rollermotor = PowerMotor(IntakeSubsystemConstants.ROLLER_MOTOR_CONFIG)
        self._pivotmotor = AngularPositionMotor(IntakeSubsystemConstants.PIVOT_MOTOR_CONFIG, IntakeSubsystemConstants.PIVOT_PID_CONFIG, IntakeSubsystemConstants.PIVOT_FEED_FORWARD_CONFIG, IntakeSubsystemConstants.PIVOT_TRAPEZOID_CONFIG, IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE,IntakeSubsystemConstants.PIVOT_GEAR_RATIO,self._pivotencoder)
        encoder_config = configs.CANcoderConfiguration()
        encoder_config.magnet_sensor = configs.MagnetSensorConfigs() \
                .with_magnet_offset(IntakeSubsystemConstants.PIVOT_ENCODER_OFFSET) \
                .with_sensor_direction(SensorDirectionValue(IntakeSubsystemConstants.PIVOT_ENCODER_REVERSED)) \
                .with_absolute_sensor_discontinuity_point(0.5)
        self._pivotencoder.configurator.apply(encoder_config)

        self._second_pivot_motor = PowerMotor(IntakeSubsystemConstants.SECOND_PIVOT_MOTOR_CONFIG)
        self._second_pivot_motor.follow(self._pivotmotor)

        self._test_mode: bool = False 
    def set_roller_power(self, power: float)->None:
        self._rollermotor.set_power(power)
        self._test_mode = True 
    def set_pivot_power(self, power: float)->None:
        self._pivotmotor.set_power(power)
        self._test_mode = True 
    def set_pivot_angle(self, angle: degrees)->None:
        self._pivotmotor.set_target_position(angle)
        self._test_mode = False 
    def periodic(self):
        if not self._test_mode: 
            if abs(self._pivotencoder.get_absolute_position().value * 360 -IntakeSubsystemConstants.PIVOT_HOME_POSITION) < IntakeSubsystemConstants.PIVOT_ANGLE_TOLERANCE:
                self._rollermotor.set_power(0)
            else:
                self._rollermotor.set_power(IntakeSubsystemConstants.INTAKE_POWER)
    def print_diagnostics(self)->None:
        SmartDashboard.putNumber("Intake Position", self._pivotencoder.get_absolute_position().value*360) 


    
   

         