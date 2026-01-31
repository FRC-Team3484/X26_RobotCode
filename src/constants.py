from wpimath.geometry import Translation2d
from wpimath.units import feetToMeters, inches, meters_per_second, degrees, turns, radians_per_second, inchesToMeters

from phoenix6.signals import NeutralModeValue
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants

from frc3484.motion import SC_LauncherSpeed, SC_MotorConfig, SC_AngularFeedForwardConfig, SC_PIDConfig
from frc3484.datatypes import SC_SwerveConfig, SC_SwerveCurrentConfig, SC_DrivePIDConfig, SC_SteerPIDConfig, SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_LinearFeedForwardConfig, SC_TrapezoidConfig, SC_ExpoConfig, SC_LauncherSpeed
from frc3484.controls import Input, XboxControllerMap
from frc3484.controls import XboxControllerMap as ControllerMap

controller = XboxControllerMap

# Drivetrain
class SwerveConstants:
    FL: int = 0
    FR: int = 1
    BL: int = 2
    BR: int = 3

    CANBUS_NAME: str = "Drivetrain CANivore"
    PIGEON_ID: int = 22

    DRIVETRAIN_WIDTH: inches = 24.0
    DRIVETRAIN_LENGTH: inches = 24.0

    WHEEL_RADIUS: inches = 2.0
    GEAR_RATIO: float = 36000.0/5880.0
    DRIVE_SCALING: float = 1.0
    STEER_RATIO: float = 12.8 # Ratio from steer motor to wheel, steer encoder is 1:1
    MAX_WHEEL_SPEED: meters_per_second = feetToMeters(8.0) # feet per second
    MAX_ROTATION_SPEED: radians_per_second = (MAX_WHEEL_SPEED / inchesToMeters(0.5*(DRIVETRAIN_WIDTH**2 + DRIVETRAIN_LENGTH**2)**0.5))

    DRIVE_CONTROLLER = PPHolonomicDriveController( # For path following
        PIDConstants(5.0, 0.0, 0.0),
        PIDConstants(5.0, 0.0, 0.0)
    )
    ALIGNMENT_CONTROLLER = PPHolonomicDriveController( # For final alignment
        PIDConstants(10.0, 0.0, 0.0),
        PIDConstants(7.0, 0.0, 0.0)
    )

    MODULE_POSITIONS: tuple[Translation2d, ...] = (
        Translation2d(DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),   # Front Left
        Translation2d(DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2),  # Front Right
        Translation2d(-DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),  # Back Left
        Translation2d(-DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2), # Back Right
    )

    MODULE_CONFIGS: tuple[SC_SwerveConfig, ...] = (
        SC_SwerveConfig(12, 13, 18, 27.685546875, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
        SC_SwerveConfig(10, 11, 19, 12.83203125, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
        SC_SwerveConfig(16, 17, 21, 38.759765625, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
        SC_SwerveConfig(14, 15, 20, 24.9609375, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
    )

    MODULE_CURRENTS: tuple[SC_SwerveCurrentConfig, ...] = (
        SC_SwerveCurrentConfig(),
        SC_SwerveCurrentConfig(),
        SC_SwerveCurrentConfig(),
        SC_SwerveCurrentConfig()
    )

    DRIVE_PID_CONFIGS: tuple[SC_DrivePIDConfig, ...] = tuple([
        SC_DrivePIDConfig(0.3, 0.0, 0.0, 0.7311, 0.1245, 0.0136)
        for _ in range(len(MODULE_CONFIGS))
    ])

    STEER_PID_CONFIGS: tuple[SC_SteerPIDConfig, ...] = tuple([
        SC_SteerPIDConfig(100, 0.0, 0.5, 1.91, 0, 0.1)
        for _ in range(len(MODULE_CONFIGS))
    ])

class TeleopDriveConstants:
    LOW_SPEED: float = 0.35
    JOG_SPEED: float = 0.25
    
    SLOW_SPEED: meters_per_second = feetToMeters(4.0) # feet per second
    SLOW_ROTATION_SPEED: radians_per_second = (SLOW_SPEED / inchesToMeters(0.5*(SwerveConstants.DRIVETRAIN_WIDTH**2 + SwerveConstants.DRIVETRAIN_LENGTH**2)**0.5))
    SLEW_FILTER_AMOUNT: float = 1

# Subsystems
class AgitatorSubsystemConstants:
    pass

class IntakeSubsystemConstants:
    INTAKE_POWER: float = 0.5
    ROLLER_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 1
    )
    PIVOT_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 2
    )
    PIVOT_PID_CONFIG: SC_PIDConfig = SC_PIDConfig(
        
    )
    PIVOT_FEED_FORWARD_CONFIG: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(

    )
    PIVOT_TRAPEZOID_CONFIG: SC_TrapezoidConfig = SC_TrapezoidConfig(

    )
    PIVOT_ENCODER_ID: int = 1
    PIVOT_ENCODER_CANBUS_NAME: str = "rio"
    PIVOT_ENCODER_OFFSET: turns = 0
    PIVOT_ENCODER_REVERSED: bool = False
    PIVOT_HOME_POSITION: degrees = 0
    PIVOT_DEPLOY_POSITION: degrees = 10
    PIVOT_ANGLE_TOLERANCE: degrees = 5
    PIVOT_GEAR_RATIO: float = 1
    
    SECOND_PIVOT_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 2
    )

class TurretSubsystemConstants:
    MOTOR_CONFIG = SC_MotorConfig (
        can_id= 1,
        inverted= False,
        can_bus_name= "rio",
        neutral_mode= NeutralModeValue.BRAKE,
        motor_type= "falcon",
    )
    PID_CONFIG = SC_PIDConfig (
        Kp=0,
        Ki=0.0,
        Kd=0,
        Kf=0,
    ) 
    FEED_FORWARD_CONFIG = SC_AngularFeedForwardConfig (
        G= 0,
        S= 0,
        V= 0,
        A= 0
    )
    EXPO_CONFIG = SC_ExpoConfig (
        Kv= 0.12,
        Ka= 0.1
    ) 

    ENCODER_A_CAN_ID: int = 0
    ENCODER_A_CAN_BUS_NAME: str = "rio"
    ENCODER_A_OFFSET: turns = 0
    ENCODER_A_REVERSED: bool = False

    ENCODER_B_CAN_ID: int = 0
    ENCODER_B_CAN_BUS_NAME: str = "rio"
    ENCODER_B_OFFSET: turns = 0
    ENCODER_B_REVERSED: bool = False

    HOME_SENSOR_PORT: int = 1

    HOME_POWER: float = -0.1
    MINIMUM_ANGLE: degrees = -360
    MAXIMUM_ANGLE: degrees = 360
    AIM_TOLERANCE: inches = 6
    
    LOOPING_DISTANCE: degrees = 90 # How far the turret needs to move to report "looping"
    MAX_ENCODER_ERROR: float = 1 # Turret will print an error if an encoder is off by this many teeth from where it expects to be
    MAX_TURRET_ERROR: degrees = 18 # Turret will print an error if the turret is off by this many degrees from where it expects to be
    TEETH_A: int = 20
    TEETH_B: int = 21
    TEETH_TURRET: int = 200

class FlywheelSubsystemConstants:
    motor_config: SC_MotorConfig = SC_MotorConfig(
        can_id=1,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE,
        motor_type = "falcon",
        current_limit_enabled = True,
        current_threshold = 50,
        current_time=0.1,
        current_limit=20
    )
    pid_config: SC_PIDConfig = SC_PIDConfig(
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        Kf=0.0
    )
    feed_forward_config: SC_LinearFeedForwardConfig= SC_LinearFeedForwardConfig(
        G=0,
        S=0,
        V=0,
        A=0
    )
    gear_ratio: float = 0.0
    tolerance: float = 0

class IndexerSubsystemConstants:
    MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=1,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE,
        motor_type="falcon",
    )

    INDEX_POWER: float = 0.0
    STOP_POWER: float = 0.0

class ClimberSubsystemConstants:
    MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=1,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE,
        motor_type="falcon",
    )

    UP_POWER: float = 0.0
    DOWN_POWER: float = 0.0

class FeederSubsystemConstants:
    MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=1,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE, 
        motor_type="falcon",
    )
    PID_CONFIG: SC_PIDConfig = SC_PIDConfig(
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        Kf=0.0,
    )
    FEED_FORWARD_CONFIG: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(
        G=0.0,
        S=0.0,
        V=0.0,
        A=0.0
    )
    GEAR_RATIO: float = 1.0
    TOLERANCE: float = 0.0

    PIECE_SENSOR_ID: int = 1
    
    REMOVE_PIECE_VELOCITY: SC_LauncherSpeed = SC_LauncherSpeed(
        0.0, 
        -0.5
    )

class LauncherSubsystemConstants:
    pass

# User Interface
class UserInterface:
    class Driver:
        CONTROLLER_PORT: int = 0
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0
        
        THROTTLE_AXIS: Input = ControllerMap.LEFT_JOY_Y
        STRAFE_AXIS: Input = ControllerMap.LEFT_JOY_X
        ROTATION_AXIS: Input = ControllerMap.RIGHT_JOY_X

        RESET_HEADING_BUTTON: Input = ControllerMap.BACK_BUTTON
        HOLD_MODE_BUTTON: Input = ControllerMap.LEFT_BUMPER
        TOGGLE_COAST_BUTTON: Input = ControllerMap.START_BUTTON
        LOW_SPEED_MODE_BUTTON: Input = ControllerMap.RIGHT_TRIGGER
        DYNAMIC_PIVOT_BUTTON: Input = ControllerMap.RIGHT_BUMPER

        JOG_UP_BUTTON: Input = ControllerMap.DPAD_UP
        JOG_DOWN_BUTTON: Input = ControllerMap.DPAD_DOWN
        JOG_LEFT_BUTTON: Input = ControllerMap.DPAD_LEFT
        JOG_RIGHT_BUTTON: Input = ControllerMap.DPAD_RIGHT

        GOTO_REEF_BUTTON: Input = ControllerMap.A_BUTTON
        GOTO_FEEDER_STATION_BUTTON: Input = ControllerMap.B_BUTTON
        GOTO_PROCESSOR_BUTTON: Input = ControllerMap.Y_BUTTON

    class Operator:
        CONTROLLER_PORT: int = 1
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0
   
    class TestConstants1:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        QUASI_FWD_BUTTON: Input = XboxControllerMap.A_BUTTON
        QUASI_REV_BUTTON: Input = XboxControllerMap.B_BUTTON
        DYNAMIC_FWD_BUTTON: Input = XboxControllerMap.X_BUTTON
        DYNAMIC_REV_BUTTON: Input = XboxControllerMap.Y_BUTTON
        
        FLYWHEEL_INPUT: Input = controller.RIGHT_TRIGGER
        INDEXER_INPUT: Input = controller.LEFT_TRIGGER
        TURRET_INPUT: Input = controller.LEFT_JOY_X
        CLIMBER_INPUT: Input = controller.RIGHT_JOY_X
    
    class TestConstants2:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        INTAKE_ROLLER_INPUT: Input = controller.RIGHT_TRIGGER
        FEEDER_INPUT: Input = controller.LEFT_JOY_Y
        INTAKE_PIVOT_INPUT: Input = controller.RIGHT_JOY_Y
    
    class DemoController:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        FLYWHEEL_LEFT_INPUT: Input = controller.LEFT_TRIGGER
        FLYWHEEL_RIGHT_INPUT: Input = controller.RIGHT_TRIGGER

        TURRET_INPUT: Input = controller.DPAD_X
        FEED_INPUT: Input = controller.A_BUTTON
        EJECT_INPUT: Input = controller.B_BUTTON
        INTAKE_INPUT: Input = controller.X_BUTTON
        
        THROTTLE_INPUT: Input = controller.LEFT_JOY_Y
        STRAFE_INPUT: Input = controller.LEFT_JOY_X
        ROTATE_INPUT: Input = controller.RIGHT_JOY_X
