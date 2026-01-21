from wpimath.geometry import Translation2d
from wpimath.units import feetToMeters, inches, meters_per_second

from phoenix6.signals import NeutralModeValue
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants

from frc3484.motion import SC_LauncherSpeed, SC_MotorConfig, SC_AngularFeedForwardConfig, SC_PIDConfig
from frc3484.datatypes import SC_SwerveConfig, SC_SwerveCurrentConfig, SC_DrivePIDConfig, SC_SteerPIDConfig
from frc3484.controls import Input
from frc3484.controls import XboxControllerMap as ControllerMap

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

# Subsystems
class AgitatorSubsystemConstants:
    pass

class ClimberSubsystemConstants:
    pass

class IntakeSubsystemConstants:
    pass

class TurretSubsystemConstants:
    pass

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
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
