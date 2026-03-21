from wpimath.geometry import Rotation3d, Transform3d, Translation2d, Pose2d, Rotation2d, Translation3d
from wpimath.units import feetToMeters, inches, inchesToMeters, meters_per_second, meters, degrees, radians_per_second, seconds, turns, meters_per_second_squared
from wpilib import Color
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from phoenix6.signals import NeutralModeValue
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants

from frc3484.motion import SC_LauncherSpeed, SC_MotorConfig, SC_AngularFeedForwardConfig, SC_PIDConfig
from frc3484.datatypes import SC_SwerveConfig, SC_SwerveCurrentConfig, SC_DrivePIDConfig, SC_SteerPIDConfig, SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_TrapezoidConfig, SC_ExpoConfig, SC_LauncherSpeed, SC_ApriltagTarget
from frc3484.controls import Input, XboxControllerMap
from frc3484.controls import XboxControllerMap as ControllerMap
from frc3484.vision import SC_CameraConfig

from src.datatypes import IntakePosition, FeederSpeed

import numpy as np

controller = XboxControllerMap
    
class RobotConstants:
    """
    Constants for generalized robot properties
    """
    TICK_RATE: seconds = 0.02
    APRIL_TAG_FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)
    ALLIANCE_ZONE_POSITION: meters = inchesToMeters(182.11)

# Drivetrain
class SwerveConstants:
    """
    Constants for configuring the drivetrain and swerve modules
    """
    FL: int = 0
    FR: int = 1
    BL: int = 2
    BR: int = 3

    CANBUS_NAME: str = "Drivetrain CANivore"
    PIGEON_ID: int = 22

    DRIVETRAIN_WIDTH: inches = 27.0
    DRIVETRAIN_LENGTH: inches = 27.0

    WHEEL_RADIUS: inches = 2.0
    GEAR_RATIO: float = 36000.0/5880.0
    DRIVE_SCALING: float = 1.0
    STEER_RATIO: float = 18.75 # Ratio from steer motor to wheel, steer encoder is 1:1
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
    TURRETLESS_AIM_PID_VALUES = SC_PIDConfig(
        5,
        0,
        0,
        0
    )

    MODULE_POSITIONS: tuple[Translation2d, ...] = (
        Translation2d(inchesToMeters(DRIVETRAIN_LENGTH / 2), inchesToMeters(DRIVETRAIN_WIDTH / 2)),   # Front Left
        Translation2d(inchesToMeters(DRIVETRAIN_LENGTH / 2), inchesToMeters(-DRIVETRAIN_WIDTH / 2)),  # Front Right
        Translation2d(inchesToMeters(-DRIVETRAIN_LENGTH / 2), inchesToMeters(DRIVETRAIN_WIDTH / 2)),  # Back Left
        Translation2d(inchesToMeters(-DRIVETRAIN_LENGTH / 2), inchesToMeters(-DRIVETRAIN_WIDTH / 2)), # Back Right
    )

    MODULE_CONFIGS: tuple[SC_SwerveConfig, ...] = (
        SC_SwerveConfig(12, 13, 19, 0.142090, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
        SC_SwerveConfig(10, 11, 18, -0.321777, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
        SC_SwerveConfig(16, 17, 21, -0.336914, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
        SC_SwerveConfig(14, 15, 20, 0.199707, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING, STEER_RATIO),
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
        SC_SteerPIDConfig(0.5, 0.0, 0.0, max_velocity=12, max_acceleration=100)
        for _ in range(len(MODULE_CONFIGS))
    ])

    pigeon_pose = None

class VisionConstants:
    """
    Constants for configuring and using vision
    """
    class HubAprilTags:
        RED_ID: int = 10
        BLUE_ID: int = 26

    ClimbAprilTagTarget: SC_ApriltagTarget = SC_ApriltagTarget(
        apriltag_ids=[31],
        offsets=[],
        safe_distance=3000,
        field=AprilTagField.k2026RebuiltWelded,
        red_apriltag_ids=[15]
    )

    APRIL_TAG_FIELD: AprilTagField = AprilTagField.k2026RebuiltWelded
    SINGLE_TAG_STDDEV: tuple[float, float, float] = (4, 4, 8)
    MULTI_TAG_STDDEV: tuple[float, float, float] = (0.5, 0.5, 1)

    CAMERA_CONFIGS: tuple[SC_CameraConfig, ...] = (
        SC_CameraConfig(
            "Camera_1",
            Transform3d(
                Translation3d(
                    inchesToMeters(6.5),
                    inchesToMeters(-0.5),
                    inchesToMeters(15.25),
                ),  
                Rotation3d().fromDegrees(0, 120, 180)
            ),
            True
        ),
         SC_CameraConfig(
            "Camera_2",
            Transform3d(
                Translation3d(
                    inchesToMeters(9.5),
                    inchesToMeters(-0.5),
                    inchesToMeters(15.25),
                ),  
                Rotation3d().fromDegrees(0, 120, 0)
            ),
            True
        )
    )

class TeleopDriveConstants:
    """
    Constants for drive commands
    """
    LOW_SPEED: float = 0.35
    JOG_SPEED: float = 0.25
    
    SLOW_SPEED: meters_per_second = feetToMeters(4.0) # feet per second
    SLOW_ROTATION_SPEED: radians_per_second = (SLOW_SPEED / inchesToMeters(0.5*(SwerveConstants.DRIVETRAIN_WIDTH**2 + SwerveConstants.DRIVETRAIN_LENGTH**2)**0.5))
    SLEW_FILTER_AMOUNT: float = 1

# Subsystems
class IntakeSubsystemConstants:
    """
    Constants for the Intake Subsystem
    """
    INTAKE_POWER: float = 0.4
    ROLLER_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 31,
        inverted=True,
        neutral_mode=NeutralModeValue.COAST
    )
    PIVOT_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 30,
        motor_type="minion",
        inverted=True
    )
    PIVOT_PID_CONFIG: SC_PIDConfig = SC_PIDConfig(
        Kp=0.5,
        Ki=0.2,
        Kd=0.0,
        Kf=0.0
    )
    PIVOT_FEED_FORWARD_CONFIG: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(
        G=0.03,
        S=0.0,
        V=0.0,
        A=0.0
    )
    PIVOT_TRAPEZOID_CONFIG: SC_TrapezoidConfig = SC_TrapezoidConfig(
        40.0, #rev/s
        80.0 #rev/s^2
    )

    
    
    HOME_SENSOR_ID: int = 4
    HOME_POSITION: IntakePosition = IntakePosition(pivot_angle=50.0, roller_power=0.0, disable_pivot=True)
    DEPLOY_POSITION: IntakePosition = IntakePosition(pivot_angle=185.0, roller_power=0.45, disable_pivot=True)
    STOW_POSITION: IntakePosition = IntakePosition(pivot_angle=185.0, roller_power=0.0, disable_pivot=True)
    GEAR_RATIO: float = 23.0
    ANGLE_TOLERANCE: degrees = 5.0 * GEAR_RATIO
    
    SECOND_PIVOT_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=32,
        inverted=True,
        motor_type="minion"
    )

class TurretSubsystemConstants:
    """
    Constants for the Turret Subsystem
    """
    MOTOR_CONFIG = SC_MotorConfig (
        can_id= 60,
        inverted=True,
        can_bus_name= "rio",
        neutral_mode= NeutralModeValue.BRAKE,
        motor_type= "minion",
    )
    PID_CONFIG = SC_PIDConfig (
        Kp=4.0,
        Ki=3.5,
        Kd=0.15,
        Kf=0.0,
    ) 
    FEED_FORWARD_CONFIG = SC_AngularFeedForwardConfig (
        G=0.0,
        S=3.0,
        V=0.0,
        A=0.0
    )
    # TRAPEZOID_CONFIG = SC_TrapezoidConfig (
    #     max_velocity=10.0,
    #     max_acceleration=20.0
    # )
    TRAPEZOID_CONFIG = SC_TrapezoidConfig ()
    MOTOR_GEAR_RATIO: float = 10.0

    ENCODER_A_CHANNEL: int = 3
    ENCODER_A_OFFSET: turns = 0.0281
    ENCODER_A_REVERSED: bool = False

    ENCODER_B_CHANNEL: int = 2
    ENCODER_B_OFFSET: turns = 0.5742
    ENCODER_B_REVERSED: bool = False

    RATE_LIMIT: float = 180

    MINIMUM_ANGLE: degrees = -90
    MAXIMUM_ANGLE: degrees = 90
    AIM_TOLERANCE: inches = 12
    
    LOOPING_DISTANCE: degrees = 90 # How far the turret needs to move to report "looping"
    MAX_ENCODER_ERROR: float = 0.49 # Turret will print an error if an encoder is off by this many teeth from where it expects to be
    MAX_TURRET_ERROR: degrees = 18 # Turret will print an error if the turret is off by this many degrees from where it expects to be
    TEETH_A: int = 20
    TEETH_B: int = 21
    TEETH_TURRET: int = 200

    ENCODER_STARTUP_DELAY: seconds = 2.0
    ENCODER_OUTPUT_PERIOD: seconds = 0.001025 # Setting these values for the encoders allegedly reduce startup time
    ENCODER_MINIMUM_PULSE: seconds = 0.000001
    ENCODER_MAXIMUM_PULSE: seconds = 0.001024

class FlywheelSubsystemConstants:
    """
    Constants for the Flywheel Subsystem
    """
    motor_config: SC_MotorConfig = SC_MotorConfig(
        can_id=61,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.COAST,
        motor_type = "falcon",
        current_limit_enabled = False,
        current_threshold = 50,
        current_time=0.1,
        current_limit=50
    )
    pid_config: SC_PIDConfig = SC_PIDConfig(
        Kp=0.45,
        Ki=0.1,
        Kd=0.0,
        Kf=0.0
    )
    feed_forward_config: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(
        G=0,
        S=0.4,
        V=0.1,
        A=0
    )
    gear_ratio: float = 1.0
    tolerance: float = 50 / 60 # rpm

class IndexerSubsystemConstants:
    """
    Constants for the Indexer Subsystem
    """
    MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=40,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE,
        motor_type="minion",
    )

    INDEX_POWER: float = 0.20
    STOP_POWER: float = 0.0

class ClimberSubsystemConstants:
    """
    Constants for the Climber Subsystem
    """
    MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=80,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE,
        motor_type="falcon",
    )

    UP_POWER: float = 0.0
    DOWN_POWER: float = 0.0

class FeederSubsystemConstants:
    """
    Constants for the Feeder Subsystem
    """
    BOTTOM_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=50,
        inverted=False,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE, 
        motor_type="minion",
    )
    BOTTOM_PID_CONFIG: SC_PIDConfig = SC_PIDConfig(
        Kp=0.052243,
        Ki=0.0,
        Kd=0.0,
        Kf=0.0,
    )
    BOTTOM_FEED_FORWARD_CONFIG: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(
        G=0.0,
        S=0.0,
        V=0.092712,
        A=0.78117
    )
    BOTTOM_MOTOR_GEAR_RATIO: float = 1.0
    BOTTOM_MOTOR_TOLERANCE: float = 0.0

    TOP_MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id=51,
        inverted=True,
        can_bus_name="rio",
        neutral_mode=NeutralModeValue.BRAKE, 
        motor_type="minion",
    )
    TOP_PID_CONFIG: SC_PIDConfig = SC_PIDConfig(
        Kp=0.12976,
        Ki=0.0,
        Kd=0.0,
        Kf=0.0,
    )
    TOP_FEED_FORWARD_CONFIG: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(
        G=0.0,
        S=0.25347,
        V=0.11071,
        A=0.03327
    )
    TOP_MOTOR_GEAR_RATIO: float = 1.0
    TOP_MOTOR_TOLERANCE: float = 0.0

    ENTRY_PIECE_SENSOR_ID: int = 1
    EXIT_PIECE_SENSOR_ID: int = 0

    FEED_SPEED: FeederSpeed = FeederSpeed(
        SC_LauncherSpeed(
            speed=4000,
            power=0
        ),
        SC_LauncherSpeed(
            speed=4000,
            power=0
        )
    )

    REMOVE_PIECE_VELOCITY: FeederSpeed = FeederSpeed(
        SC_LauncherSpeed(
            speed=0.0, 
            power=-0.5
        ),
        SC_LauncherSpeed(
            speed=0.0, 
            power=-0.5
        )
    )

    STOP_VELOCITY: FeederSpeed = FeederSpeed(
        SC_LauncherSpeed(
            speed=0.0,
            power=0.0
        ),
        SC_LauncherSpeed(
            speed=0.0,
            power=0.0
        )
    )

class LEDSubsystemConstants:
    LED_PWM_PORT: int = 0
    LED_STRIP_LENGTH: int = 72

    LED_SPACING: meters = 1 / 60
    WAVELENGTH: meters = 0.25
    FUSION_SCROLLING_SPEED: meters_per_second = 0.25
    GREEN_SCROLL_SPEED: meters_per_second = 0.6
    GAMMA: float = 2.2
    BAR_SIZE: int = 12
    VELOCITY: meters_per_second = 0.5
    INTAKE_VELOCITY: meters_per_second_squared = 0.5
    EXIT_ACCELERATION: meters_per_second_squared = 0.5
    PIVOT_ANIMATION: seconds = 0.8
    MIN_VOLTAGE: float = 11.6
    FIRE_HEIGHT: int = 1
    FIRE_SPARKS: int = 2
    DELAY: int = 1
    MOVE_RATE: float = 0.05
    FILL_SIZE: int = 2
    EMPTY_SIZE: int = 2
    LOW_BATTERY_CYCLE: seconds = 2
    PURPLE_CYCLE_TIME: seconds = 1
    ALGAE_GREEN_X25: Color = Color("#10F01A")
    CORAL_PINK_X25: Color = Color("#FF0091")
    DRIVE_ORANGE_X25: Color = Color("#FF8200")
    TEAM_BLUE_X25: Color = Color("#009BB4")
    FIRE_RED_X25: Color = Color("#FF1515")
    SNOW_WHITE_X26: Color = Color("#b8b8d1")
    ICE_BLUE_X26: Color = Color("#86a0fc")
    CHARGED_GREEN_X26: Color = Color("#54815f")
    STATIC_YELLOW_X26: Color = Color("#cdf253")
    ANCIENT_PURPLE_X26: Color = Color("#cf54f4")
    COLORS: list[Color] = [ALGAE_GREEN_X25, CORAL_PINK_X25, DRIVE_ORANGE_X25, TEAM_BLUE_X25, FIRE_RED_X25, SNOW_WHITE_X26, ICE_BLUE_X26, CHARGED_GREEN_X26, STATIC_YELLOW_X26, ANCIENT_PURPLE_X26]
    COLOR_FUSION: list[Color] = [CHARGED_GREEN_X26, ICE_BLUE_X26]
    COLOR_WAVE_COLORS: list[Color] = [ICE_BLUE_X26, SNOW_WHITE_X26, CHARGED_GREEN_X26]
    STATIC_COLOR: list[Color] = [STATIC_YELLOW_X26]

class LauncherSubsystemConstants:
    """
    Constants for the Launcher Subsystem
    """

    DEBOUNCE_TIMER: seconds = 0.5
    
class FeedTargetSubsystemConstants:
    """
    Constants for the Feed Target Subsystem
    """
    TARGET_MOVE_SPEED: meters_per_second = feetToMeters(5.0)
    TARGET_1_INITIAL_POSITION: Translation2d = Translation2d(inchesToMeters(91), inchesToMeters(79.25))
    TARGET_2_INITIAL_POSITION: Translation2d = Translation2d(inchesToMeters(91), inchesToMeters(237.75))
    HUB_OFFSET: Pose2d = Pose2d(inchesToMeters(-23.5), 0.0, 0)

    TURRET_OFFSET: Pose2d = Pose2d(
        x=0,
        y=0,
        rotation=Rotation2d(0)
    )

    FEED_RPM: np.ndarray = np.array([500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000], np.float32)
    FEED_DISTANCES: np.ndarray = np.array([7.134, 15.938, 28.081, 43.392, 61.665, 82.664, 106.133, 131.802, 159.398, 188.647, 206.334, 222.949, 240.386, 258.613, 277.589, 297.263, 317.581, 338.486, 359.915], np.float32)
    FEED_FLIGHT_TIME: np.ndarray = np.array([0.296, 0.442, 0.588, 0.733, 0.876, 1.017, 1.157, 1.294, 1.429, 1.561, 1.637, 1.706, 1.776, 1.847, 1.92, 1.992, 2.066, 2.14, 2.214], np.float32)

    HUB_RPM: np.ndarray = np.array([2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000], np.float32)
    HUB_DISTANCES: np.ndarray = np.array([77.011, 105.928, 135.305, 165.778, 184.041, 201.125, 219.002, 237.644, 257.015, 277.068, 297.752, 319.012, 340.785], np.float32)
    HUB_FLIGHT_TIME: np.ndarray = np.array([0.829, 1.028, 1.199, 1.357, 1.445, 1.523, 1.601, 1.68, 1.759, 1.838, 1.917, 1.996, 2.075], np.float32)

    LATENCY: seconds = 0.05

class DoneLaunchingCommandConstants:
    """
    Constants for the DoneLaunchingCommand
    """
    TIMEOUT: seconds = 1

# User Interface
class UserInterface:
    """
    Constants for the different controller interfaces to the robot
    """
    class Driver:
        """
        Creates button inputs for the driver controller
        """
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

        GOTO_CLIMB_BUTTON: Input = ControllerMap.Y_BUTTON

    class Operator:
        """
        Creates button inputs for the operator controller
        """

        # Regular Teleop Inputs
        CONTROLLER_PORT: int = 1
        JOYSTICK_DEADBAND: float = 0.05

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        RIGHT_FEEDER_BUTTON: Input = ControllerMap.RIGHT_BUMPER
        LEFT_FEEDER_BUTTON: Input = ControllerMap.LEFT_BUMPER

        RIGHT_FEEDER_AXIS_X: Input = ControllerMap.RIGHT_JOY_X
        RIGHT_FEEDER_AXIS_Y: Input = ControllerMap.RIGHT_JOY_Y
        LEFT_FEEDER_AXIS_X: Input = ControllerMap.LEFT_JOY_X
        LEFT_FEEDER_AXIS_Y: Input = ControllerMap.LEFT_JOY_Y

        LAUNCHER_BUTTON: Input = ControllerMap.RIGHT_TRIGGER
        INTAKE_BUTTON: Input = ControllerMap.LEFT_TRIGGER
        RETRACT_INTAKE_BUTTON: Input = ControllerMap.Y_BUTTON
        EJECT_BUTTON: Input = ControllerMap.B_BUTTON

        CLIMBER_EXTEND_BUTTON: Input = ControllerMap.DPAD_UP
        CLIMBER_RETRACT_BUTTON: Input = ControllerMap.DPAD_DOWN

        IGNORE_VISION_BUTTON: Input = ControllerMap.BACK_BUTTON

        # Simple Teleop Inputs
        SIMPLE_INTAKE_BUTTON: Input = ControllerMap.LEFT_TRIGGER
        SIMPLE_RETRACT_INTAKE_BUTTON: Input = ControllerMap.Y_BUTTON

        SIMPLE_CLIMBER_EXTEND_BUTTON: Input = ControllerMap.DPAD_UP
        SIMPLE_CLIMBER_RETRACT_BUTTON: Input = ControllerMap.DPAD_DOWN

        SIMPLE_FEED_BUTTON: Input = ControllerMap.X_BUTTON
        SIMPLE_EJECT_BUTTON: Input = ControllerMap.B_BUTTON

        SIMPLE_FLYWHEEL_AXIS: Input = ControllerMap.RIGHT_TRIGGER
        FLYWHEEL_RPM_BUTTON: Input = ControllerMap.RIGHT_BUMPER

        SIMPLE_TURRET_AXIS: Input = ControllerMap.LEFT_JOY_X

    class TestConstants1:
        """
        Creates button inputs for the test controller 1, which handles the flywheel, indexer, turret, and climber
        """
        CONTROLLER_PORT: int = 0
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0
        
        FLYWHEEL_INPUT: Input = controller.RIGHT_TRIGGER
        INDEXER_INPUT: Input = controller.LEFT_TRIGGER
        TURRET_INPUT: Input = controller.LEFT_JOY_X
        CLIMBER_INPUT: Input = controller.RIGHT_JOY_X

        POWER_LIMIT: float = 0.4
    
    class TestConstants2:
        """
        Creates button inputs for the test controller 2, which handles the intake and feeder
        """
        CONTROLLER_PORT: int = 1
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        INTAKE_ROLLER_INPUT: Input = controller.RIGHT_TRIGGER
        FEEDER_INPUT: Input = controller.LEFT_JOY_Y
        INTAKE_PIVOT_INPUT: Input = controller.RIGHT_JOY_Y

        POWER_LIMIT: float = 0.4
    
    class DemoController:
        """
        Creates button inputs for the demo controller, for testing the robot without usual teleop automation
        """
        CONTROLLER_PORT: int = 0
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        FLYWHEEL_LEFT_INPUT: Input = controller.LEFT_TRIGGER
        FLYWHEEL_RIGHT_INPUT: Input = controller.RIGHT_TRIGGER

        TURRET_LEFT: Input = controller.LEFT_BUMPER
        TURRET_RIGHT: Input = controller.RIGHT_BUMPER
        FEED_INPUT: Input = controller.X_BUTTON
        EJECT_FEEDER: Input = controller.B_BUTTON
        INTAKE_INPUT: Input = controller.A_BUTTON
        RETRACT_INTAKE: Input = controller.Y_BUTTON

        CLIMB_EXTEND: Input = controller.START_BUTTON
        CLIMB_RETRACT: Input = controller.BACK_BUTTON
        
        THROTTLE_INPUT: Input = controller.LEFT_JOY_Y
        STRAFE_INPUT: Input = controller.LEFT_JOY_X
        ROTATE_INPUT: Input = controller.RIGHT_JOY_X

        JOG_UP_BUTTON: Input = ControllerMap.DPAD_UP
        JOG_DOWN_BUTTON: Input = ControllerMap.DPAD_DOWN
        JOG_LEFT_BUTTON: Input = ControllerMap.DPAD_LEFT
        JOG_RIGHT_BUTTON: Input = ControllerMap.DPAD_RIGHT

        RESET_HEADING_BUTTON: Input = ControllerMap.RIGHT_STICK_BUTTON

    class SysidController:
        """
        Creates button inputs for the sysid controller, for running various sysid routines
        """
        CONTROLLER_PORT: int = 0
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        QUASI_FWD_BUTTON: Input = XboxControllerMap.A_BUTTON
        QUASI_REV_BUTTON: Input = XboxControllerMap.B_BUTTON
        DYNAMIC_FWD_BUTTON: Input = XboxControllerMap.X_BUTTON
        DYNAMIC_REV_BUTTON: Input = XboxControllerMap.Y_BUTTON
