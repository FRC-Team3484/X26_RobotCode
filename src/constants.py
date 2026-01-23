from frc3484.motion import SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_ExpoConfig
from phoenix6.signals import NeutralModeValue
from wpimath.units import degrees, inches


# Constants

# Subsystems
class AgitatorSubsystemConstants:
    pass

class ClimberSubsystemConstants:
    pass

class IntakeSubsystemConstants:
    pass

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
    GEAR_RATIO: float = 1.0
    ENCODER_A_CAN_ID: int = 0
    ENCODER_A_CAN_BUS_NAME: str = "rio"
    ENCODER_B_CAN_ID: int = 0
    ENCODER_B_CAN_BUS_NAME: str = "rio"
    HOME_SENSOR_PORT: int = 1

    HOME_POWER: float = -0.1
    MINIMUM_ANGLE: degrees = -360
    MAXIMUM_ANGLE: degrees = 360
    AIM_TOLERANCE: inches = 6
    
    ENCODER_CPR = 4096
    LOOPING_MOVE_THRESH_REV: float = 0.25
    ABS_MATH_TOL_TEETH: float = 0.5
    ABS_CORRECTION_ENABLE: True
    ABS_CORRECTION_MAX_JUMP_REV: float = 0.05
    TEETH_A: int = 20
    TEETH_B: int = 21
    TEETH_TURRET: int = 200
    ABS_OFFSET_A_REV: float = 0
    ABS_OFFSET_B_REV: float = 0

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
from frc3484.motion import SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_ExpoConfig
from phoenix6.signals import NeutralModeValue
from wpimath.units import degrees, inches


# Constants

# Subsystems
class AgitatorSubsystemConstants:
    pass

class ClimberSubsystemConstants:
    pass

class IntakeSubsystemConstants:
    pass

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
    GEAR_RATIO: float = 1.0
    ENCODER_A_CAN_ID: int = 0
    ENCODER_A_CAN_BUS_NAME: str = "rio"
    ENCODER_B_CAN_ID: int = 0
    ENCODER_B_CAN_BUS_NAME: str = "rio"
    HOME_SENSOR_PORT: int = 1

    HOME_POWER: float = -0.1
    MINIMUM_ANGLE: degrees = -360
    MAXIMUM_ANGLE: degrees = 360
    AIM_TOLERANCE: inches = 6
    
    ENCODER_CPR = 4096
    LOOPING_MOVE_THRESH_REV: float = 0.25
    ABS_MATH_TOL_ENABLE: float = 0.5
    ABS_CORRECTION_ENABLE: True
    ABS_CORRECTION_MAX_JUMP_REV: float = 0.05
    TEETH_A: int = 20
    TEETH_B: int = 21
    TEETH_TURRET: int = 200
    ABS_OFFSET_A_REV: float = 0
    ABS_OFFSET_B_REV: float = 0

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
