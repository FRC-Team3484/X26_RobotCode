from frc3484.motion import SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_ExpoConfig
from phoenix6.signals import NeutralModeValue
from wpimath.units import degrees, inches, turns


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
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass