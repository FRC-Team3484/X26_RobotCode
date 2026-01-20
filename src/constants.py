from frc3484.motion import SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_TrapezoidConfig
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
    TRAPEZOID_CONFIG = SC_TrapezoidConfig (
        max_velocity= 0,
        max_acceleration= 0,
        max_jerk= 0
    )
    GEAR_RATIO: float = 1.0
    HOME_POWER: float = -0.1
    HOME_SENSOR_POSITION: degrees = 0
    MINIMUM_ANGLE: degrees = HOME_SENSOR_POSITION
    MAXIMUM_ANGLE: degrees = 180
    AIM_TOLERANCE: inches = 6

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
