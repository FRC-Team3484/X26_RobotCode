from frc3484.motion import SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_TrapezoidConfig
from phoenix6.signals import NeutralModeValue
from wpimath.units import volts, volt_seconds_per_radian, volt_seconds_squared_per_radian, feet_per_second, feet_per_second_squared

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
        G= volts,
        S= volts,
        V= volt_seconds_per_radian,
        A= volt_seconds_squared_per_radian
    )
    TRAPEZOID_CONFIG = SC_TrapezoidConfig (
        max_velocity= feet_per_second(0),
        max_acceleration= feet_per_second_squared(0),
        max_jerk= 0
    )


class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
