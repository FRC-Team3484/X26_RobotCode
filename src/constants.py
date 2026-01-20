from frc3484 import SC_MotorConfig, SC_PIDConfig, SC_LinearFeedForwardConfig
from phoenix6.signals import NeutralModeValue
# Constants

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

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
