from phoenix6.signals import NeutralModeValue

from frc3484.motion import SC_MotorConfig, SC_AngularFeedForwardConfig, SC_PIDConfig

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

class LauncherSubsystemConstants:
    pass
