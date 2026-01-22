# Constants
from frc3484.datatypes import SC_MotorConfig, SC_AngularFeedForwardConfig, SC_PIDConfig, SC_TrapezoidConfig
# Subsystems

class AgitatorSubsystemConstants:
    pass

class ClimberSubsystemConstants:
    pass

class IntakeSubsystemConstants:
    INTAKE_POWER: float = 0.5
    MOTOR_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 1
    )
    MOTOR_DEPLOY_CONFIG: SC_MotorConfig = SC_MotorConfig(
        can_id = 2
    )
    PID_CONFIG: SC_PIDConfig = SC_PIDConfig(
        
    )
    FEED_FORWARD_CONFIG: SC_AngularFeedForwardConfig = SC_AngularFeedForwardConfig(

    )
    TRAPEZOID_CONFIG: SC_TrapezoidConfig = SC_TrapezoidConfig(

    )
    ANGLE_TOLERANCE: float = 0.01
    GEAR_RATIO: float = 1
class TurretSubsystemConstants:
    pass

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
