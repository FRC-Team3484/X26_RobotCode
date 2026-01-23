# Constants
from frc3484.datatypes import SC_MotorConfig, SC_AngularFeedForwardConfig, SC_PIDConfig, SC_TrapezoidConfig 
from wpimath.units import degrees, turns 
# Subsystems

class AgitatorSubsystemConstants:
    pass

class ClimberSubsystemConstants:
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
    pass

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
