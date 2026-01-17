from frc3484.motion import AngularPositionMotor, SC_MotorConfig, SC_PIDConfig, SC_AngularFeedForwardConfig, SC_TrapezoidConfig 
from commands2 import subsystem
from wpimath.units import degrees

class TurretSubsystem(subsystem) :
    def __init__(self, motor_config: SC_MotorConfig, pid_config: SC_PIDConfig, feed_forward_config: SC_AngularFeedForwardConfig, trapezoid_config: SC_TrapezoidConfig, angle_tolerance: degrees, gear_ratio: float = 1.0, external_encoder: None = None)-> None:
        self._motor = AngularPositionMotor(motor_config, pid_config, feed_forward_config, trapezoid_config, angle_tolerance, gear_ratio, external_encoder)

