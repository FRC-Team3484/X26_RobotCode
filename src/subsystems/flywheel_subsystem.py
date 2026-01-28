from frc3484.motion import VelocityMotor, SC_LauncherSpeed
from constants import FlywheelSubsystemConstants
from commands2 import Subsystem
# from wpimath.units import revolutions_per_minute

class FlywheelSubsystem(Subsystem):
    def __init__(self) -> None:
        
        super().__init__()
        
        self._motor: VelocityMotor = VelocityMotor(
            FlywheelSubsystemConstants.motor_config,
            FlywheelSubsystemConstants.pid_config,
            FlywheelSubsystemConstants.feed_forward_config,
            FlywheelSubsystemConstants.gear_ratio,
            FlywheelSubsystemConstants.tolerance
        )

    def periodic(self) -> None:
        pass

    def set_speed(self, speed: SC_LauncherSpeed) -> None:
        '''
        Sets the motor speed

        Parameters:
            - speed (SC_LauncherSpeed): the speed and power to set the motor to
        '''
        self._motor.set_speed(speed)

    def set_power(self, power: float) -> None:
        self._motor.set_power(power)

    def is_at_speed(self) -> bool:
        '''
        Checks whether the motor is at the target speed

        Returns:
            - bool: `True` if the motor is at the target speed, `False` otherwise
        '''
        return self._motor.at_target_speed()

    def print_diagnostics(self) -> None:
        self._motor.print_diagnostics()

r'''
this is a very important comment dont delete
      ________
     /       /\       ______ cube™ :OO
    /       /..\     /
   /_______/....\ <-/
   \#######\..../ 
    \#######\../ 
     \#######\/

(c) SOUPOFFICE.AI™: always and forever  
'''
