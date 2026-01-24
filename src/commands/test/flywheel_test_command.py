from commands2 import Command
from subsystems.flywheel_subsystem import  FlywheelSubsystem

from oi import TestInterface

class FlywheelTestCommand(Command):
    def __init__(self, flywheel: FlywheelSubsystem, oi: TestInterface):
        """
        A command for testing the flywheel subsystem

        Parameters:
         - flywheel (`FlywheelSubsystem`): the flywheel subsystem
         - oi (`oi.TestInterface`): the oi test interface for controller bindings
        """
        
        super().__init__()

        self._flywheel = flywheel
        self._oi = oi
        self.addRequirements(flywheel)
        
    def initialize(self):
        pass

    def execute(self) -> None:
        self._flywheel.set_power(self._oi.get_wheel_power())
        self._flywheel.print_diagnostics()
    
    def end(self, interrupted: bool):
        self._flywheel.set_power(0)
    
    def isFinished(self) -> bool:
        return False