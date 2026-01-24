from typing import override
from commands2 import Command
from subsystems.flywheel_subsystem import  FlywheelSubsystem

from oi import TestInterface

class FlywheelTestCommand(Command):
    def __init__(self, test_interface: TestInterface, flywheel: FlywheelSubsystem) -> None:
        """
        A command for testing the flywheel subsystem

        Parameters:
            - oi (`oi.TestInterface`): the oi test interface for controller bindings
            - flywheel (`FlywheelSubsystem`): the flywheel subsystem
        """
        
        super().__init__()

        self._flywheel: FlywheelSubsystem = flywheel
        self._oi: TestInterface = test_interface
        self.addRequirements(flywheel)
        
    @override
    def execute(self) -> None:
        self._flywheel.set_power(self._oi.get_wheel_power())
        self._flywheel.print_diagnostics()
    
    @override
    def end(self, interrupted: bool) -> None:
        self._flywheel.set_power(0)
    
    @override
    def isFinished(self) -> bool:
        return False