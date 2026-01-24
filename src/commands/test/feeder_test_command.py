from typing import override
from commands2 import Command

from subsystems.feeder_subsystem import FeederSubsystem
from oi import TestInterface

class FeederTestCommand(Command):
    """
    The test command for the feeder

    Based on joystick input, will set the power of the feeder

    Parameters:
        - test_interface (`TestInterface`): the test interface
        - feeder_subsystem (`FeederSubsystem`): the feeder subsystem
    """
    def __init__(self, test_interface: TestInterface, feeder_subsystem: FeederSubsystem) -> None:
        super().__init__()
        self._test_interface: TestInterface = test_interface
        self._feeder_subsystem: FeederSubsystem = feeder_subsystem

    @override
    def execute(self) -> None:
        self._feeder_subsystem.set_power(self._test_interface.get_feeder())

    @override
    def end(self, interrupted: bool) -> None:
        self._feeder_subsystem.set_power(0)

    @override
    def isFinished(self) -> bool:
        return False