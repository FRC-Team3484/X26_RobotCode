from typing import override
from commands2 import Command

from subsystems.climber_subsystem import ClimberSubsystem
from oi import TestInterface1

class ClimberTestCommand(Command):
    """
    a command for testing the climber subsystem

    Parameters:
        - climber_subsystem (`ClimberSubsystem`): the climber subsystem
        - oi (`oi.TestInterface1`): the oi test interface for controller bindings
    """
    def __init__(self, oi: TestInterface1, climber_subsystem: ClimberSubsystem) -> None:
        super().__init__()
        self._oi: TestInterface1 = oi
        self._climber_subsystem: ClimberSubsystem = climber_subsystem

    @override
    def execute(self) -> None:
        self._climber_subsystem.set_power(self._oi.get_climber())

    @override
    def end(self, interrupted: bool) -> None:
        self._climber_subsystem.set_power(0)

    @override
    def isFinished(self) -> bool:
        return False