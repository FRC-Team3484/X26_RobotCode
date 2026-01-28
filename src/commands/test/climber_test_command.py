from commands2 import Command

from subsystems.climber_subsystem import ClimberSubsystem
from oi import TestInterface

class ClimberTestCommand(Command):
    def __init__(self, climber_subsystem: ClimberSubsystem, oi: TestInterface) -> None:
        super().__init__()
        self._oi: TestInterface = oi
        self._climber_subsystem: ClimberSubsystem = climber_subsystem

    def execute(self) -> None:
        self._climber_subsystem.set_power(self._oi.get_climber())

    def end(self, interrupted: bool) -> None:
        self._climber_subsystem.set_power(0)

    def isFinished(self) -> bool:
        return False