from typing import override

from commands2 import Command

from subsystems.intake_subsystem import IntakeSubsystem
from oi import TestInterface2

class IntakeTestCommand(Command):
    """
    Intake test command

    Controls the intake based on controller inputs

    Parameters:
        - oi (`oi.TestInterface`): the oi test interface for controller bindings
        - intake (`IntakeSubsystem`): the intake subsystem
    """
    def __init__(self, oi: TestInterface2, intake: IntakeSubsystem) -> None:
        super().__init__()
        self._intake: IntakeSubsystem = intake
        self._oi: TestInterface2 = oi
        self.addRequirements(intake)

    @override
    def execute(self) -> None:
        self._intake.set_pivot_power(self._oi.get_intake_pivot())
        self._intake.set_roller_power(self._oi.get_intake_roller())
        self._intake.print_diagnostics()

    @override
    def end(self, interrupted: bool) -> None:
        self._intake.set_pivot_power(0)
        self._intake.set_roller_power(0)

    @override
    def isFinished(self) -> bool:
        return False