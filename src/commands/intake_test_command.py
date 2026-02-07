from commands2 import Command
from subsystems.intake_subsystem import IntakeSubsystem
from oi import TestInterface2
class IntakeTestCommand(Command):
    def __init__(self, intake: IntakeSubsystem, oi: TestInterface2):
        super().__init__()
        self._intake = intake
        self._oi = oi
        self.addRequirements(intake)

    def initialize(self):
        pass

    def execute(self) -> None:
        self._intake.set_pivot_power(self._oi.get_intake_pivot())
        self._intake.set_roller_power(self._oi.get_intake_roller())
        self._intake.print_diagnostics()

    def end(self, interrupted: bool) -> None:
        self._intake.set_pivot_power(0)
        self._intake.set_roller_power(0)

    def isFinished(self) -> bool:
        return False

    