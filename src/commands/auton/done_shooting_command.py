from typing import override
from commands2 import Command
from wpilib import Timer

from constants import DoneShootingCommandConstants
from subsystems.feeder_subsystem import FeederSubsystem

class DoneShootingCommand(Command):
    """
    A command that ends when the feeder has a piece
    """
    def __init__(self, feeder_subsystem: FeederSubsystem) -> None:
        super().__init__()
        self._feeder_subsystem: FeederSubsystem = feeder_subsystem
        self._timer: Timer = Timer()
        self._timer.start()

    @override
    def execute(self) -> None:
        if self._feeder_subsystem.has_piece():
            self._timer.reset()

    @override
    def end(self, interrupted: bool):
        self._timer.stop()
        self._timer.reset()

    @override
    def isFinished(self) -> bool:
        return self._timer.hasElapsed(DoneShootingCommandConstants.TIMEOUT)