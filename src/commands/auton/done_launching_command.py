from typing import override

from wpilib import Timer
from commands2 import Command

from src.constants import DoneLaunchingCommandConstants
from src.subsystems.feeder_subsystem import FeederSubsystem

class DoneLaunchingCommand(Command):
    """
    A command that ends when the feeder has a piece

    Parameters:
        - feeder_subsystem (`FeederSubsystem`): the feeder subsystem
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
        return self._timer.hasElapsed(DoneLaunchingCommandConstants.TIMEOUT)