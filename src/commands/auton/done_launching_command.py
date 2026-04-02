from typing import override

from wpilib import Timer
from commands2 import Command

from src.constants import DoneLaunchingCommandConstants
from src.subsystems.feeder_subsystem import FeederSubsystem

class DoneLaunchingCommand(Command):
    """
    A command that ends when the feeder no longer has a piece

    Parameters:
        - feeder_subsystem (`FeederSubsystem`): the feeder subsystem
    """
    def __init__(self, feeder_subsystem: FeederSubsystem) -> None:
        super().__init__()
        self._feeder_subsystem: FeederSubsystem = feeder_subsystem
        self._timer: Timer = Timer()
        self._has_had_piece: bool = False

    @override
    def initialize(self):
        self._timer.reset()
        self._timer.start()
        self._has_had_piece = False

    @override
    def execute(self) -> None:
        if self._feeder_subsystem.has_piece():
            self._has_had_piece = True
            self._timer.reset()

    @override
    def end(self, interrupted: bool):
        self._timer.stop()
        self._timer.reset()

    @override
    def isFinished(self) -> bool:
        return \
            (self._has_had_piece and self._timer.hasElapsed(DoneLaunchingCommandConstants.TIMEOUT)) or \
            self._timer.hasElapsed(DoneLaunchingCommandConstants.NO_LAUNCH_TIMEOUT)