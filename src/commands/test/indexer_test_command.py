from typing import override

from commands2 import Command
from subsystems.indexer_subsystem import IndexerSubsystem

from oi import TestInterface1

class IndexerTestCommand(Command):
    """
    a command for testing the indexer subsystem

    Parameters:
        - indexer_subsystem (`IndexerSubsystem`): the indexer subsystem
        - oi (`oi.TestInterface`): the oi test interface for controller bindings
    """
    def __init__(self, oi: TestInterface1, indexer_subsystem: IndexerSubsystem) -> None:
        super().__init__()
        self._oi: TestInterface1 = oi
        self._indexer_subsystem: IndexerSubsystem = indexer_subsystem

    @override
    def execute(self) -> None:
        self._indexer_subsystem.set_power(self._oi.get_indexer())
    
    @override
    def end(self, interrupted: bool) -> None:
        self._indexer_subsystem.set_power(0)
    
    @override
    def isFinished(self) -> bool:
        return False