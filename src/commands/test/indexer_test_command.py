from commands2 import Command

from subsystems.indexer_subsystem import IndexerSubsystem
from oi import TestInterface

class IndexerTestCommand(Command):
    def __init__(self, indexer_subsystem: IndexerSubsystem, oi: TestInterface) -> None:
        super().__init__()
        self._oi = oi
        self._indexer_subsystem = indexer_subsystem

    def execute(self) -> None:
        self._indexer_subsystem.set_power(self._oi.get_indexer())
    
    def end(self, interrupted: bool) -> None:
        self._indexer_subsystem.set_power(0)
    
    def isFinished(self) -> bool:
        return False