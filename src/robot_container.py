from subsystems.indexer_subsystem import IndexerSubsystem

class RobotContainer:
    def __init__(self) -> None:
        
        # Subsystems
        self._indexer_subsystem: IndexerSubsystem = IndexerSubsystem()