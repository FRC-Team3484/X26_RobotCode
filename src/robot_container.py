from subsystems.indexer_subsystem import IndexerSubsystem

import config

class RobotContainer:
    def __init__(self) -> None:
        
        # Subsystems
        if config.INDEXER_ENABLED:
            self._indexer_subsystem: IndexerSubsystem = IndexerSubsystem()