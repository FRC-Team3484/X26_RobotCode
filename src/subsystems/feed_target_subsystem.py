from commands2 import Subsystem
from wpilib import Field2d

class FeedTargetSubsystem(Subsystem):
    def __init__(self, field: Field2d) -> None:
        super().__init__()

        self._field: Field2d = field