from enum import Enum

from commands2 import Subsystem
from wpimath.filter import Debouncer

from src.datatypes import TargetType, LauncherTarget
from src.constants import LauncherSubsystemConstants, IndexerSubsystemConstants, FeederSubsystemConstants
from src.subsystems.feeder_subsystem import FeederSubsystem
from src.subsystems.flywheel_subsystem import FlywheelSubsystem
from src.subsystems.indexer_subsystem import IndexerSubsystem
from src.subsystems.feed_target_subsystem import FeedTargetSubsystem


class LauncherStates(Enum):
    REST = 0
    TRACK = 1
    PREPARE = 2
    FIRE = 3

class TurretlessLauncherSubsystem(Subsystem):
    """
    Coordinates all the subsystems to launch game pieces

    The FeedTargetSubsystem handles the math for pointing the turret (or drivetrain)
        at the target, and powering the flywheel to the correct speed

    This version does not use the turret, in case of a mechanical failure. The TeleopTurretlessDriveSlowCommand 
        is responsible for aiming the drivetrain to point at the target

    Parameters:
        - feeder (`FeederSubsystem`): the feeder subsystem
        - indexer (`IndexerSubsystem`): the indexer subsystem
        - flywheel (`FlywheelSubsystem`): the flywheel subsystem
        - feed_targets (`FeedTargetSubsystem`): the feed target subsystem
    """
    def __init__(self, feeder: FeederSubsystem | None, indexer: IndexerSubsystem | None, flywheel: FlywheelSubsystem, feed_targets: FeedTargetSubsystem) -> None:
        super().__init__()
        self.feeder: FeederSubsystem | None = feeder
        self.flywheel: FlywheelSubsystem = flywheel
        self.feed_targets: FeedTargetSubsystem = feed_targets
        self.indexer: IndexerSubsystem | None = indexer

        self.state: LauncherStates = LauncherStates.REST
        self._target_type: TargetType = TargetType.NONE

        self._target: LauncherTarget | None = None

        self.stop()

    @property
    def target(self) -> LauncherTarget | None:
        return self._target

    def periodic(self) -> None:
        """
        Once a target is set, the turret will track the target, 
            and the flywheel will spin to the correct speed before firing
        """
        if self._target_type == TargetType.NONE:
            return
        self._target = self.feed_targets.get_target(self._target_type)
        
        match self.state:
            case LauncherStates.REST:
                pass
            case LauncherStates.TRACK:
                pass
            case LauncherStates.PREPARE:
                self.flywheel.set_speed(self._target.flywheel_speed)

                if self.flywheel.is_at_speed():
                    self.state = LauncherStates.FIRE
            case LauncherStates.FIRE:
                self.flywheel.set_speed(self._target.flywheel_speed)
                self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER) if self.indexer != None else None
                self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED) if self.feeder != None else None

    def aim_at(self, target_type: TargetType) -> None:
        """
        Sets the target type, and sets the state to track

        This tells the launcher to begin aiming at the given target

        Parameters:
            - target_type (`TargetType`): the target type
        """
        self._target_type = target_type
        self.state = LauncherStates.TRACK
    
    def fire_at(self, target_type: TargetType) -> None:
        """
        Sets the target type, and sets the state to prepare

        This tells the launcher to begin preparing to fire at the given target

        Parameters:
            - target_type (`TargetType`): the target type
        """
        self._target_type = target_type
        if self.state != LauncherStates.FIRE:
            self.state = LauncherStates.PREPARE
    
    def stop(self) -> None:
        """
        Stops the launcher
        """
        self._target_type = TargetType.NONE
        if self.feeder:
            self.feeder.set_velocity(FeederSubsystemConstants.STOP_VELOCITY)
        if self.indexer:
            self.indexer.set_power(IndexerSubsystemConstants.STOP_POWER)
        self.flywheel.set_power(0)
        
        self.state = LauncherStates.REST