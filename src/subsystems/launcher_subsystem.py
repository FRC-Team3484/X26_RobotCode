from enum import Enum

from commands2 import Subsystem
from wpimath.filter import Debouncer

from src.datatypes import FeederSpeed, TargetType, LauncherTarget
from src.constants import IndexerSubsystemConstants, FeederSubsystemConstants, LauncherSubsystemConstants
from src.subsystems.feeder_subsystem import FeederSubsystem
from src.subsystems.flywheel_subsystem import FlywheelSubsystem
from src.subsystems.turret_subsystem import TurretSubsystem
from src.subsystems.indexer_subsystem import IndexerSubsystem
from src.subsystems.feed_target_subsystem import FeedTargetSubsystem


class LauncherStates(Enum):
    REST = 0
    TRACK = 1
    PREPARE = 2
    FIRE = 3

class LauncherSubsystem(Subsystem):
    """
    Coordinates all the subsystems to launch game pieces

    The FeedTargetSubsystem handles the math for pointing the turret (or drivetrain)
        at the target, and powering the flywheel to the correct speed

    Parameters:
        - feeder (`FeederSubsystem`): the feeder subsystem
        - indexer (`IndexerSubsystem`): the indexer subsystem
        - flywheel (`FlywheelSubsystem`): the flywheel subsystem
        - turret (`TurretSubsystem`): the turret subsystem
        - feed_targets (`FeedTargetSubsystem`): the feed target subsystem
    """
    def __init__(self, feeder: FeederSubsystem | None, indexer: IndexerSubsystem | None, flywheel: FlywheelSubsystem, turret: TurretSubsystem, feed_targets: FeedTargetSubsystem) -> None:
        super().__init__()
        self.feeder: FeederSubsystem | None = feeder
        self.flywheel: FlywheelSubsystem = flywheel
        self.turret: TurretSubsystem = turret
        self.feed_targets: FeedTargetSubsystem = feed_targets
        self.indexer: IndexerSubsystem | None = indexer

        self._debounce_timer: Debouncer = Debouncer(LauncherSubsystemConstants.DEBOUNCE_TIMER, Debouncer.DebounceType.kRising)

        self.state: LauncherStates = LauncherStates.REST
        self._target_type: TargetType = TargetType.NONE
        self.stop()

    def periodic(self) -> None:
        """
        Once a target is set, the turret will track the target, 
            and the flywheel will spin to the correct speed before firing
        """
        if self._target_type == TargetType.NONE:
            return
        target: LauncherTarget = self.feed_targets.get_target(self._target_type)
        
        match self.state:
            case LauncherStates.REST:
                pass
            case LauncherStates.TRACK:
                self.turret.aim(target.turret_target)
            case LauncherStates.PREPARE:
                self.turret.aim(target.turret_target)
                self.flywheel.set_speed(target.flywheel_speed)

                if self._debounce_timer.calculate(self.flywheel.is_at_speed()) and self.turret.at_target():
                    self.state = LauncherStates.FIRE
            case LauncherStates.FIRE:
                self.turret.aim(target.turret_target)
                self.flywheel.set_speed(target.flywheel_speed)
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
        self.turret.set_power(0)
        
        self.state = LauncherStates.REST

    def getSubsystems(self) -> list[Subsystem]:
        """
        Returns a list of all the subsystems in the launcher
        """
        subsystems = [self.flywheel, self.turret]
        if self.feeder is not None:
            subsystems.append(self.feeder)
        if self.indexer is not None:
            subsystems.append(self.indexer)
        return subsystems

    def set_feeder_speed(self, speed: FeederSpeed) -> None:
        """
        Sets the speed of the feeder

        Parameters:
            - speed (`FeederSpeed`): the speed to set the feeder to
        """
        if self.feeder and self.state != LauncherStates.FIRE:
            self.feeder.set_velocity(speed)

