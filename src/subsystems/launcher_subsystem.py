from enum import Enum

from commands2 import Subsystem
from wpimath.filter import Debouncer

from src.datatypes import TargetType
from src.constants import LauncherSubsystemConstants, IndexerSubsystemConstants, FeederSubsystemConstants
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

        if self._target_type == TargetType.NONE:
            return
        target = self.feed_targets.get_target(self._target_type)
        match self.state:
            case LauncherStates.REST:
                pass
            case LauncherStates.TRACK:
                self.turret.aim(target.turret_target)
            case LauncherStates.PREPARE:
                self.turret.aim(target.turret_target)
                self.flywheel.set_speed(target.flywheel_speed)
                # if self._debounce_timer.calculate(self.flywheel.is_at_speed()):
                #     self.state = LauncherStates.FIRE

                if self.flywheel.is_at_speed():
                    self.state = LauncherStates.FIRE
            case LauncherStates.FIRE:
                self.turret.aim(target.turret_target)
                self.flywheel.set_speed(target.flywheel_speed)
                self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER) if self.indexer != None else None
                self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED) if self.feeder != None else None

    def aim_at(self, target_type: TargetType) -> None:
        self._target_type = target_type
        self.state = LauncherStates.TRACK
    
    def fire_at(self, target_type: TargetType):
        self._target_type = target_type
        if self.state != LauncherStates.FIRE:
            self.state = LauncherStates.PREPARE
    
    def stop(self) -> None:
        self._target_type = TargetType.NONE
        if self.feeder:
            self.feeder.set_velocity(FeederSubsystemConstants.STOP_VELOCITY)
        if self.indexer:
            self.indexer.set_power(IndexerSubsystemConstants.STOP_POWER)
        self.flywheel.set_power(0)
        self.turret.set_power(0)
        
        self.state = LauncherStates.REST