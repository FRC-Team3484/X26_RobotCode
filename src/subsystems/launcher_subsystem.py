from enum import Enum

from commands2 import Subsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d

from constants import LauncherSubsystemConstants
from constants import IndexerSubsystemConstants
from frc3484.pose_manipulation import apply_offset_to_pose

from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.turret_subsystem import TurretSubsystem
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from subsystems.indexer_subsystem import IndexerSubsystem

class LauncherStates(Enum):
    REST = 0
    TRACK = 1
    PREPARE = 2
    FIRE = 3

class LauncherSubsystem(Subsystem):
    def __init__(self, feeder: FeederSubsystem | None, flywheel: FlywheelSubsystem, turret: TurretSubsystem, drivetrain: DrivetrainSubsystem, indexer: IndexerSubsystem|None) -> None:
        super().__init__()
        self.feeder: FeederSubsystem | None = feeder
        self.flywheel: FlywheelSubsystem = flywheel
        self.turret: TurretSubsystem = turret
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.indexer: IndexerSubsystem | None = indexer
        self.states = LauncherStates
        self.state: LauncherStates
        self.stop()

    def _get_turret_pose(self) -> Pose2d:
        robot_pose: Pose2d = self.drivetrain.get_pose()
        return apply_offset_to_pose(robot_pose, LauncherSubsystemConstants.TURRET_OFFSET)

    def target_translation(self, target_translation: Translation2d) -> Translation2d:
        turret_pose: Pose2d = self._get_turret_pose()
        turret_translation: Translation2d = Translation2d(turret_pose.x, turret_pose.y)
        turret_rotation: Rotation2d = turret_pose.rotation()

        difference: Translation2d = target_translation - turret_translation

        difference.rotateBy(-turret_rotation)

        return difference

    def periodic(self) -> None:
        match self.state:
            case self.states.REST:
                pass
            case self.states.TRACK:
                pass
            case self.states.PREPARE:
                pass
            case self.states.FIRE:
                pass
    
    def stop(self) -> None:
        self.feeder.set_power(0) if self.feeder != None else None
        self.indexer.set_power(IndexerSubsystemConstants.STOP_POWER) if self.indexer != None else None
        self.flywheel.set_power(0)
        self.turret.set_power(0)
        
        self.state = self.states.REST

