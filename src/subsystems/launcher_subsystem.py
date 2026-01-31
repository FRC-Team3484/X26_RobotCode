from enum import Enum
from typing import Literal
import numpy as np

from commands2 import Subsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.units import metersToInches

from constants import LauncherSubsystemConstants, IndexerSubsystemConstants, FeederSubsystemConstants

from frc3484.pose_manipulation import apply_offset_to_pose
from frc3484.datatypes import SC_LauncherSpeed, SC_ApriltagTarget

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
    def __init__(self, feeder: FeederSubsystem | None, indexer: IndexerSubsystem | None, flywheel: FlywheelSubsystem, turret: TurretSubsystem, drivetrain: DrivetrainSubsystem) -> None:
        super().__init__()
        self.feeder: FeederSubsystem | None = feeder
        self.flywheel: FlywheelSubsystem = flywheel
        self.turret: TurretSubsystem = turret
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.indexer: IndexerSubsystem | None = indexer
        self.states = LauncherStates
        self.state: LauncherStates
        self._target: Translation2d
        self._target_type: Literal["hub", "feed"]
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
                self.turret.aim(self.target_translation(self._target))
            case self.states.PREPARE:
                self._set_turret_and_flywheel()
                if self.turret.at_target() and self.flywheel.is_at_speed():
                    self.state = self.states.FIRE
            case self.states.FIRE:
                self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER) if self.indexer != None else None
                self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED) if self.feeder != None else None
    
    def _set_turret_and_flywheel(self) -> None:
        target_translation: Translation2d = self.target_translation(self._target)
        self.turret.aim(target_translation)
        self.flywheel.set_speed(
            SC_LauncherSpeed(
                np.interp(
                    float(metersToInches(target_translation.norm())),
                    LauncherSubsystemConstants.FEED_DISTANCES if self._target_type == "feed" else LauncherSubsystemConstants.HUB_DISTANCES,
                    LauncherSubsystemConstants.FEED_RPM if self._target_type == "feed" else LauncherSubsystemConstants.HUB_RPM
                ),
                0.0
            )
        )

    def aim_at(self, target: Translation2d, target_type: Literal["hub", "feed"]):
        self._target = target
        self._target_type = target_type

        self.state = self.states.TRACK
    
    def fire_at(self, target: Translation2d, target_type: Literal["hub", "feed"]):
        self._target = target
        self._target_type = target_type

        self.state = self.states.PREPARE
    
    def stop(self) -> None:
        self.feeder.set_power(0) if self.feeder != None else None
        self.indexer.set_power(IndexerSubsystemConstants.STOP_POWER) if self.indexer != None else None
        self.flywheel.set_power(0)
        self.turret.set_power(0)
        
        self.state = self.states.REST