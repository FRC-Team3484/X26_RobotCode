from enum import Enum
from typing import Literal
import numpy as np

from commands2 import Subsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import metersToInches, seconds

from constants import LauncherSubsystemConstants, IndexerSubsystemConstants, FeederSubsystemConstants, TurretSubsystemConstants

# from frc3484.pose_manipulation import apply_offset_to_pose
from frc3484.datatypes import SC_LauncherSpeed

from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from subsystems.indexer_subsystem import IndexerSubsystem

class LauncherStates(Enum):
    REST = 0
    TRACK = 1
    PREPARE = 2
    FIRE = 3

class TurretlessLauncherSubsystem(Subsystem):
    def __init__(self, feeder: FeederSubsystem | None, indexer: IndexerSubsystem | None, flywheel: FlywheelSubsystem, drivetrain: DrivetrainSubsystem) -> None:
        super().__init__()
        self.feeder: FeederSubsystem | None = feeder
        self.flywheel: FlywheelSubsystem = flywheel
        
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.indexer: IndexerSubsystem | None = indexer
        self.states = LauncherStates
        self.state: LauncherStates
        self._target: Translation2d
        self._target_type: Literal["hub", "feed"]
        self.dist_array: np.ndarray
        self.flight_time_array: np.ndarray
        self.rpm_array: np.ndarray
        
        self._turret_to_target: Translation2d
        self.stop()

    @property
    def turret_to_target(self) -> Translation2d:
        return self._turret_to_target

    def _get_turret_position(self) -> Pose2d:
        robot_pose: Pose2d = self.drivetrain.get_pose()
        return Pose2d(
            robot_pose.translation() + LauncherSubsystemConstants.TURRET_OFFSET.translation().rotateBy(robot_pose.rotation()),
            robot_pose.rotation() + LauncherSubsystemConstants.TURRET_OFFSET.rotation()
        )

    def _get_turret_velocity(self) -> ChassisSpeeds:
        robot_velocity: ChassisSpeeds = self.drivetrain.get_velocity()
        turret_position: Pose2d = self._get_turret_position()
        turret_to_robot_velocity: Translation2d = Translation2d(-robot_velocity.omega*turret_position.Y(), robot_velocity.omega*turret_position.X())

        return ChassisSpeeds(robot_velocity.vx+turret_to_robot_velocity.X(), robot_velocity.vy+turret_to_robot_velocity.Y(), robot_velocity.omega)


    def _calculate_turret_to_target_translation(self):
        turret_pose: Pose2d = self._get_turret_position()
        turret_translation: Translation2d = Translation2d(turret_pose.x, turret_pose.y)
        turret_rotation: Rotation2d = turret_pose.rotation()

        difference: Translation2d = self._target - turret_translation

        flight_time: seconds = LauncherSubsystemConstants.LATENCY + np.interp(metersToInches(difference.norm()), self.dist_array, self.flight_time_array)
        turret_velocity: ChassisSpeeds = self._get_turret_velocity()

        turret_travel_distance: Translation2d = Translation2d(turret_velocity.vx*flight_time, turret_velocity.vy*flight_time)

        difference -= turret_travel_distance

        difference.rotateBy(-turret_rotation)

        self._turret_to_target = difference

    def periodic(self) -> None:
        self._calculate_turret_to_target_translation()
        match self.state:
            case self.states.REST:
                pass
            case self.states.TRACK:
                pass # self.turret.aim(self.target_translation(self._target))
            case self.states.PREPARE:
                self._set_turret_and_flywheel()
                if abs(self.turret_to_target.angle().degrees()) \
                <= Translation2d(self.turret_to_target.norm(), TurretSubsystemConstants.AIM_TOLERANCE).angle().degrees() \
                and self.flywheel.is_at_speed():
                    self.state = self.states.FIRE
            case self.states.FIRE:
                self._set_turret_and_flywheel()
                self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER) if self.indexer != None else None
                self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED) if self.feeder != None else None
    
    def _set_turret_and_flywheel(self) -> None:
        target_translation: Translation2d = self.turret_to_target
        self.flywheel.set_speed(
            SC_LauncherSpeed(
                np.interp(
                    float(metersToInches(target_translation.norm())),
                    self.dist_array,
                    self.rpm_array
                ),
                0.0
            )
        )

    def aim_at(self, target: Translation2d, target_type: Literal["hub", "feed"]):
        self._target = target
        self._target_type = target_type

        self.dist_array = LauncherSubsystemConstants.FEED_DISTANCES if self._target_type == "feed" else LauncherSubsystemConstants.HUB_DISTANCES
        self.rpm_array = LauncherSubsystemConstants.FEED_RPM if self._target_type == "feed" else LauncherSubsystemConstants.HUB_RPM
        self.flight_time_array = LauncherSubsystemConstants.FEED_FLIGHT_TIME if self._target_type == "feed" else LauncherSubsystemConstants.HUB_FLIGHT_TIME

        self.state = self.states.TRACK
    
    def fire_at(self, target: Translation2d, target_type: Literal["hub", "feed"]):
        self._target = target
        self._target_type = target_type
        if self.state != self.states.FIRE:
            self.state = self.states.PREPARE
    
    def stop(self) -> None:
        self.feeder.set_power(0) if self.feeder != None else None
        self.indexer.set_power(IndexerSubsystemConstants.STOP_POWER) if self.indexer != None else None
        self.flywheel.set_power(0)
        
        self.state = self.states.REST