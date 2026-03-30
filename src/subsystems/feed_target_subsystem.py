import numpy as np

from typing import Callable, override

from wpilib import DataLogManager, Field2d, DriverStation
from wpiutil.log import DataLog, FloatLogEntry
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters, seconds, metersToInches
from commands2 import Subsystem

from frc3484.motion import SC_SpeedRequest
from frc3484.pose_manipulation import apply_offset_to_pose, get_april_tag_pose

from src.config import LAUNCH_WHILE_MOVING_ENABLED
from src.constants import FeedTargetSubsystemConstants, RobotConstants
from src.datatypes import TargetType, LauncherTarget
from src.oi import OperatorInterface
import src.config as config


class FeedTargetSubsystem(Subsystem):
    """
    Handles the target points for where to feed pieces, and draws them on a Field2d

    Parameters:
        - operator_interface (`OperatorInterface`): The operator interface for the robot
        - field (`Field2d`): The field to draw the targets on
        - robot_pose_source (`Callable[[], Pose2d]`): A callable that returns the current robot pose
        - robot_velocity_source (`Callable[[], ChassisSpeeds]`): A callable that returns the current robot velocity
    """
    def __init__(self, operator_interface: OperatorInterface, field: Field2d, robot_pose_source: Callable[[], Pose2d], robot_velocity_source: Callable[[], ChassisSpeeds]) -> None:
        super().__init__()

        self._oi: OperatorInterface = operator_interface
        self._field: Field2d = field
        self._robot_pose: Pose2d = Pose2d()
        self._robot_velocity: ChassisSpeeds = ChassisSpeeds()
        self._robot_pose_source: Callable[[], Pose2d] = robot_pose_source
        self._robot_velocity_source: Callable[[], ChassisSpeeds] = robot_velocity_source
        self._target_1: Translation2d = FeedTargetSubsystemConstants.TARGET_1_INITIAL_POSITION
        self._target_2: Translation2d = FeedTargetSubsystemConstants.TARGET_2_INITIAL_POSITION

        # Invert initial points when starting on the red alliance
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self._target_1 = self._invert_target(self._target_1)
            self._target_2 = self._invert_target(self._target_2)

        # Calculate hub location
        self._hub_location: Translation2d = self._calculate_hub_position()
        
        # Logging
        if config.LOGGING_ENABLED:
            log: DataLog = DataLogManager.getLog()
            self._turret_pose_x_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_pose/x")
            self._turret_pose_y_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_pose/y")
            self._turret_pose_rotation_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_pose/rotatoin")
            self._flight_time_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/flight_time")
            self._turret_velocity_x_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_velocity/x")
            self._turret_velocity_y_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_velocity/y")
            self._turret_velocity_rotation_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_velocity/rotation")
            self._turret_to_target_x_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_to_target/x")
            self._turret_to_target_y_log: FloatLogEntry = FloatLogEntry(log, "/feed_target/turret_to_target/y")
            self._target_rpm_log: FloatLogEntry = FloatLogEntry(log, "/flywheel/target_rpm")

    @override
    def periodic(self) -> None:
        """
        Calculates the amount to move the target points and then applies them, 
            makes sure they are within the field, then draws them on the field
        """
        self._robot_pose = self._robot_pose_source()
        self._robot_velocity = self._robot_velocity_source()
        move: float = FeedTargetSubsystemConstants.TARGET_MOVE_SPEED * RobotConstants.TICK_RATE
        self._target_1 = self._limit_target_to_field(Translation2d(
            move * self._oi.get_left_feed_point_axis_x() + self._target_1.X(), 
            move * self._oi.get_left_feed_point_axis_y() + self._target_1.Y()
        ))
        self._target_2 = self._limit_target_to_field(Translation2d(
            move * self._oi.get_right_feed_point_axis_x() + self._target_2.X(), 
            move * self._oi.get_right_feed_point_axis_y() + self._target_2.Y()
        ))

        # Sometimes the robot would randomly lose the position of the hub. If the hub position is back at (0, 0), repair it
        if self._hub_location.X() == 0 and self._hub_location.Y() == 0:
            self._hub_location = self._calculate_hub_position()

        self._field.getObject("target_1").setPose(Pose2d(self._target_1, Rotation2d()))
        self._field.getObject("target_2").setPose(Pose2d(self._target_2, Rotation2d()))
        self._field.getObject("hub").setPose(Pose2d(self.get_hub_position(), Rotation2d()))

    def get_target_1(self) -> Translation2d:
        """
        Returns the first target point

        Returns:
            Translation2d: The first target point
        """
        return self._target_1

    def get_target_2(self) -> Translation2d:
        """
        Returns the second target point

        Returns:
            Translation2d: The second target point
        """
        return self._target_2

    def get_hub_position(self) -> Translation2d:
        """
        Returns the hub position based on the selected alliance

        Returns:
            Translation2d: The hub position
        """
        return self._hub_location

    def _invert_target(self, target: Translation2d) -> Translation2d:
        """
        Inverts a target to be on the opposite side of the field

        Parameters:
            target (Translation2d): The target to invert

        Returns:
            Translation2d: The inverted target
        """
        field_center_point: Translation2d = Translation2d(
            RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() / 2, 
            RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2
        )
        return target.rotateAround(field_center_point, Rotation2d().fromDegrees(180.0))

    def _limit_target_to_field(self, target: Translation2d) -> Translation2d:
        """
        Limits a target to be within the field

        Parameters:
            target (Translation2d): The target to limit

        Returns:
            Translation2d: The limited target
        """
        x: meters = max(min(target.X(), RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength()), 0)
        y: meters = max(min(target.Y(), RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth()), 0)

        return Translation2d(x, y)

    def _calculate_hub_position(self) -> Translation2d:
        """
        Calculates the hub position based on the selected alliance

        Returns:
            Translation2d: The hub position
        """
        red_hub_pose: Pose2d | None = get_april_tag_pose(10, RobotConstants.APRIL_TAG_FIELD_LAYOUT)
        blue_hub_pose: Pose2d | None = get_april_tag_pose(26, RobotConstants.APRIL_TAG_FIELD_LAYOUT)

        if not red_hub_pose: red_hub_pose = Pose2d()
        if not blue_hub_pose: blue_hub_pose = Pose2d()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return apply_offset_to_pose(red_hub_pose, FeedTargetSubsystemConstants.HUB_OFFSET).translation()
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return apply_offset_to_pose(blue_hub_pose, FeedTargetSubsystemConstants.HUB_OFFSET).translation()
        else:
            return Translation2d()
    
    def get_target(self, target_type: TargetType) -> LauncherTarget:
        """
        Gets the robot-centric vector from the turret to a target

        Parameters:
            target_type (TargetType): The target to get the translation to
        Returns:
            LauncherTarget: The robot-centric vector from the turret to the target
        """

        def get_turret_position() -> Pose2d:
            return Pose2d(
                self._robot_pose.translation() + FeedTargetSubsystemConstants.TURRET_OFFSET.translation().rotateBy(self._robot_pose.rotation()),
                self._robot_pose.rotation() + FeedTargetSubsystemConstants.TURRET_OFFSET.rotation()
            )
        def get_turret_velocity() -> ChassisSpeeds:
            robot_velocity: ChassisSpeeds = self._robot_velocity
            turret_position: Pose2d = get_turret_position()
            turret_to_robot_velocity: Translation2d = Translation2d(-robot_velocity.omega*turret_position.Y(), robot_velocity.omega*turret_position.X())

            return ChassisSpeeds(robot_velocity.vx+turret_to_robot_velocity.X(), robot_velocity.vy+turret_to_robot_velocity.Y(), robot_velocity.omega)

        if target_type == TargetType.TARGET_1:
            target: Translation2d = self.get_target_1()
            rpm_array: np.ndarray = FeedTargetSubsystemConstants.FEED_RPM
            dist_array: np.ndarray = FeedTargetSubsystemConstants.FEED_DISTANCES
            time_array: np.ndarray = FeedTargetSubsystemConstants.FEED_FLIGHT_TIME

        elif target_type == TargetType.TARGET_2:
            target: Translation2d = self.get_target_2()
            rpm_array: np.ndarray = FeedTargetSubsystemConstants.FEED_RPM
            dist_array: np.ndarray = FeedTargetSubsystemConstants.FEED_DISTANCES
            time_array: np.ndarray = FeedTargetSubsystemConstants.FEED_FLIGHT_TIME
        elif target_type == TargetType.HUB:
            target: Translation2d = self.get_hub_position()
            rpm_array: np.ndarray = FeedTargetSubsystemConstants.HUB_RPM
            dist_array: np.ndarray = FeedTargetSubsystemConstants.HUB_DISTANCES
            time_array: np.ndarray = FeedTargetSubsystemConstants.HUB_FLIGHT_TIME
        else:
            return LauncherTarget(Translation2d(1, 0), SC_SpeedRequest(0, 0))
        
        turret_pose: Pose2d = get_turret_position()
        turret_translation: Translation2d = turret_pose.translation()
        turret_rotation: Rotation2d = turret_pose.rotation()

        turret_to_target: Translation2d = target - turret_translation

        if LAUNCH_WHILE_MOVING_ENABLED:
            flight_time: seconds = FeedTargetSubsystemConstants.LATENCY + np.interp(metersToInches(turret_to_target.norm()), dist_array, time_array)
            turret_velocity: ChassisSpeeds = get_turret_velocity()

            turret_travel_distance: Translation2d = Translation2d(turret_velocity.vx*flight_time, turret_velocity.vy*flight_time)

            turret_to_target -= turret_travel_distance

            if config.LOGGING_ENABLED:
                self._flight_time_log.append(flight_time)
                self._turret_velocity_x_log.append(turret_velocity.vx)
                self._turret_velocity_y_log.append(turret_velocity.vy)
                self._turret_velocity_rotation_log.append(turret_velocity.omega)

        turret_to_target = turret_to_target.rotateBy(-turret_rotation)
        flywheel_speed: SC_SpeedRequest = SC_SpeedRequest(
            np.interp(metersToInches(turret_to_target.norm()), dist_array, rpm_array),
            0.0
        )

        if config.LOGGING_ENABLED:
            self._turret_pose_x_log.append(turret_pose.X())
            self._turret_pose_y_log.append(turret_pose.Y())
            self._turret_pose_rotation_log.append(turret_pose.rotation().degrees())
            self._turret_to_target_x_log.append(turret_to_target.X())
            self._turret_to_target_y_log.append(turret_to_target.Y())
            self._target_rpm_log.append(flywheel_speed.speed)

        return LauncherTarget(turret_to_target, flywheel_speed)
