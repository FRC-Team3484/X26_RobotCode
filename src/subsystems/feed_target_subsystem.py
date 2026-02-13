from typing import override

from wpilib import Field2d, DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import meters
from commands2 import Subsystem

from frc3484.pose_manipulation import apply_offset_to_pose, get_april_tag_pose

from constants import FeedTargetSubsystemConstants, RobotConstants
from oi import OperatorInterface


class FeedTargetSubsystem(Subsystem):
    """
    Handles the target points for where to feed pieces, and draws them on a Field2d

    Parameters:
        - operator_interface (`OperatorInterface`): The operator interface for the robot
        - field (`Field2d`): The field to draw the targets on
    """
    def __init__(self, operator_interface: OperatorInterface, field: Field2d) -> None:
        super().__init__()

        self._oi: OperatorInterface = operator_interface
        self._field: Field2d = field

        self._target_1: Translation2d = FeedTargetSubsystemConstants.TARGET_1_INITIAL_POSITION
        self._target_2: Translation2d = FeedTargetSubsystemConstants.TARGET_2_INITIAL_POSITION

        # Invert initial points when starting on the red alliance
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self._target_1 = self._invert_target(self._target_1)
            self._target_2 = self._invert_target(self._target_2)

        # Calculate hub location
        self._hub_location: Translation2d
        red_hub_pose: Pose2d | None = get_april_tag_pose(10, RobotConstants.APRIL_TAG_FIELD_LAYOUT)
        blue_hub_pose: Pose2d | None = get_april_tag_pose(26, RobotConstants.APRIL_TAG_FIELD_LAYOUT)

        if not red_hub_pose: red_hub_pose = Pose2d()
        if not blue_hub_pose: blue_hub_pose = Pose2d()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self._hub_location = apply_offset_to_pose(red_hub_pose, FeedTargetSubsystemConstants.HUB_OFFSET).translation()
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self._hub_location = apply_offset_to_pose(blue_hub_pose, FeedTargetSubsystemConstants.HUB_OFFSET).translation()

    @override
    def periodic(self) -> None:
        """
        Calculates the amount to move the target points and then applies them, 
            makes sure they are within the field, then draws them on the field
        """
        move: float = FeedTargetSubsystemConstants.TARGET_MOVE_SPEED * RobotConstants.TICK_RATE
        self._target_1 = self._limit_target_to_field(Translation2d(
            move * self._oi.get_left_feed_point_axis_x() + self._target_1.X(), 
            move * self._oi.get_left_feed_point_axis_y() + self._target_1.Y()
        ))
        self._target_2 = self._limit_target_to_field(Translation2d(
            move * self._oi.get_right_feed_point_axis_x() + self._target_2.X(), 
            move * self._oi.get_right_feed_point_axis_y() + self._target_2.Y()
        ))

        self._field.getObject("target_1").setPose(Pose2d(self._target_1, Rotation2d()))
        self._field.getObject("target_2").setPose(Pose2d(self._target_2, Rotation2d()))

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
            RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2, 
            RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() / 2
        )

        center_offset: Translation2d = target - field_center_point
        return center_offset.rotateBy(Rotation2d().fromDegrees(180.0)) + field_center_point

    def _limit_target_to_field(self, target: Translation2d) -> Translation2d:
        """
        Limits a target to be within the field

        Parameters:
            target (Translation2d): The target to limit

        Returns:
            Translation2d: The limited target
        """
        x: meters = max(min(target.X(), RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth()), 0)
        y: meters = max(min(target.Y(), RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength()), 0)

        return Translation2d(x, y)