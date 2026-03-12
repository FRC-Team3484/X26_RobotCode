from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d
from commands2 import Command

from frc3484.pose_manipulation import get_april_tag_poses, get_nearest_pose

from src.subsystems.launcher_subsystem import LauncherSubsystem
from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.constants import RobotConstants, VisionConstants
from src.subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem

HUB_APRIL_TAGS = VisionConstants.HubAprilTags


class LauncherAutonCommand(Command):
    """
    A command for operating the launcher in auton

    Parameters:
        - launcher ('LauncherSubsystem'): subsytem for launcher operation
    """
    def __init__(self, launcher: LauncherSubsystem | TurretlessLauncherSubsystem, drivetrain: DrivetrainSubsystem):
        super().__init__()
        self.launcher: LauncherSubsystem | TurretlessLauncherSubsystem = launcher
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.field: AprilTagFieldLayout = RobotConstants.APRIL_TAG_FIELD_LAYOUT

        self.addRequirements(launcher)

    def execute(self):
        current_pose: Pose2d = self.drivetrain.get_pose()
        hub_poses: list[Pose2d] = get_april_tag_poses(
            [
                HUB_APRIL_TAGS.RED_ID,
                HUB_APRIL_TAGS.BLUE_ID
            ],
            RobotConstants.APRIL_TAG_FIELD_LAYOUT
        )

        self.launcher.aim_at(get_nearest_pose(current_pose, hub_poses).translation(), 'hub')
        # self.launcher.aim_at()
        pass
    
    def end(self, interrupted: bool):
        self.launcher.stop()
    
    def isFinished(self) -> bool:
        return super().isFinished()