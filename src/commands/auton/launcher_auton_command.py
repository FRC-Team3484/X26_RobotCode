from commands2 import Command
from wpimath.geometry import Pose2d, Translation2d

from frc3484.pose_manipulation import get_april_tag_poses, get_nearest_pose

from subsystems.launcher_subsystem import LauncherSubsystem
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from constants import LauncherSubsystemConstants, RobotConstants, VisionConstants

HUB_APRIL_TAGS = VisionConstants.HubAprilTags


class LauncherAutonCommand(Command):
    def __init__(self, launcher: LauncherSubsystem, drivetrain: DrivetrainSubsystem, launcher_constants: LauncherSubsystemConstants):
        super().__init__()
        self.launcher = launcher
        self.drivetrain = drivetrain
        self.launcher_constants = launcher_constants
        self.field = RobotConstants.APRIL_TAG_FIELD_LAYOUT
        

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