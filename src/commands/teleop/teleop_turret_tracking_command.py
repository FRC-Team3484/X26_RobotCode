from typing import override
from commands2 import Command

from wpilib import DriverStation

from src.subsystems.launcher_subsystem import LauncherSubsystem
from src.subsystems.feed_target_subsystem import FeedTargetSubsystem
from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.constants import RobotConstants
from src.subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem
from src.datatypes import TargetType


class TeleopTurretTrackingCommand(Command):
    """
    Docstring for TeleopLauncherCommand
    Tells the turret what target to follow

    Parameters:
        - launch (`IntakeSubsystem`): the launcher subsystem
        - feed (`FeedTargetSubsystem`): gets the feed targets for the turret
        - drive (`DrivetrainSubsystem`): uses the nearest target by getting pose from drivetrain
    """
    def __init__(self, launch: LauncherSubsystem | TurretlessLauncherSubsystem, feed: FeedTargetSubsystem, drive: DrivetrainSubsystem, feed_target_subsystem: FeedTargetSubsystem) -> None:
        super().__init__()
        self.addRequirements() 
        
        self._launcher: LauncherSubsystem | TurretlessLauncherSubsystem = launch
        self._feed: FeedTargetSubsystem = feed
        self._drive: DrivetrainSubsystem = drive
        self._feed_target_subsystem: FeedTargetSubsystem = feed_target_subsystem

        self._alliance: DriverStation.Alliance | None = DriverStation.Alliance.kBlue

    @override
    def initialize(self):
        self._alliance = DriverStation.getAlliance()
    
    @override
    def execute(self) -> None:
        if self._in_alliance_zone():
            self._launcher.aim_at(TargetType.HUB)
        else:
            dist1 = self._drive.get_pose().translation().distance(self._feed.get_target_1())
            dist2 = self._drive.get_pose().translation().distance(self._feed.get_target_2())
            if dist1 < dist2:
                self._launcher.aim_at(TargetType.TARGET_1)
            else:
                self._launcher.aim_at(TargetType.TARGET_2)

    @override
    def isFinished(self) -> bool:
            return False
    
    @override
    def end(self, interrupted: bool) -> None:
        self._launcher.stop()

    def _in_alliance_zone(self) -> bool:
        x_position = self._drive.get_pose().X()
        if self._alliance == DriverStation.Alliance.kBlue and x_position < RobotConstants.ALLIANCE_ZONE_POSITION:
            return True

        if self._alliance == DriverStation.Alliance.kRed and x_position > RobotConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() - RobotConstants.ALLIANCE_ZONE_POSITION:
            return True
        return False
        



