from typing import override
from commands2 import Command
from src.oi import OperatorInterface
from src.subsystems.launcher_subsystem import LauncherSubsystem
from src.subsystems.feed_target_subsystem import FeedTargetSubsystem

class TeleopLaunchCommand(Command):
    """
    Teleop Command for Launching

    Handles the operator buttons allowing them to launch pieces

    Parameters:
        - launch (`LauncherSubsystem`): the launcher subsystem
        - driver_oi (`OperatorInterface`): the oi operator interface for controller bindings
        - feed (`FeedTargetSubsystem`): the subsystem to generate the Translation2D needed for targets
    """
    def __init__(self, launch: LauncherSubsystem, driver_oi: OperatorInterface, feed: FeedTargetSubsystem):
        super().__init__()
        self.addRequirements(launch) 

        self._launch: LauncherSubsystem = launch
        self._oi: OperatorInterface = driver_oi
        self._feed: FeedTargetSubsystem = feed
    
    @override
    def execute(self) -> None:
        if self._oi.get_launcher():
            self._launch.fire_at(target= self._feed.get_hub_position(), target_type= 'hub')
        elif self._oi.get_left_feed_point():
            self._launch.fire_at(target= self._feed.get_target_1(), target_type= 'feed')
        elif self._oi.get_right_feed_point():
            self._launch.fire_at(target= self._feed.get_target_2(), target_type= 'feed')

    @override
    def isFinished(self) -> bool:
            return False
    
    @override
    def end(self, interrupted: bool) -> None:
        self._launch.stop()