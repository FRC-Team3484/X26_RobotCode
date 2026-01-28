from commands2 import Command

from constants import RobotConstants


class SysIDContainer():
    """
    Handles SysID commands
    """
    def __init__(self) -> None:
        pass

    def get_sysid_command(self, mode: RobotConstants.SysIDMode) -> Command:
        """
        Returns the command/command group for the currently selected sysid mode

        When SysIDMode is DISABLED, no commands will be run
        When SysIDMode is DRIVETRAIN, the drivetrain sysid command will be run
        When SysIDMode is FLYWHEEL, the flywheel sysid command will be run
        When SysIDMode is FEEDER, the feeder sysid command will be run

        Returns:
            The command/command group for the currently selected sysid mode
        """
        if mode == RobotConstants.SysIDMode.DISABLED:
            print("[SysID Container] No sysid mode selected, so no commands will be run")
            return Command()

        elif mode == RobotConstants.SysIDMode.DRIVETRAIN:
            return Command()

        elif mode == RobotConstants.SysIDMode.FLYWHEEL:
            return Command()

        elif mode == RobotConstants.SysIDMode.FEEDER:
            return Command()