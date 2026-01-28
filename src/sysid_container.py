from commands2 import Command


class SysIDContainer():
    """
    Handles SysID commands
    """
    def __init__(self) -> None:
        pass

    def get_drivetrain_sysid(self) -> Command:
        """
        Returns a command group that runs the drivetrain SysID based on the buttons that are pressed
        """
        # TODO: Add drivetrain SysID
        return Command()

    def get_flywheel_sysid(self) -> Command:
        """
        Returns a command group that runs the flywheel SysID based on the buttons that are pressed
        """
        # TODO: Add flywheel SysID
        return Command()

    def get_feeder_sysid(self) -> Command:
        """
        Returns a command group that runs the feeder SysID based on the buttons that are pressed
        """
        # TODO: Add feeder SysID
        return Command()