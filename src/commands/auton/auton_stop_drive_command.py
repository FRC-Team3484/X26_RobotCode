from commands2 import Command

from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem

class AutonStopDriveCommand(Command):
    """
    A command for stopping the drivetrain at the end of auton

    The command stops the motors and ends

    Parameters:
        - drivetrain_subsystem ('DrivetrainSubsystem'): subsytem for launcher operation
    """
    def __init__(self, drivetrain_subsystem: DrivetrainSubsystem):
        super().__init__()
        self._drivetrain_subsystem: DrivetrainSubsystem = drivetrain_subsystem

        self.addRequirements(drivetrain_subsystem)

    def initialize(self):
        self._drivetrain_subsystem.stop_motors()

    def isFinished(self) -> bool:
        return True