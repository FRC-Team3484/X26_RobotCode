from commands2 import Command
from wpilib import DriverStation
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from wpimath.filter import SlewRateLimiter

from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.constants import TeleopDriveConstants
from src.oi import DriverInterface

class TeleopDriveSlowCommand(Command):
    """
    Teleop drive command that takes the driver interface inputs and converts them to robot movement, but slowly, and with a slew filter

    Parameters:
        - drivetrain (`DrivetrainSubsystem`): the drivetrain subsystem
        - driver_oi (`DriverInterface`): the driver interface
    """
    def __init__(self, drivetrain: DrivetrainSubsystem, driver_oi: DriverInterface|None = None) -> None:
        super().__init__()
        self.addRequirements(drivetrain)

        self._drivetrain: DrivetrainSubsystem = drivetrain
        self._oi: DriverInterface|None = driver_oi

        self._pivot_corner: Translation2d = Translation2d()

        self._alliance: DriverStation.Alliance = DriverStation.Alliance.kBlue

    def initialize(self) -> None:
        '''
        Drivetrain is field-centric but controls are driver-centric so we need to know the alliance color to know if conrols should be flipped.
        Do this every time the drive command is initialized so we don't need to restart code to change alliance during testing.
        '''
        alliance: DriverStation.Alliance | None = DriverStation.getAlliance()
        if alliance is None:
            print('Teleop Drive Command failed to determine alliance color')
        else:
            self._alliance = alliance

    def execute(self) -> None:
        if self._oi is not None:
            povs = [self._oi.get_jog_up(),
                    self._oi.get_jog_right(),
                    self._oi.get_jog_down(),
                    self._oi.get_jog_left()]

            if self._oi.get_hold_mode():
                self._drivetrain.set_module_states((
                    SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                    SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
                    SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)),
                    SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0))
                ), True, False)

            elif any(povs):
                strafe: float = 0.0
                throttle: float = 0.0
                if povs[0]:
                    throttle += 1.0
                if povs[2]:
                    throttle -= 1.0
                if povs[1]:
                    strafe -= 1.0
                if povs[3]:
                    strafe += 1.0

                self._drivetrain.drive_robotcentric(
                    ChassisSpeeds(
                        throttle * TeleopDriveConstants.JOG_SPEED,
                        strafe * TeleopDriveConstants.JOG_SPEED,
                        0.0
                        ),
                    True
                    )
            
            else:

                throttle: float = self._oi.get_throttle()
                strafe: float = self._oi.get_strafe()

                if self._alliance == DriverStation.Alliance.kBlue:
                    throttle = -throttle
                    strafe = -strafe

                self._drivetrain.drive(
                    throttle * TeleopDriveConstants.LOW_SPEED, strafe * TeleopDriveConstants.LOW_SPEED,
                    self._oi.get_rotation() * TeleopDriveConstants.LOW_SPEED,
                    True
                    )

    def end(self, interrupted: bool) -> None:
        self._drivetrain.stop_motors()

    def isFinished(self) -> bool:
        return False
    
