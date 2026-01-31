import math
from enum import Enum

from commands2 import Command
from wpilib import DriverStation
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from constants import SwerveConstants, TeleopDriveConstants
from oi import DriverInterface

class DriveState(Enum):
    DRIVE = 0
    PIVOT = 1

class TeleopDriveCommand(Command):
    """
    Teleop drive command that takes the driver interface inputs and converts them to robot movement

    Parameters:
        - drivetrain (`DrivetrainSubsystem`): the drivetrain subsystem
        - driver_oi (`DriverInterface`): the driver interface
    """
    def __init__(self, drivetrain: DrivetrainSubsystem, driver_oi: DriverInterface|None = None) -> None:
        super().__init__()
        self.addRequirements(drivetrain)

        self._drivetrain: DrivetrainSubsystem = drivetrain
        self._oi: DriverInterface|None = driver_oi

        self._state: DriveState = DriveState.DRIVE

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
            match self._state:
                case DriveState.DRIVE:
                    povs = [self._oi.get_jog_up(),
                            self._oi.get_jog_right(),
                            self._oi.get_jog_down(),
                            self._oi.get_jog_left()]

                    if self._oi.get_reset_heading():
                        self._drivetrain.set_heading()

                    if self._oi.get_dynamic_pivot():
                        self._state = DriveState.PIVOT
                        
                        self._pivot_corner = Translation2d(1, math.copysign(1.0, self._oi.get_rotation()))
                        pivot_drive: Translation2d = Translation2d(self._oi.get_throttle(), self._oi.get_strafe())
                        if self._alliance == DriverStation.Alliance.kBlue:
                            pivot_drive = Translation2d(-pivot_drive.x, -pivot_drive.y)

                        self._pivot_corner.rotateBy(pivot_drive.angle())
                        self._pivot_corner.rotateBy(self._drivetrain.get_pose().rotation())
                        self._pivot_corner = Translation2d(
                            math.copysign(1.0, self._pivot_corner.X()) * SwerveConstants.DRIVETRAIN_LENGTH,
                            math.copysign(1.0, self._pivot_corner.Y()) * SwerveConstants.DRIVETRAIN_WIDTH
                        )

                    elif self._oi.get_hold_mode():
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
                            strafe += 1.0
                        if povs[3]:
                            strafe -= 1.0

                        self._drivetrain.drive_robotcentric(
                            ChassisSpeeds(throttle * TeleopDriveConstants.JOG_SPEED,
                                          strafe * TeleopDriveConstants.JOG_SPEED,
                                          0.0),
                            True)
                    
                    else:
                        throttle: float = self._oi.get_throttle()
                        strafe: float = self._oi.get_strafe()
                        rotation: float = self._oi.get_rotation()

                        if self._alliance == DriverStation.Alliance.kRed:
                            throttle = -throttle
                            strafe = -strafe

                        if self._oi.get_low_speed_mode():
                            throttle *= TeleopDriveConstants.LOW_SPEED
                            strafe *= TeleopDriveConstants.LOW_SPEED
                            rotation *= TeleopDriveConstants.LOW_SPEED

                        self._drivetrain.drive(throttle, strafe, rotation, True)

                case DriveState.PIVOT:
                    self._drivetrain.dynamic_pivot_drive(0, 0, 1.0 if self._oi.get_rotation() > 0 else -1.0, self._pivot_corner, True)
                    if not self._oi.get_dynamic_pivot():
                        self._state = DriveState.DRIVE

    def end(self, interrupted: bool) -> None:
        self._drivetrain.stop_motors()

    def isFinished(self) -> bool:
        return False
    
