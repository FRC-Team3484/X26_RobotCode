from typing import override
from commands2 import Command
from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds

from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from constants import TeleopDriveConstants
from oi import DemoInterface

class TestDriveCommand(Command):
    """
    Test drive command that takes the driver interface inputs and converts them to robot movement

    Parameters:
        - drivetrain (`DrivetrainSubsystem`): the drivetrain subsystem
        - test_oi (`TestInterface`): the driver interface
    """
    def __init__(self, drivetrain: DrivetrainSubsystem, test_oi: DemoInterface | None = None) -> None:
        super().__init__()
        self.addRequirements(drivetrain)

        self._drivetrain: DrivetrainSubsystem = drivetrain
        self._oi: DemoInterface | None = test_oi

        self._alliance: DriverStation.Alliance = DriverStation.Alliance.kBlue

    @override
    def initialize(self) -> None:
        '''
        Drivetrain is field-centric but controls are driver-centric so we need to know the alliance color to know if conrols should be flipped.
        Do this every time the drive command is initialized so we don't need to restart code to change alliance during testing.
        '''
        alliance: DriverStation.Alliance | None = DriverStation.getAlliance()
        if alliance is None:
            print('Test Drive Command failed to determine alliance color')
        else:
            self._alliance = alliance

    @override
    def execute(self) -> None:
        if self._oi is not None:
            povs = [self._oi.demo_get_jog_up(),
                    self._oi.demo_get_jog_right(),
                    self._oi.demo_get_jog_down(),
                    self._oi.demo_get_jog_up()]

            if self._oi.demo_get_reset_heading():
                self._drivetrain.set_heading()

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
                throttle: float = self._oi.demo_get_throttle()
                strafe: float = self._oi.demo_get_strafe()
                rotation: float = self._oi.demo_get_rotate()

                if self._alliance == DriverStation.Alliance.kRed:
                    throttle = -throttle
                    strafe = -strafe

                self._drivetrain.drive(throttle, strafe, rotation, True)

    @override
    def end(self, interrupted: bool) -> None:
        self._drivetrain.stop_motors()

    @override
    def isFinished(self) -> bool:
        return False
