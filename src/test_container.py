from enum import Enum
from commands2 import Command, ParallelCommandGroup
from wpilib import SendableChooser, SmartDashboard

from oi import TestInterface
from robot_container import RobotContainer
from commands.test.flywheel_test_command import FlywheelTestCommand
from commands.test.feeder_test_command import FeederTestCommand
from commands.test.intake_test_command import IntakeTestCommand
from commands.test.turret_test_command import TurretTestCommand
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from sysid_container import SysIDContainer

class TestMode(Enum):
        DISABLED = 0
        MOTOR = 1
        SYSID = 2
        DEMO = 3

class SysIDMode(Enum):
    DISABLED = 0
    DRIVETRAIN_DRIVE = 1
    DRIVETRAIN_STEER = 2
    FLYWHEEL = 3
    FEEDER = 4


class TestContainer:
    """
    Handles test commands

    Parameters:
        - robot_container (`RobotContainer`): the robot container
        - oi (`TestInterface`): the oi test interface
    """
    def __init__(self, 
            oi: TestInterface, 
            robot_container: RobotContainer, 
            drivetrain_subsystem: DrivetrainSubsystem | None = None, 
            flywheel_subsystem: FlywheelSubsystem | None = None, 
            feeder_subsystem: FeederSubsystem | None = None
        ) -> None:
        self._oi: TestInterface = oi
        self._robot_container: RobotContainer = robot_container
        self._sysid_container: SysIDContainer = SysIDContainer(oi, drivetrain_subsystem)
        
        self._mode_chooser: SendableChooser = SendableChooser()
        self._sysid_chooser: SendableChooser = SendableChooser()

        self._mode_chooser.setDefaultOption("Disabled", TestMode.DISABLED)
        self._mode_chooser.addOption("Motor", TestMode.MOTOR)
        self._mode_chooser.addOption("SysID", TestMode.SYSID)
        self._mode_chooser.addOption("Demo", TestMode.DEMO)
        SmartDashboard.putData("Test Mode", self._mode_chooser)

        self._sysid_chooser.setDefaultOption("Disabled", SysIDMode.DISABLED)
        if drivetrain_subsystem is not None:
            self._sysid_chooser.addOption("Drivetrain Drive", object=SysIDMode.DRIVETRAIN_DRIVE)
            self._sysid_chooser.addOption("Drivetrain Steer", SysIDMode.DRIVETRAIN_STEER)
        if flywheel_subsystem is not None:
            self._sysid_chooser.addOption("Flywheel", SysIDMode.FLYWHEEL)
        if feeder_subsystem is not None:
            self._sysid_chooser.addOption("Feeder", SysIDMode.FEEDER)
        SmartDashboard.putData("SysID Mode", self._sysid_chooser)

        SmartDashboard.putBoolean("Flywheel Test Enabled", False)
        SmartDashboard.putBoolean("Feeder Test Enabled", False)
        SmartDashboard.putBoolean("Intake Test Enabled", False)
        SmartDashboard.putBoolean("Turret Test Enabled", False)

    def get_test_command(self) -> Command:
        """
        Returns the command/command group for the currently selected test mode

        When TestMode is DISABLED, no commands will be run
        When TestMode is MOTOR, the flywheel, feeder, and intake test commands will be run, if they are enabled
        When TestMode is SYSID, the sysid commands will be run, based on the selected sysid mode
        When TestMode is DEMO, the demo commands will be run (not implemented yet)

        Returns:
            The command/command group for the currently selected test mode
        """
        if self._mode_chooser.getSelected() == TestMode.DISABLED:
            print("[Test Container] No test mode selected, so no commands will be run")
            return Command()

        elif self._mode_chooser.getSelected() == TestMode.MOTOR:
            commands: list[Command] = []

            if SmartDashboard.getBoolean("Flywheel Test Enabled", False):
                commands.append(FlywheelTestCommand(self._oi, self._robot_container.flywheel_subsystem))

            if SmartDashboard.getBoolean("Feeder Test Enabled", False):
                commands.append(FeederTestCommand(self._oi, self._robot_container.feeder_subsystem))

            if SmartDashboard.getBoolean("Intake Test Enabled", False):
                commands.append(IntakeTestCommand(self._oi, self._robot_container.intake_subsystem))

            if SmartDashboard.getBoolean("Turret Test Enabled", False):
                commands.append(TurretTestCommand(self._oi, self._robot_container.turret_subsystem))

            return ParallelCommandGroup(*commands)

        elif self._mode_chooser.getSelected() == TestMode.SYSID:
            if self._sysid_chooser.getSelected() == SysIDMode.DISABLED:
                print("[Test Container] No sysid mode selected, so no commands will be run")
                return Command()

            elif self._sysid_chooser.getSelected() == SysIDMode.DRIVETRAIN_DRIVE:
                return self._sysid_container.get_drivetrain_sysid("drive")

            elif self._sysid_chooser.getSelected() == SysIDMode.DRIVETRAIN_STEER:
                return self._sysid_container.get_drivetrain_sysid("steer")

            elif self._sysid_chooser.getSelected() == SysIDMode.FLYWHEEL:
                return self._sysid_container.get_flywheel_sysid()

            elif self._sysid_chooser.getSelected() == SysIDMode.FEEDER:
                return self._sysid_container.get_feeder_sysid()

            else:
                return Command()

        elif self._mode_chooser.getSelected() == TestMode.DEMO:
            # TODO: Implement demo commands
            return Command()

        # I know you won't like this but the linter insists
        else:
            return Command()