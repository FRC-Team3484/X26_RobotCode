from enum import Enum
from commands2 import Command, ParallelCommandGroup
from wpilib import SendableChooser, SmartDashboard

from commands.test.test_drive_command import TestDriveCommand
from oi import DriverInterface, TestInterface
from robot_container import RobotContainer
from commands.test.climber_test_command import ClimberTestCommand
from commands.test.flywheel_test_command import FlywheelTestCommand
from commands.test.feeder_test_command import FeederTestCommand
from commands.test.indexer_test_command import IndexerTestCommand
from commands.test.intake_test_command import IntakeTestCommand
from commands.test.turret_test_command import TurretTestCommand

from commands.test.launcher_rpm_test_command import LauncherRpmTestCommand
from commands.teleop.teleop_drive_command import TeleopDriveCommand

from sysid_container import SysIDContainer

class TestMode(Enum):
    DISABLED = 0
    MOTOR = 1
    SYSID = 2
    DEMO = 3
    LAUNCHER_RPM_TEST = 4

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
            test_interface: TestInterface, 
            robot_container: RobotContainer, 
        ) -> None:
        self._test_interface: TestInterface = test_interface
        self._robot_container: RobotContainer = robot_container

        self._sysid_container: SysIDContainer = SysIDContainer(test_interface, self._robot_container.drivetrain_subsystem)

        self._mode_chooser: SendableChooser = SendableChooser()
        self._sysid_chooser: SendableChooser = SendableChooser()

        self._mode_chooser.setDefaultOption("Disabled", TestMode.DISABLED)
        self._mode_chooser.addOption("Motor", TestMode.MOTOR)
        self._mode_chooser.addOption("SysID", TestMode.SYSID)
        self._mode_chooser.addOption("Demo", TestMode.DEMO)
        self._mode_chooser.addOption("Launcher RPM Test", TestMode.LAUNCHER_RPM_TEST)
        SmartDashboard.putData("Test Mode", self._mode_chooser)

        self._sysid_chooser.setDefaultOption("Disabled", SysIDMode.DISABLED)
        if self._robot_container.drivetrain_subsystem is not None:
            self._sysid_chooser.addOption("Drivetrain Drive", object=SysIDMode.DRIVETRAIN_DRIVE)
            self._sysid_chooser.addOption("Drivetrain Steer", SysIDMode.DRIVETRAIN_STEER)
        if self._robot_container.flywheel_subsystem is not None:
            self._sysid_chooser.addOption("Flywheel", SysIDMode.FLYWHEEL)
        if self._robot_container.feeder_subsystem is not None:
            self._sysid_chooser.addOption("Feeder", SysIDMode.FEEDER)
        SmartDashboard.putData("SysID Mode", self._sysid_chooser)

        SmartDashboard.putBoolean("Climber Enabled", False)
        SmartDashboard.putBoolean("Flywheel Test Enabled", False)
        SmartDashboard.putBoolean("Feeder Test Enabled", False)
        SmartDashboard.putBoolean("Indexer Test Enabled", False)
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

            if SmartDashboard.getBoolean("Climber Enabled", False) and self._robot_container.climber_subsystem is not None:
                commands.append(ClimberTestCommand(self._robot_container.climber_subsystem, self._test_interface))

            if SmartDashboard.getBoolean("Flywheel Test Enabled", False) and self._robot_container.flywheel_subsystem is not None:
                commands.append(FlywheelTestCommand(self._test_interface, self._robot_container.flywheel_subsystem))

            if SmartDashboard.getBoolean("Feeder Test Enabled", False) and self._robot_container.feeder_subsystem is not None:
                commands.append(FeederTestCommand(self._test_interface, self._robot_container.feeder_subsystem))

            if SmartDashboard.getBoolean("Indexer Test Enabled", False) and self._robot_container.indexer_subsystem is not None:
                commands.append(IndexerTestCommand(self._robot_container.indexer_subsystem, self._test_interface))

            if SmartDashboard.getBoolean("Intake Test Enabled", False) and self._robot_container.intake_subsystem is not None:
                commands.append(IntakeTestCommand(self._test_interface, self._robot_container.intake_subsystem))

            if SmartDashboard.getBoolean("Turret Test Enabled", False) and self._robot_container.turret_subsystem is not None:
                commands.append(TurretTestCommand(self._test_interface, self._robot_container.turret_subsystem))

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

        elif self._mode_chooser.getSelected() == TestMode.DEMO:
            # TODO: Implement demo commands
            return Command()

        elif self._mode_chooser.getSelected() == TestMode.LAUNCHER_RPM_TEST:
            if self._robot_container.flywheel_subsystem is not None and self._robot_container.indexer_subsystem is not None and self._robot_container.feeder_subsystem is not None:
                command_group: ParallelCommandGroup = ParallelCommandGroup(
                    LauncherRpmTestCommand(self._test_interface, self._robot_container.flywheel_subsystem, self._robot_container.indexer_subsystem, self._robot_container.feeder_subsystem)
                )

                if self._robot_container.drivetrain_subsystem is not None:
                    command_group.addCommands(
                        TestDriveCommand(self._robot_container.drivetrain_subsystem, self._test_interface)
                    )

                return command_group
            else:
                print("[Test Container] Launcher RPM Test requires flywheel, indexer, and feeder subsystems")
                return Command()

        else:
            print("[Test Container] Unknown test mode selected, so no commands will be run")
            return Command()