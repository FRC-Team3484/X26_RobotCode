from commands2 import Command, ParallelCommandGroup
from wpilib import SendableChooser, SmartDashboard

from oi import TestInterface
from robot_container import RobotContainer
from commands.test.flywheel_test_command import FlywheelTestCommand
from commands.test.feeder_test_command import FeederTestCommand
from commands.test.intake_test_command import IntakeTestCommand
from commands.test.turret_test_command import TurretTestCommand
from sysid_container import SysIDContainer
from constants import RobotConstants


class TestContainer:
    """
    Handles test commands

    Parameters:
        - robot_container (`RobotContainer`): the robot container
        - oi (`TestInterface`): the oi test interface
    """
    def __init__(self, oi: TestInterface, robot_container: RobotContainer) -> None:
        self._oi: TestInterface = oi
        self._robot_container: RobotContainer = robot_container
        self._sysid_container: SysIDContainer = SysIDContainer()
        
        self._mode_chooser: SendableChooser = SendableChooser()
        self._sysid_chooser: SendableChooser = SendableChooser()

        self._mode_chooser.setDefaultOption("Disabled", RobotConstants.TestMode.DISABLED)
        self._mode_chooser.addOption("Motor", RobotConstants.TestMode.MOTOR)
        self._mode_chooser.addOption("Demo", RobotConstants.TestMode.DEMO)
        SmartDashboard.putData("Test Mode", self._mode_chooser)

        self._sysid_chooser.setDefaultOption("Disabled", RobotConstants.SysIDMode.DISABLED)
        self._sysid_chooser.addOption("Drivetrain", RobotConstants.SysIDMode.DRIVETRAIN)
        self._sysid_chooser.addOption("Flywheel", RobotConstants.SysIDMode.FLYWHEEL)
        self._sysid_chooser.addOption("Feeder", RobotConstants.SysIDMode.FEEDER)
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
        When TestMode is DEMO, the demo commands will be run (not implemented yet)

        Returns:
            The command/command group for the currently selected test mode
        """
        if self._mode_chooser.getSelected() == RobotConstants.TestMode.DISABLED:
            print("[Test Container] No test mode selected, so no commands will be run")
            return Command()

        elif self._mode_chooser.getSelected() == RobotConstants.TestMode.MOTOR:
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

        elif self._mode_chooser.getSelected() == RobotConstants.TestMode.SYSID:
            return self._sysid_container.get_sysid_command(self._sysid_chooser.getSelected())

        elif self._mode_chooser.getSelected() == RobotConstants.TestMode.DEMO:
            # TODO: Implement demo commands
            return Command()