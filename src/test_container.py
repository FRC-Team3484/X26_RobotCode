from enum import Enum

from commands2 import Command, ParallelCommandGroup
from wpilib import SendableChooser, SmartDashboard

from oi import TestInterface
from robot_container import RobotContainer
from commands.test.climber_test_command import ClimberTestCommand
from commands.test.flywheel_test_command import FlywheelTestCommand
from commands.test.feeder_test_command import FeederTestCommand
from commands.test.indexer_test_command import IndexerTestCommand
from commands.test.intake_test_command import IntakeTestCommand
from commands.test.turret_test_command import TurretTestCommand

class Mode(Enum):
    DISABLED = 0
    MOTOR = 1
    DEMO = 2

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
        
        self._mode_chooser: SendableChooser = SendableChooser()

        self._mode_chooser.setDefaultOption("Disabled", Mode.DISABLED)
        self._mode_chooser.addOption("Motor", Mode.MOTOR)
        self._mode_chooser.addOption("Demo", Mode.DEMO)
        SmartDashboard.putData("Test Mode", self._mode_chooser)

        SmartDashboard.putBoolean("Climber Enabled", False)
        SmartDashboard.putBoolean("Flywheel Test Enabled", False)
        SmartDashboard.putBoolean("Feeder Test Enabled", False)
        SmartDashboard.putBoolean("Indexer Test Enabled", False)
        SmartDashboard.putBoolean("Intake Test Enabled", False)
        SmartDashboard.putBoolean("Turret Test Enabled", False)

    def get_test_command(self) -> Command:
        """
        Returns the command/command group for the currently selected test mode

        When Mode is DISABLED, no commands will be run
        When Mode is MOTOR, the flywheel, feeder, and intake test commands will be run, if they are enabled
        When Mode is DEMO, the demo commands will be run (not implemented yet)

        Returns:
            The command/command group for the currently selected test mode
        """
        if self._mode_chooser.getSelected() == Mode.DISABLED:
            print("[Test Container] No test mode selected, so no commands will be run")
            return Command()

        elif self._mode_chooser.getSelected() == Mode.MOTOR:
            commands: list[Command] = []

            if SmartDashboard.getBoolean("Climber Enabled", False):
                commands.append(ClimberTestCommand(self._robot_container.climber_subsystem, self._oi))

            if SmartDashboard.getBoolean("Flywheel Test Enabled", False):
                commands.append(FlywheelTestCommand(self._oi, self._robot_container.flywheel_subsystem))

            if SmartDashboard.getBoolean("Feeder Test Enabled", False):
                commands.append(FeederTestCommand(self._oi, self._robot_container.feeder_subsystem))

            if SmartDashboard.getBoolean("Indexer Test Enabled", False):
                commands.append(IndexerTestCommand(self._robot_container.indexer_subsystem, self._oi))

            if SmartDashboard.getBoolean("Intake Test Enabled", False):
                commands.append(IntakeTestCommand(self._oi, self._robot_container.intake_subsystem))

            if SmartDashboard.getBoolean("Turret Test Enabled", False):
                commands.append(TurretTestCommand(self._oi, self._robot_container.turret_subsystem))

            return ParallelCommandGroup(*commands)

        else:
            # TODO: Implement demo commands
            return Command()