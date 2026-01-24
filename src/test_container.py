from enum import Enum

from commands2 import Command, ParallelCommandGroup
from wpilib import SendableChooser, SmartDashboard

from oi import TestInterface
from robot_container import RobotContainer
from commands.test.flywheel_test_command import FlywheelTestCommand
from commands.test.feeder_test_command import FeederTestCommand
from commands.test.intake_test_command import IntakeTestCommand

class Mode(Enum):
    DISABLED = 0
    MOTOR = 1
    DEMO = 2

class TestContainer:
    def __init__(self, robot_container: RobotContainer, oi: TestInterface) -> None:
        self._robot_container: RobotContainer = robot_container
        self._oi: TestInterface = oi
        
        self._mode_chooser: SendableChooser = SendableChooser()

        self._mode_chooser.setDefaultOption("Disabled", Mode.DISABLED)
        self._mode_chooser.addOption("Motor", Mode.MOTOR)
        self._mode_chooser.addOption("Demo", Mode.DEMO)
        SmartDashboard.putData("Test Mode", self._mode_chooser)

        SmartDashboard.putBoolean("Flywheel Test Enabled", False)
        SmartDashboard.putBoolean("Feeder Test Enabled", False)
        SmartDashboard.putBoolean("Intake Test Enabled", False)

    def get_test_command(self) -> Command:
        if self._mode_chooser.getSelected() == Mode.DISABLED:
            print("[Test Container] No test mode selected, so no commands will be run")
            return Command()

        elif self._mode_chooser.getSelected() == Mode.MOTOR:
            commands: list[Command] = []

            if SmartDashboard.getBoolean("Flywheel Test Enabled", False):
                commands.append(FlywheelTestCommand(self._robot_container.flywheel_subsystem, self._oi))

            if SmartDashboard.getBoolean("Feeder Test Enabled", False):
                commands.append(FeederTestCommand(self._oi, self._robot_container.feeder_subsystem))

            if SmartDashboard.getBoolean("Intake Test Enabled", False):
                commands.append(IntakeTestCommand(self._robot_container.intake_subsystem, self._oi))

            return ParallelCommandGroup(*commands)

        else:
            # TODO: Implement demo commands
            return Command()