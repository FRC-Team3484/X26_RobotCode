from enum import Enum

from commands2 import Command
from wpilib import SendableChooser, SmartDashboard

class Mode(Enum):
    DISABLED = 0
    MOTOR = 1
    DEMO = 2

class TestContainer:
    def __init__(self) -> None:
        self._mode_chooser: SendableChooser = SendableChooser()

        self._mode_chooser.setDefaultOption("Disabled", Mode.DISABLED)
        self._mode_chooser.addOption("Motor", Mode.MOTOR)
        self._mode_chooser.addOption("Demo", Mode.DEMO)
        SmartDashboard.putData("Test Mode", self._mode_chooser)

        SmartDashboard.putBoolean("Flywheel Test Enabled", False)
        SmartDashboard.putBoolean("Indexer Test Enabled", False)
        SmartDashboard.putBoolean("Intake Test Enabled", False)

    def get_test_command(self) -> Command:
        if self._mode_chooser.getSelected() == Mode.DISABLED:
            print("[Test Container] No test mode selected, so no commands will be run")
            return Command()

        elif self._mode_chooser.getSelected() == Mode.MOTOR:
            commands: list[Command] = []

            if SmartDashboard.getBoolean("Flywheel Test Enabled", False):
                commands.append(Command())

            if SmartDashboard.getBoolean("Indexer Test Enabled", False):
                commands.append(Command())

            if SmartDashboard.getBoolean("Intake Test Enabled", False):
                commands.append(Command())

            return commands

        else:
            return Command()