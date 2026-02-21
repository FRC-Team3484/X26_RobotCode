from enum import Enum

from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, InstantCommand


class AutonMode(Enum):
    NONE = 0

class AutonGenerator:
    """
    Handles returning auton commands
    """
    def __init__(self) -> None:
        self._auton_chooser: SendableChooser = SendableChooser()

        self._auton_chooser.setDefaultOption("None", AutonMode.NONE)
        SmartDashboard.putData("Auton Mode", self._auton_chooser)

    def get_auton_command(self) -> Command:
        match self._auton_chooser.getSelected():
            case AutonMode.NONE:
                return InstantCommand()

            case _:
                print("[Auton Generator] No auton mode selected, so no commands will be run")
                return Command()