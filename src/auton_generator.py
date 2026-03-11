from enum import Enum

from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.path import PathPlannerPath
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, InstantCommand

from src.commands.auton.auton_intake_command import AutonIntakeCommand
from src.commands.auton.launcher_auton_command import LauncherAutonCommand
from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.subsystems.intake_subsystem import IntakeSubsystem
from src.subsystems.launcher_subsystem import LauncherSubsystem
from src.subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem


class AutonMode(Enum):
    NONE = 0
    TWO_CYCLE_LEFT = 1
    TWO_CYCLE_RIGHT = 2
    FEED_LEFT = 3
    FEED_RIGHT = 4
    PLOW_LEFT = 5
    PLOW_RIGHT = 6

class AutonGenerator:
    """
    Handles returning auton commands
    """
    def __init__(self, drivetrain_subsystem: DrivetrainSubsystem | None, launcher_subsystem: LauncherSubsystem | TurretlessLauncherSubsystem | None, intake_subsystem: IntakeSubsystem | None) -> None:
        self._drivetrain_subsystem: DrivetrainSubsystem | None = drivetrain_subsystem
        self._launcher_subsystem: LauncherSubsystem | TurretlessLauncherSubsystem | None = launcher_subsystem
        self._intake_subsystem: IntakeSubsystem | None = intake_subsystem

        # Register NamedCommands
        if self._launcher_subsystem and self._drivetrain_subsystem:
            NamedCommands.registerCommand("Score", LauncherAutonCommand(self._launcher_subsystem, self._drivetrain_subsystem))
        else:
            print("[Auton Generator] Unable to register named score command")
            NamedCommands.registerCommand("Score", InstantCommand())

        if self._intake_subsystem:
            NamedCommands.registerCommand("Intake", AutonIntakeCommand(self._intake_subsystem))
        else:
            print("[Auton Generator] Unable to register named intake command")
            NamedCommands.registerCommand("Intake", InstantCommand())

        self._auton_chooser: SendableChooser = SendableChooser()

        self._auton_chooser.setDefaultOption("None", AutonMode.NONE)
        self._auton_chooser.addOption("Two Cycle Left", AutonMode.TWO_CYCLE_LEFT)
        self._auton_chooser.addOption("Two Cycle Right", AutonMode.TWO_CYCLE_RIGHT)
        self._auton_chooser.addOption("Feed Left", AutonMode.FEED_LEFT)
        self._auton_chooser.addOption("Feed Right", AutonMode.FEED_RIGHT)
        SmartDashboard.putData("Auton Mode", self._auton_chooser)

    def _load_named_path(self, path_name: str) -> Command:
        path: PathPlannerPath = PathPlannerPath.fromPathFile(path_name)
        return AutoBuilder.followPath(path)

    def get_auton_command(self) -> Command:
        if self._drivetrain_subsystem is None:
            print("[Auton Generator] No drivetrain subsystem, so no commands will be run")
            return Command()

        match self._auton_chooser.getSelected():
            case AutonMode.NONE:
                return InstantCommand()

            case AutonMode.TWO_CYCLE_LEFT:
                return self._load_named_path(path_name="Two Cycle Left")

            case AutonMode.TWO_CYCLE_RIGHT:
                return self._load_named_path(path_name="Two Cycle Right")

            case AutonMode.FEED_LEFT:
                return self._load_named_path(path_name="Feed Left")

            case AutonMode.FEED_RIGHT:
                return self._load_named_path(path_name="Feed Right")

            case AutonMode.PLOW_LEFT:
                return self._load_named_path(path_name="Plow Left")

            case AutonMode.PLOW_RIGHT:
                return self._load_named_path(path_name="Plow Right")

            case _:
                print("[Auton Generator] No auton mode selected, so no commands will be run")
                return Command()