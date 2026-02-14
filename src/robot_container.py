from commands2.command import Command
from commands2 import ParallelCommandGroup
from wpilib import Field2d, SmartDashboard

from oi import DriverInterface, OperatorInterface
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from subsystems.climber_subsystem import ClimberSubsystem
from subsystems.feed_target_subsystem import FeedTargetSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.indexer_subsystem import IndexerSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.launcher_subsystem import LauncherSubsystem
from subsystems.turret_subsystem import TurretSubsystem
from constants import VisionConstants, SwerveConstants

from commands.teleop.teleop_intake_command import TeleopIntakeCommand
from commands.teleop.teleop_drive_command import TeleopDriveCommand
from commands.teleop.teleop_turret_tracking_command import TeleopTurretTrackingCommand
from commands.teleop.teleop_climb_command import TeleopClimbCommand

from commands.teleop.teleop_drive_slow_command import TeleopDriveSlowCommand
from commands.teleop.teleop_launch_command import TeleopLaunchCommand


from frc3484.pathfinding import SC_Pathfinding
import config
from subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem

class RobotContainer:
    def __init__(self, driver_interface: DriverInterface, operator_interface: OperatorInterface) -> None:
        self._field: Field2d = Field2d()
        self._oi: OperatorInterface = OperatorInterface()
        SmartDashboard.putData('Field', self._field)

        self._driver_interface: DriverInterface = driver_interface
        self._operator_interface: OperatorInterface = operator_interface
        
        # Subsystems
        if config.DRIVETRAIN_ENABLED:
            self._drivetrain_subsystem: DrivetrainSubsystem = DrivetrainSubsystem(None, None, self._field)
            self._pathfinder: SC_Pathfinding = SC_Pathfinding(
                drivetrain_subsystem=self._drivetrain_subsystem,
                pose_supplier= self._drivetrain_subsystem.get_pose,
                output=lambda speeds: self._drivetrain_subsystem.drive_robotcentric(speeds, False),
                alignment_controller=SwerveConstants.ALIGNMENT_CONTROLLER
            )
            
        if config.CLIMBER_ENABLED:
            self._climber_subsystem: ClimberSubsystem = ClimberSubsystem()

        if config.FLYWHEEL_ENABLED:
            self._flywheel_subsystem: FlywheelSubsystem = FlywheelSubsystem()

        if config.FEEDER_ENABLED:
            self._feeder_subsystem: FeederSubsystem = FeederSubsystem()

        if config.INDEXER_ENABLED:
            self._indexer_subsystem: IndexerSubsystem = IndexerSubsystem()

        if config.INTAKE_ENABLED:
            self._intake_subsystem: IntakeSubsystem = IntakeSubsystem()

        if config.TURRET_ENABLED:
            self._turret_subsystem: TurretSubsystem = TurretSubsystem()

        if config.FEED_TARGET_ENABLED:
            self._feed_target_subsystem: FeedTargetSubsystem = FeedTargetSubsystem(self._operator_interface, self._field)

        # Command Groups
        self._intake_commands: ParallelCommandGroup = ParallelCommandGroup()
        self._feed_commands: ParallelCommandGroup = ParallelCommandGroup()
        self._launch_commands: ParallelCommandGroup = ParallelCommandGroup()
        self._goto_climb_commands: ParallelCommandGroup = ParallelCommandGroup()

        if config.INTAKE_ENABLED:
            self._intake_commands.addCommands(
                TeleopIntakeCommand(self._intake_subsystem, self._operator_interface)
            )
            self._feed_commands.addCommands(
                TeleopIntakeCommand(self._intake_subsystem, self._operator_interface)
            )
            self._launch_commands.addCommands(
                TeleopIntakeCommand(self._intake_subsystem, self._operator_interface)
            )
        if config.TURRET_ENABLED and self.launcher_subsystem:
            self._intake_commands.addCommands(
                TeleopTurretTrackingCommand(self.launcher_subsystem, self._feed_target_subsystem, self._drivetrain_subsystem)
            )
        if config.CLIMBER_ENABLED:
            self._intake_commands.addCommands(
                TeleopClimbCommand(self._operator_interface, self._climber_subsystem)
            )
            self._goto_climb_commands.addCommands(
                self.goto_climb()
            )
        if config.DRIVETRAIN_ENABLED:
            self._intake_commands.addCommands(
                TeleopDriveCommand(self._drivetrain_subsystem, self._driver_interface)
            )
            if config.LAUNCH_WHILE_MOVING_ENABLED:
                self._feed_commands.addCommands(
                    TeleopDriveSlowCommand(self._drivetrain_subsystem, self._driver_interface)
                )
                self._launch_commands.addCommands(
                    TeleopDriveSlowCommand(self._drivetrain_subsystem, self._driver_interface)
                )
        if self.launcher_subsystem:
            self._launch_commands.addCommands(
                TeleopLaunchCommand(self.launcher_subsystem, self._operator_interface, self._feed_target_subsystem)
            )

    # Subsystem Properties
    @property
    def drivetrain_subsystem(self) -> DrivetrainSubsystem | None:
        if config.DRIVETRAIN_ENABLED:
            return self._drivetrain_subsystem
        else:
            print("[RobotContainer] Unable to return DrivetrainSubsystem because it is disabled")
            return None

    @property
    def climber_subsystem(self) -> ClimberSubsystem | None:
        if config.CLIMBER_ENABLED:
            return self._climber_subsystem
        else:
            print("[RobotContainer] Unable to return ClimberSubsystem because it is disabled")
            return None

    @property
    def flywheel_subsystem(self) -> FlywheelSubsystem | None:
        if config.FLYWHEEL_ENABLED:
            return self._flywheel_subsystem
        else:
            print("[RobotContainer] Unable to return FlywheelSubsystem because it is disabled")
            return None

    @property
    def feeder_subsystem(self) -> FeederSubsystem | None:
        if config.FEEDER_ENABLED:
            return self._feeder_subsystem
        else:
            print("[RobotContainer] Unable to return FeederSubsystem because it is disabled")
            return None

    @property
    def indexer_subsystem(self) -> IndexerSubsystem | None:
        if config.INDEXER_ENABLED:
            return self._indexer_subsystem
        else:
            print("[RobotContainer] Unable to return IndexerSubsystem because it is disabled")
            return None

    @property
    def intake_subsystem(self) -> IntakeSubsystem | None:
        if config.INTAKE_ENABLED:
            return self._intake_subsystem
        else:
            print("[RobotContainer] Unable to return IntakeSubsystem because it is disabled")
            return None

    @property
    def launcher_subsystem(self) -> LauncherSubsystem | TurretlessLauncherSubsystem | None:
        if config.FLYWHEEL_ENABLED and config.DRIVETRAIN_ENABLED:
            if config.TURRET_ENABLED:
                return LauncherSubsystem(self.feeder_subsystem, self.indexer_subsystem, self._flywheel_subsystem, self._turret_subsystem, self._drivetrain_subsystem)
            else:
                return TurretlessLauncherSubsystem(self.feeder_subsystem, self.indexer_subsystem, self._flywheel_subsystem, self._drivetrain_subsystem)
        else:
            print("[RobotContainer] Unable to return LauncherSubsystem because the required subsystems are disabled")
            return None

    @property
    def turret_subsystem(self) -> TurretSubsystem | None:
        if config.TURRET_ENABLED:
            return self._turret_subsystem
        else:
            print("[RobotContainer] Unable to return TurretSubsystem because it is disabled")
            return None

    @property
    def feed_target_subsystem(self) -> FeedTargetSubsystem | None:
        if config.FEED_TARGET_ENABLED:
            return self._feed_target_subsystem
        else:
            print("[RobotContainer] Unable to return FeedTargetSubsystem because it is disabled")
            return None
        
    def goto_climb(self) -> Command:
        return self._pathfinder.pathfind_to_target(VisionConstants.ClimbAprilTagTarget)

    # Command Group Properties
    @property
    def teleop_intake_commands(self) -> Command:
        return self._intake_commands

    @property
    def teleop_feed_commands(self) -> Command:
        return self._feed_commands

    @property
    def teleop_launch_commands(self) -> Command:
        return self._launch_commands

    @property
    def goto_climb_commands(self) -> Command:
        return self._goto_climb_commands