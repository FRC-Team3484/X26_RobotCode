from wpilib import Field2d, SmartDashboard

from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from subsystems.climber_subsystem import ClimberSubsystem
from subsystems.feed_target_subsystem import FeedTargetSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.indexer_subsystem import IndexerSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.turret_subsystem import TurretSubsystem

import config

class RobotContainer:
    def __init__(self) -> None:
        self._field: Field2d = Field2d()
        SmartDashboard.putData('Field', self._field)
        
        # Subsystems
        if config.DRIVETRAIN_ENABLED:
            self._drivetrain_subsystem: DrivetrainSubsystem = DrivetrainSubsystem(None, None, self._field)
            
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
            self._feed_target_subsystem: FeedTargetSubsystem = FeedTargetSubsystem(self._field)

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