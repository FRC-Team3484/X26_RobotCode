from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.turret_subsystem import TurretSubsystem

import config

class RobotContainer:
    def __init__(self) -> None:
        
        # Subsystems
        if config.DRIVETRAIN_ENABLED:
            self._drivetrain_subsystem: DrivetrainSubsystem = DrivetrainSubsystem(None, None)
            
        if config.FLYWHEEL_ENABLED:
            self._flywheel_subsystem: FlywheelSubsystem = FlywheelSubsystem()

        if config.FEEDER_ENABLED:
            self._indexer_subsystem: FeederSubsystem = FeederSubsystem()

        if config.INTAKE_ENABLED:
            self._intake_subsystem: IntakeSubsystem = IntakeSubsystem()

        if config.TURRET_ENABLED:
            self._turret_subsystem: TurretSubsystem = TurretSubsystem()

    @property
    def drivetrain_subsystem(self) -> DrivetrainSubsystem:
        if config.DRIVETRAIN_ENABLED:
            return self._drivetrain_subsystem
        else:
            raise AttributeError("[RobotContainer] Unable to return DrivetrainSubsystem because it is disabled")

    @property
    def flywheel_subsystem(self) -> FlywheelSubsystem:
        if config.FLYWHEEL_ENABLED:
            return self._flywheel_subsystem
        else:
            raise AttributeError("[RobotContainer] Unable to return FlywheelSubsystem because it is disabled")

    @property
    def feeder_subsystem(self) -> FeederSubsystem:
        if config.FEEDER_ENABLED:
            return self._indexer_subsystem
        else:
            raise AttributeError("[RobotContainer] Unable to return FeederSubsystem because it is disabled")

    @property
    def intake_subsystem(self) -> IntakeSubsystem:
        if config.INTAKE_ENABLED:
            return self._intake_subsystem
        else:
            raise AttributeError("[RobotContainer] Unable to return IntakeSubsystem because it is disabled")

    @property
    def turret_subsystem(self) -> TurretSubsystem:
        if config.TURRET_ENABLED:
            return self._turret_subsystem
        else:
            raise AttributeError("[RobotContainer] Unable to return TurretSubsystem because it is disabled")