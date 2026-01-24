from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.intake_subsystem import IntakeSubsystem

import config

class RobotContainer:
    def __init__(self) -> None:
        
        # Subsystems
        if config.FLYWHEEL_ENABLED:
            self._flywheel_subsystem: FlywheelSubsystem = FlywheelSubsystem()

        if config.FEEDER_ENABLED:
            self._indexer_subsystem: FeederSubsystem = FeederSubsystem()

        if config.INTAKE_ENABLED:
            self._intake_subsystem: IntakeSubsystem = IntakeSubsystem()

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