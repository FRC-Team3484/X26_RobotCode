from commands2 import Command
from oi import DemoInterface
from constants import \
    ClimberSubsystemConstants, \
    FeederSubsystemConstants, \
    FlywheelSubsystemConstants, \
    IndexerSubsystemConstants, \
    IntakeSubsystemConstants, \
    TurretSubsystemConstants

from subsystems.climber_subsystem import ClimberSubsystem
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.indexer_subsystem import IndexerSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.turret_subsystem import TurretSubsystem

class DemoCommand(Command):
    def __init__(
            self,
            oi: DemoInterface,
            climb: ClimberSubsystem,
            intake: IntakeSubsystem,
            feeder: FeederSubsystem,
            flywheel: FlywheelSubsystem,
            indexer: IndexerSubsystem,
            turret: TurretSubsystem
        ):
        super().__init__()
        
        self.oi: DemoInterface = oi

        self.climber_constants = ClimberSubsystemConstants
        self.intake_constants = IntakeSubsystemConstants
        self.feeder_constants = FeederSubsystemConstants
        self.flywheel_constants = FlywheelSubsystemConstants
        self.indexer_constants = IndexerSubsystemConstants
        self.turret_constants = TurretSubsystemConstants

        self.climber = climb
        self.intake = intake
        self.feeder = feeder
        self.flywheel = flywheel
        self.indexer = indexer
        self.turret = turret

    def execute(self):
        if self.oi.demo_get_extend_climb():
            self.climber.set_power(self.climber_constants.UP_POWER)
        
        if self.oi.demo_get_retract_climb():
            self.climber.set_power(self.climber_constants.DOWN_POWER)

        if self.oi.demo_get_intake():
            self.intake.set_pivot_angle(self.intake_constants.PIVOT_DEPLOY_POSITION)
        
        if self.oi.demo_get_feed():
            self.indexer.set_power(self.indexer_constants.INDEX_POWER)
            self.feeder.set_velocity(self.feeder_constants.FEED_SPEED)
        
        if self.oi.demo_get_eject_feeder():
            self.feeder.set_velocity(self.feeder_constants.REMOVE_PIECE_VELOCITY)

        self.flywheel.set_power(self.oi.demo_get_flywheel())
        self.turret.set_power(self.oi.demo_get_turret())

    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return super().isFinished()
