from commands2 import Command, ParallelCommandGroup

from src.oi import DemoInterface
from src.constants import \
    ClimberSubsystemConstants, \
    FeederSubsystemConstants, \
    IndexerSubsystemConstants, \
    IntakeSubsystemConstants

from src.subsystems.climber_subsystem import ClimberSubsystem
from src.subsystems.feeder_subsystem import FeederSubsystem
from src.subsystems.flywheel_subsystem import FlywheelSubsystem
from src.subsystems.indexer_subsystem import IndexerSubsystem
from src.subsystems.intake_subsystem import IntakeSubsystem
from src.subsystems.turret_subsystem import TurretSubsystem

class DemoCommand(ParallelCommandGroup):
    def __init__(self, oi: DemoInterface, *commands: Command):
        super().__init__(*commands)
        self.oi = oi

    def add_intake(self, subsystem: IntakeSubsystem):
        self.addCommands(DemoIntake(subsystem, self.oi))
    
    def add_climb(self, subsystem: ClimberSubsystem):
        self.addCommands(DemoClimb(subsystem, self.oi))
    
    def add_feeder(self, subsystem: FeederSubsystem):
        self.addCommands(DemoFeeder(subsystem, self.oi))
    
    def add_flywheel(self, subsystem: FlywheelSubsystem):
        self.addCommands(DemoFlywheel(subsystem, self.oi))
    
    def add_indexer(self, subsystem: IndexerSubsystem):
        self.addCommands(DemoIndexer(subsystem, self.oi))
        
    def add_turret(self, subsystem: TurretSubsystem):
        self.addCommands(DemoTurret(subsystem, self.oi))
    

class DemoIntake(Command):
    def __init__(self, intake: IntakeSubsystem, oi: DemoInterface):
        super().__init__()
        self.intake = intake
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_intake():
            self.intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_DEPLOY_POSITION)
        else:
            self.intake.set_pivot_angle(IntakeSubsystemConstants.PIVOT_HOME_POSITION)
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoClimb(Command):
    def __init__(self, climb: ClimberSubsystem, oi: DemoInterface):
        super().__init__()
        self.climb = climb
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_extend_climb():
            self.climb.set_power(ClimberSubsystemConstants.UP_POWER)
        
        if self.oi.demo_get_retract_climb():
            self.climb.set_power(ClimberSubsystemConstants.DOWN_POWER)
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoIndexer(Command):
    def __init__(self, indexer: IndexerSubsystem, oi: DemoInterface):
        super().__init__()
        self.indexer = indexer
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_feed():
            self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER)
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoFeeder(Command):
    def __init__(self, feeder: FeederSubsystem, oi: DemoInterface):
        super().__init__()
        self.feeder = feeder
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_feed():
            self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED)

    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoFlywheel(Command):
    def __init__(self, flywheel: FlywheelSubsystem, oi: DemoInterface):
        super().__init__()
        self.flywheel = flywheel
        self.oi = oi
        
    
    def execute(self):
        self.flywheel.set_power(self.oi.demo_get_flywheel())

    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoTurret(Command):
    def __init__(self, turret: TurretSubsystem, oi: DemoInterface):
        super().__init__()
        self.turret = turret
        self.oi = oi
        
    
    def execute(self):
        self.turret.set_power(self.oi.demo_get_turret())
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False