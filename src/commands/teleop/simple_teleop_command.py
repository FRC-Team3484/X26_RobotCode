from commands2 import Command, ParallelCommandGroup

from src.oi import OperatorInterface
from src.constants import \
    ClimberSubsystemConstants, \
    FeederSubsystemConstants, \
    IndexerSubsystemConstants, \
    IntakeSubsystemConstants, \
    TurretSubsystemConstants, \
    UserInterface

from src.datatypes import TargetType

from src.subsystems.climber_subsystem import ClimberSubsystem
from src.subsystems.feeder_subsystem import FeederSubsystem
from src.subsystems.flywheel_subsystem import FlywheelSubsystem
from src.subsystems.indexer_subsystem import IndexerSubsystem
from src.subsystems.intake_subsystem import IntakeSubsystem
from src.subsystems.turret_subsystem import TurretSubsystem
from src.subsystems.feed_target_subsystem import FeedTargetSubsystem

class SimpleTeleopCommand(ParallelCommandGroup):
    """
    A simpler teleop command group, that allows two controllers to run all of the subsystems without any complex automation

    Paramaters:
        oi: OperatorInterface - The operator interface
    """
    def __init__(self, oi: OperatorInterface, *commands: Command):
        super().__init__(*commands)
        self.oi: OperatorInterface = oi

    def add_intake(self, subsystem: IntakeSubsystem):
        self.addCommands(SimpleIntake(subsystem, self.oi))
    
    def add_climb(self, subsystem: ClimberSubsystem):
        self.addCommands(SimpleClimb(subsystem, self.oi))
    
    def add_feeder(self, subsystem: FeederSubsystem):
        self.addCommands(SimpleFeeder(subsystem, self.oi))
    
    def add_flywheel(self, subsystem: FlywheelSubsystem, feed_target: FeedTargetSubsystem):
        self.addCommands(SimpleFlywheel(subsystem, self.oi, feed_target))
    
    def add_indexer(self, subsystem: IndexerSubsystem):
        self.addCommands(SimpleIndexer(subsystem, self.oi))
        
    def add_turret(self, subsystem: TurretSubsystem):
        self.addCommands(SimpleTurret(subsystem, self.oi))

class SimpleIntake(Command):
    def __init__(self, intake: IntakeSubsystem, oi: OperatorInterface) -> None:
        super().__init__()
        self.addRequirements(intake)
        self.intake: IntakeSubsystem = intake
        self.oi: OperatorInterface = oi
        self._has_deployed: bool = False
    
    def execute(self) -> None:
        if self.oi.get_simple_intake():
            self.intake.set_pivot(IntakeSubsystemConstants.DEPLOY_POSITION)
            self._has_deployed = True
        elif self.oi.get_simple_retract_intake():
            self.intake.set_pivot(IntakeSubsystemConstants.HOME_POSITION)
            self._has_deployed = False
        elif self._has_deployed:
            self.intake.set_pivot(IntakeSubsystemConstants.STOW_POSITION)
    
    def end(self, interrupted: bool) -> None:
        self.intake.stop_motors()
    
    def isFinished(self) -> bool:
        return False
    
class SimpleClimb(Command):
    def __init__(self, climb: ClimberSubsystem, oi: OperatorInterface) -> None:
        super().__init__()
        self.addRequirements(climb)
        self.climb: ClimberSubsystem = climb
        self.oi: OperatorInterface = oi
    
    def execute(self):
        if self.oi.get_simple_climber_extend():
            self.climb.set_power(ClimberSubsystemConstants.UP_POWER)
        
        elif self.oi.get_simple_climber_retract():
            self.climb.set_power(ClimberSubsystemConstants.DOWN_POWER)

        else:
            self.climb.set_power(0)
    
    def end(self, interrupted: bool) -> None:
        self.climb.set_power(0)
    
    def isFinished(self) -> bool:
        return False
    
class SimpleIndexer(Command):
    def __init__(self, indexer: IndexerSubsystem, oi: OperatorInterface) -> None:
        super().__init__()
        self.addRequirements(indexer)
        self.indexer: IndexerSubsystem = indexer
        self.oi: OperatorInterface = oi
        
    def execute(self) -> None:
        if self.oi.get_simple_feed():
            self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER)
        else:
            self.indexer.set_power(0)
    
    def end(self, interrupted: bool) -> None:
        self.indexer.set_power(0)
    
    def isFinished(self) -> bool:
        return False
    
class SimpleFeeder(Command):
    def __init__(self, feeder: FeederSubsystem, oi: OperatorInterface) -> None:
        super().__init__()
        self.addRequirements(feeder)
        self.feeder: FeederSubsystem = feeder
        self.oi: OperatorInterface = oi
        
    def execute(self) -> None:
        if self.oi.get_simple_feed():
            self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED)
        elif self.oi.get_simple_eject():
            self.feeder.set_velocity(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY)
        else:
            self.feeder.set_power((0, 0))

    def end(self, interrupted: bool) -> None:
        self.feeder.set_power((0, 0))
    
    def isFinished(self) -> bool:
        return False
    
class SimpleFlywheel(Command):
    def __init__(self, flywheel: FlywheelSubsystem, oi: OperatorInterface, feed_target: FeedTargetSubsystem) -> None:
        super().__init__()
        self.addRequirements(flywheel)
        self.flywheel: FlywheelSubsystem = flywheel
        self.oi: OperatorInterface = oi
        self.feed_target: FeedTargetSubsystem = feed_target
        
    def execute(self) -> None:
        flywheel_power: float = self.oi.get_simple_flywheel()
        if flywheel_power > 0:
            self.flywheel.set_power(flywheel_power)
        elif self.oi.get_flywheel_rpm():
            self.flywheel.set_speed(self.feed_target.get_target(TargetType.HUB).flywheel_speed)

        self.flywheel.print_diagnostics()
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class SimpleTurret(Command):
    def __init__(self, turret: TurretSubsystem, oi: OperatorInterface) -> None:
        super().__init__()
        self.addRequirements(turret)
        self.turret: TurretSubsystem = turret
        self.oi: OperatorInterface = oi
    
    def execute(self) -> None:
        turret_request: float = self.oi.get_simple_turret()
        if self.turret.get_position() >= TurretSubsystemConstants.MAXIMUM_ANGLE and turret_request > 0:
            turret_request = 0
            self.oi.set_rumble(UserInterface.Operator.RUMBLE_HIGH)
        elif self.turret.get_position() <= TurretSubsystemConstants.MINIMUM_ANGLE and turret_request < 0:
            turret_request = 0
            self.oi.set_rumble(UserInterface.Operator.RUMBLE_HIGH)
        else:
            self.oi.set_rumble(UserInterface.Operator.RUMBLE_OFF)
        
        self.turret.set_power(turret_request)
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False