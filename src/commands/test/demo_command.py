from commands2 import Command, ParallelCommandGroup

from src.oi import DemoInterface
from src.constants import \
    ClimberSubsystemConstants, \
    FeederSubsystemConstants, \
    IndexerSubsystemConstants, \
    IntakeSubsystemConstants

from wpilib import DriverStation

from src.subsystems.climber_subsystem import ClimberSubsystem
from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
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
    
    def add_drive(self, subsystem: DrivetrainSubsystem):
        self.addCommands(DemoDrive(subsystem, self.oi))

class DemoIntake(Command):
    def __init__(self, intake: IntakeSubsystem, oi: DemoInterface):
        self.addRequirements(intake)
        self.intake = intake
        self.oi = oi
        self._has_deployed: bool = False
    
    def execute(self):
        if self.oi.demo_get_intake():
            self.intake.set_pivot(IntakeSubsystemConstants.DEPLOY_POSITION)
            self._has_deployed = True
        elif self.oi.demo_get_retract_intake():
            self.intake.set_pivot(IntakeSubsystemConstants.HOME_POSITION)
            self._has_deployed = False
        elif self._has_deployed:
            self.intake.set_pivot(IntakeSubsystemConstants.STOW_POSITION)
    
    def end(self, interrupted: bool):
        self.intake.stop_motors()
    
    def isFinished(self) -> bool:
        return False
    
class DemoClimb(Command):
    def __init__(self, climb: ClimberSubsystem, oi: DemoInterface):
        self.addRequirements(climb)
        self.climb = climb
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_extend_climb():
            self.climb.set_power(ClimberSubsystemConstants.UP_POWER)
        
        elif self.oi.demo_get_retract_climb():
            self.climb.set_power(ClimberSubsystemConstants.DOWN_POWER)

        else:
            self.climb.set_power(0)
    
    def end(self, interrupted: bool):
        self.climb.set_power(0)
    
    def isFinished(self) -> bool:
        return False
    
class DemoIndexer(Command):
    def __init__(self, indexer: IndexerSubsystem, oi: DemoInterface):
        self.addRequirements(indexer)
        self.indexer = indexer
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_feed():
            self.indexer.set_power(IndexerSubsystemConstants.INDEX_POWER)
        else:
            self.indexer.set_power(0)
    
    def end(self, interrupted: bool):
        self.indexer.set_power(0)
    
    def isFinished(self) -> bool:
        return False
    
class DemoFeeder(Command):
    def __init__(self, feeder: FeederSubsystem, oi: DemoInterface):
        self.addRequirements(feeder)
        self.feeder = feeder
        self.oi = oi
        
    
    def execute(self):
        if self.oi.demo_get_feed():
            self.feeder.set_velocity(FeederSubsystemConstants.FEED_SPEED)
        elif self.oi.demo_get_eject_feeder():
            self.feeder.set_velocity(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY)
        else:
            self.feeder.set_power((0, 0))

    def end(self, interrupted: bool):
        self.feeder.set_power((0, 0))
    
    def isFinished(self) -> bool:
        return False
    
class DemoFlywheel(Command):
    def __init__(self, flywheel: FlywheelSubsystem, oi: DemoInterface):
        self.addRequirements(flywheel)
        self.flywheel = flywheel
        self.oi = oi
        
    
    def execute(self):
        self.flywheel.set_power(self.oi.demo_get_flywheel())
        self.flywheel.print_diagnostics()

    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoTurret(Command):
    def __init__(self, turret: TurretSubsystem, oi: DemoInterface):
        self.addRequirements(turret)
        self.turret = turret
        self.oi = oi
        
    
    def execute(self):
        self.turret.set_power(self.oi.demo_get_turret())
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
class DemoDrive(Command):
    def __init__(self, drivetrain: DrivetrainSubsystem, oi: DemoInterface):
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain
        self.oi = oi
        
    
    def execute(self):
        throttle: float = self.oi.demo_get_throttle()
        strafe: float = self.oi.demo_get_strafe()
        rotation: float = self.oi.demo_get_rotate()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            throttle = -throttle
            strafe = -strafe

        self.drivetrain.drive(throttle, strafe, rotation, True)
    
    def end(self, interrupted: bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False