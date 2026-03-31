from typing import cast, override

from wpilib import SmartDashboard
from wpimath.units import revolutions_per_minute
from commands2 import Command

from frc3484.motion import SC_SpeedRequest

from src.oi import DemoInterface
from src.datatypes import FeederSpeed, TargetType
from src.constants import IndexerSubsystemConstants, FeederSubsystemConstants
from src.subsystems.feeder_subsystem import FeederSubsystem
from src.subsystems.flywheel_subsystem import FlywheelSubsystem
from src.subsystems.indexer_subsystem import IndexerSubsystem
from subsystems.feed_target_subsystem import FeedTargetSubsystem


class LauncherRpmTestCommand(Command):
    """
    Command for testing the launcher at individual RPMs

    Once a RPM is set in SmartDashboard, and the intake button is pressed, the launcher will be set to that RPM. Once that RPM is reached, the feeder and indexer will be powered

    Parameters:
        - demo_interface (`DemoInterface`): the oi test interface for controller bindings
        - flywheel (`FlywheelSubsystem`): the flywheel subsystem
        - indexer (`IndexerSubsystem`): the indexer subsystem
        - feeder (`FeederSubsystem`): the feeder subsystem
    """
    def __init__(self, demo_interface: DemoInterface, flywheel_subsystem: FlywheelSubsystem, indexer_subsystem: IndexerSubsystem, feeder_subsystem: FeederSubsystem, feed_target_subsystem: FeedTargetSubsystem) -> None:
        super().__init__()

        self._oi: DemoInterface = demo_interface
        self._flywheel_subsystem: FlywheelSubsystem = flywheel_subsystem
        self._indexer_subsystem: IndexerSubsystem = indexer_subsystem
        self._feeder_subsystem: FeederSubsystem = feeder_subsystem
        self._feed_target_subsystem: FeedTargetSubsystem = feed_target_subsystem

        if SmartDashboard.getNumber("Launcher RPM", 0) == 0:
            SmartDashboard.putNumber("Launcher RPM", 0)

        self.addRequirements(flywheel_subsystem)
    
    @override
    def execute(self) -> None:
        SmartDashboard.putNumber("Hub Distance", self._feed_target_subsystem.get_target(TargetType.HUB).turret_target.norm())

        if self._oi.demo_get_intake():
            self._flywheel_subsystem.set_speed(
                SC_SpeedRequest(
                    cast(revolutions_per_minute, SmartDashboard.getNumber("Launcher RPM", 0)), 
                    0
                )
            )

            if self._oi.demo_get_flywheel():
                self._indexer_subsystem.set_power(IndexerSubsystemConstants.INDEX_POWER)
                self._feeder_subsystem.set_velocity(FeederSubsystemConstants.FEED_SPEED)
        else:
            self._flywheel_subsystem.set_speed(SC_SpeedRequest(0, 0))
            self._indexer_subsystem.set_power(0)
            self._feeder_subsystem.set_velocity(FeederSpeed(SC_SpeedRequest(0, 0), SC_SpeedRequest(0, 0)))

    @override
    def end(self, interrupted: bool) -> None:
        self._flywheel_subsystem.set_speed(SC_SpeedRequest(0, 0))
        self._indexer_subsystem.set_power(0)
        self._feeder_subsystem.set_velocity(FeederSpeed(SC_SpeedRequest(0, 0), SC_SpeedRequest(0, 0)))

    @override
    def isFinished(self) -> bool:
        return False