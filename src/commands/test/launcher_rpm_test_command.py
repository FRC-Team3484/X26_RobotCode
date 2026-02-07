from typing import cast, override

from wpilib import SmartDashboard
from wpimath.units import revolutions_per_minute
from commands2 import Command
from frc3484.motion import SC_LauncherSpeed

from oi import DemoInterface
from constants import IndexerSubsystemConstants, FeederSubsystemConstants
from subsystems.feeder_subsystem import FeederSubsystem
from subsystems.flywheel_subsystem import FlywheelSubsystem
from subsystems.indexer_subsystem import IndexerSubsystem


class LauncherRpmTestCommand(Command):
    """
    Command for testing the launcher at individual RPMs

    Once a RPM is set in SmartDashboard, and the intake button is pressed, the launcher will be set to that RPM. Once that RPM is reached, the feeder and indexer will be powered

    Parameters:
        - test_interface (`TestInterface`): the oi test interface for controller bindings
        - flywheel (`FlywheelSubsystem`): the flywheel subsystem
        - indexer (`IndexerSubsystem`): the indexer subsystem
        - feeder (`FeederSubsystem`): the feeder subsystem
    """
    def __init__(self, demo_interface: DemoInterface, flywheel_subsystem: FlywheelSubsystem, indexer_subsystem: IndexerSubsystem, feeder_subsystem: FeederSubsystem) -> None:
        super().__init__()

        self._oi: DemoInterface = demo_interface
        self._flywheel_subsystem: FlywheelSubsystem = flywheel_subsystem
        self._indexer_subsystem: IndexerSubsystem = indexer_subsystem
        self._feeder_subsystem: FeederSubsystem = feeder_subsystem

        SmartDashboard.putNumber("Launcher RPM", 0)

        self.addRequirements(flywheel_subsystem)
    
    @override
    def execute(self) -> None:
        if self._oi.demo_get_intake():
            self._flywheel_subsystem.set_speed(
                SC_LauncherSpeed(
                    cast(revolutions_per_minute, SmartDashboard.getNumber("Launcher RPM", 0)), 
                    0
                ))

            if self._flywheel_subsystem.is_at_speed():
                self._indexer_subsystem.set_power(IndexerSubsystemConstants.INDEX_POWER)
                self._feeder_subsystem.set_velocity(FeederSubsystemConstants.FEED_VELOCITY)
        else:
            self._flywheel_subsystem.set_speed(SC_LauncherSpeed(0, 0))
            self._indexer_subsystem.set_power(0)
            self._feeder_subsystem.set_velocity(SC_LauncherSpeed(0, 0))

    @override
    def end(self, interrupted: bool) -> None:
        self._flywheel_subsystem.set_speed(SC_LauncherSpeed(0, 0))
        self._indexer_subsystem.set_power(0)
        self._feeder_subsystem.set_velocity(SC_LauncherSpeed(0, 0))

    @override
    def isFinished(self) -> bool:
        return False