from typing import override
from commands2 import Command
from subsystems.turret_subsystem import TurretSubsystem
from oi import TestInterface

class TurretTestCommand(Command):
    """
    A command for testing the turret subsystem

    Based on joystick input, will set the angle of the turret

    Parameters:
        - oi (`oi.TestInterface`): the oi test interface for controller bindings
        - turret_subsystem (`TurretSubsystem`): the turret subsystem
    """
    def __init__(self, oi: TestInterface, turret_subsystem: TurretSubsystem) -> None:
        super().__init__()
        self._turret_subsystem: TurretSubsystem = turret_subsystem
        self._oi: TestInterface = oi
        self.addRequirements(turret_subsystem)

    @override
    def initialize(self) -> None:
        pass

    @override
    def execute(self) -> None:
        self._turret_subsystem.set_power(self._oi.get_turret())
        self._turret_subsystem.print_diagnostics()
    
    @override
    def end(self, interrupted: bool) -> None:
        self._turret_subsystem.set_power(0)

    @override
    def isFinished(self) -> bool:
        return False

