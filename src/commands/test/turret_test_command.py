from commands2 import Command
from subsystems.turret_subsystem import TurretSubsystem
from oi import TestInterface

class TurretTestCommand(Command):
    def __init__(self, oi: TestInterface, turret_subsystem: TurretSubsystem):
        super().__init__()
        self._turret_subsystem = turret_subsystem
        self._oi = oi
        self.addRequirements(turret_subsystem)

    def initialize(self):
        pass

    def execute(self):
        self._turret_subsystem.set_power(self._oi.get_turret())
        self._turret_subsystem.print_diagnostics()
    
    def end(self, interrupted: bool):
        
        self._turret_subsystem.set_power(0)

    def isFinished(self) -> bool:
        return False

