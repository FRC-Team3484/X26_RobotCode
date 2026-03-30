
from typing import override
from commands2 import Command

from constants import FeederSubsystemConstants
from oi import OperatorInterface
from subsystems.feeder_subsystem import FeederSubsystem


class TeleopEjectCommand(Command):
    """
    Teleop command for ejecting a piece

    Parameters:
        - operator_interface (`OperatorInterface`): the oi test interface for controller bindings
        - feeder_subsystem (`FeederSubsystem`): the feeder subsystem
    """
    def __init__(self, operator_interface: OperatorInterface, feeder_subsystem: FeederSubsystem) -> None:
        super().__init__()

        self._operator_interface: OperatorInterface = operator_interface
        self._feeder_subsystem: FeederSubsystem = feeder_subsystem

        self.addRequirements(feeder_subsystem)

    def execute(self) -> None:
        """
        Ejects a piece when the eject button is pressed. Otherwise, stops the feeder
        """
        if self._operator_interface.get_eject():
            self._feeder_subsystem.set_velocity(FeederSubsystemConstants.REMOVE_PIECE_VELOCITY)
        else:
            self._feeder_subsystem.set_velocity(FeederSubsystemConstants.STOP_VELOCITY)
    
    def end(self, interrupted: bool) -> None:
        self._feeder_subsystem.set_velocity(FeederSubsystemConstants.STOP_VELOCITY)

    @override
    def isFinished(self) -> bool:
        return False