from typing import Literal

from commands2 import Command, ParallelCommandGroup
from commands2.sysid import SysIdRoutine

from oi import TestInterface
from subsystems.drivetrain_subsystem import DrivetrainSubsystem


class SysIDContainer():
    """
    Handles SysID commands
    """
    def __init__(self, oi: TestInterface, drivetrain_subsystem: DrivetrainSubsystem | None = None) -> None:
        self._oi: TestInterface = oi
        self._drivetrain_subsystem: DrivetrainSubsystem | None = drivetrain_subsystem

    def get_drivetrain_sysid(self, motor: Literal['drive', 'steer']) -> Command:
        """
        Returns a command group that runs the drivetrain drive SysID based on the buttons that are pressed
        """
        if self._drivetrain_subsystem is not None:
            quasistatic_forward: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'quasistatic', SysIdRoutine.Direction.kForward)
            quasistatic_reverse: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'quasistatic', SysIdRoutine.Direction.kReverse)
            dynamic_forward: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'dynamic', SysIdRoutine.Direction.kForward)
            dynamic_reverse: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'dynamic', SysIdRoutine.Direction.kReverse)

            return ParallelCommandGroup(
                quasistatic_forward.onlyWhile(self._oi.get_quasistatic_forward),
                quasistatic_reverse.onlyWhile(self._oi.get_quasistatic_reverse),
                dynamic_forward.onlyWhile(self._oi.get_dynamic_forward),
                dynamic_reverse.onlyWhile(self._oi.get_dynamic_reverse)
            ).repeatedly()
        else:
            print("[SysID Container] Unable to return drivetrain SysID commands because DrivetrainSubsystem is None")
            return Command()

    def get_flywheel_sysid(self) -> Command:
        """
        Returns a command group that runs the flywheel SysID based on the buttons that are pressed
        """
        # TODO: Add flywheel SysID
        return Command()

    def get_feeder_sysid(self) -> Command:
        """
        Returns a command group that runs the feeder SysID based on the buttons that are pressed
        """
        # TODO: Add feeder SysID
        return Command()