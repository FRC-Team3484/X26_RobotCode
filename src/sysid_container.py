from typing import Literal

from commands2 import Command, SelectCommand
from commands2.sysid import SysIdRoutine

from src.oi import SysIDInterface
from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.subsystems.feeder_subsystem import FeederSubsystem
from src.subsystems.flywheel_subsystem import FlywheelSubsystem
from src.subsystems.turret_subsystem import TurretSubsystem


class SysIDContainer():
    """
    Handles SysID commands
    """
    def __init__(self, 
            oi: SysIDInterface, 
            drivetrain_subsystem: DrivetrainSubsystem | None = None, 
            flywheel_subsystem: FlywheelSubsystem | None = None, 
            feeder_subsystem: FeederSubsystem | None = None,
            turret_subsystem: TurretSubsystem | None = None
        ) -> None:
        self._oi: SysIDInterface = oi
        self._drivetrain_subsystem: DrivetrainSubsystem | None = drivetrain_subsystem
        self._flywheel_subsystem: FlywheelSubsystem | None = flywheel_subsystem
        self._feeder_subsystem: FeederSubsystem | None = feeder_subsystem
        self._turret_subsystem: TurretSubsystem | None = turret_subsystem

    def get_drivetrain_sysid(self, motor: Literal['drive', 'steer']) -> Command:
        """
        Returns a command group that runs the drivetrain drive SysID based on the buttons that are pressed
        """
        if self._drivetrain_subsystem is not None:
            quasistatic_forward: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'quasistatic', SysIdRoutine.Direction.kForward)
            quasistatic_reverse: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'quasistatic', SysIdRoutine.Direction.kReverse)
            dynamic_forward: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'dynamic', SysIdRoutine.Direction.kForward)
            dynamic_reverse: Command = self._drivetrain_subsystem.get_sysid_command(motor, 'dynamic', SysIdRoutine.Direction.kReverse)

            return SelectCommand(
                {
                    'quasistatic_forward': quasistatic_forward,
                    'quasistatic_reverse': quasistatic_reverse,
                    'dynamic_forward': dynamic_forward,
                    'dynamic_reverse': dynamic_reverse
                },
                lambda: 'quasistatic_forward' if self._oi.get_quasistatic_forward() else (
                    'quasistatic_reverse' if self._oi.get_quasistatic_reverse() else (
                        'dynamic_forward' if self._oi.get_dynamic_forward() else (
                            'dynamic_reverse' if self._oi.get_dynamic_reverse() else None
                        )
                    )
                )
            ).onlyWhile(
                lambda: self._oi.get_quasistatic_forward()
                    or self._oi.get_quasistatic_reverse()
                    or self._oi.get_dynamic_forward()
                    or self._oi.get_dynamic_reverse()) \
            .repeatedly()
        else:
            print("[SysID Container] Unable to return drivetrain SysID commands because DrivetrainSubsystem is None")
            return Command()

    def get_flywheel_sysid(self) -> Command:
        """
        Returns a command group that runs the flywheel SysID based on the buttons that are pressed
        """
        if self._flywheel_subsystem is not None:
            quasistatic_forward: Command = self._flywheel_subsystem.get_sysid_command('quasistatic', SysIdRoutine.Direction.kForward)
            quasistatic_reverse: Command = self._flywheel_subsystem.get_sysid_command('quasistatic', SysIdRoutine.Direction.kReverse)
            dynamic_forward: Command = self._flywheel_subsystem.get_sysid_command('dynamic', SysIdRoutine.Direction.kForward)
            dynamic_reverse: Command = self._flywheel_subsystem.get_sysid_command('dynamic', SysIdRoutine.Direction.kReverse)

            return SelectCommand(
                {
                    'quasistatic_forward': quasistatic_forward,
                    'quasistatic_reverse': quasistatic_reverse,
                    'dynamic_forward': dynamic_forward,
                    'dynamic_reverse': dynamic_reverse
                },
                lambda: 'quasistatic_forward' if self._oi.get_quasistatic_forward() else (
                    'quasistatic_reverse' if self._oi.get_quasistatic_reverse() else (
                        'dynamic_forward' if self._oi.get_dynamic_forward() else (
                            'dynamic_reverse' if self._oi.get_dynamic_reverse() else None
                        )
                    )
                )
            ).onlyWhile(
                lambda: self._oi.get_quasistatic_forward()
                    or self._oi.get_quasistatic_reverse()
                    or self._oi.get_dynamic_forward()
                    or self._oi.get_dynamic_reverse()) \
            .repeatedly()
        else:
            print("[SysID Container] Unable to return flywheel SysID commands because FlywheelSubsystem is None")
            return Command()

    def get_feeder_sysid(self, motor: Literal['pull', 'push']) -> Command:
        """
        Returns a command group that runs the feeder SysID based on the buttons that are pressed
        """
        if self._feeder_subsystem is not None:
            quasistatic_forward: Command = self._feeder_subsystem.get_sysid_command(motor, 'quasistatic', SysIdRoutine.Direction.kForward)
            quasistatic_reverse: Command = self._feeder_subsystem.get_sysid_command(motor, 'quasistatic', SysIdRoutine.Direction.kReverse)
            dynamic_forward: Command = self._feeder_subsystem.get_sysid_command(motor, 'dynamic', SysIdRoutine.Direction.kForward)
            dynamic_reverse: Command = self._feeder_subsystem.get_sysid_command(motor, 'dynamic', SysIdRoutine.Direction.kReverse)

            return SelectCommand(
                {
                    'quasistatic_forward': quasistatic_forward,
                    'quasistatic_reverse': quasistatic_reverse,
                    'dynamic_forward': dynamic_forward,
                    'dynamic_reverse': dynamic_reverse
                },
                lambda: 'quasistatic_forward' if self._oi.get_quasistatic_forward() else (
                    'quasistatic_reverse' if self._oi.get_quasistatic_reverse() else (
                        'dynamic_forward' if self._oi.get_dynamic_forward() else (
                            'dynamic_reverse' if self._oi.get_dynamic_reverse() else None
                        )
                    )
                )
            ).onlyWhile(
                lambda: self._oi.get_quasistatic_forward()
                    or self._oi.get_quasistatic_reverse()
                    or self._oi.get_dynamic_forward()
                    or self._oi.get_dynamic_reverse()) \
            .repeatedly()
        else:
            print("[SysID Container] Unable to return feeder SysID commands because FeederSubsystem is None")
            return Command()

    def get_turret_sysid(self) -> Command:
        """
        Returns a command group that runs the turret SysID based on the buttons that are pressed
        """
        if self._turret_subsystem is not None:
            quasistatic_forward: Command = self._turret_subsystem.get_sysid_command('quasistatic', SysIdRoutine.Direction.kForward)
            quasistatic_reverse: Command = self._turret_subsystem.get_sysid_command('quasistatic', SysIdRoutine.Direction.kReverse)
            dynamic_forward: Command = self._turret_subsystem.get_sysid_command('dynamic', SysIdRoutine.Direction.kForward)
            dynamic_reverse: Command = self._turret_subsystem.get_sysid_command('dynamic', SysIdRoutine.Direction.kReverse)

            return SelectCommand(
                {
                    'quasistatic_forward': quasistatic_forward,
                    'quasistatic_reverse': quasistatic_reverse,
                    'dynamic_forward': dynamic_forward,
                    'dynamic_reverse': dynamic_reverse
                },
                lambda: 'quasistatic_forward' if self._oi.get_quasistatic_forward() else (
                    'quasistatic_reverse' if self._oi.get_quasistatic_reverse() else (
                        'dynamic_forward' if self._oi.get_dynamic_forward() else (
                            'dynamic_reverse' if self._oi.get_dynamic_reverse() else None
                        )
                    )
                )
            ).onlyWhile(
                lambda: self._oi.get_quasistatic_forward()
                    or self._oi.get_quasistatic_reverse()
                    or self._oi.get_dynamic_forward()
                    or self._oi.get_dynamic_reverse()) \
            .repeatedly()
        else:
            print("[SysID Container] Unable to return feeder SysID commands because FeederSubsystem is None")
            return Command()