from commands2 import Command
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.filter import SlewRateLimiter

from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.subsystems.turretless_launcher_subsystem import TurretlessLauncherSubsystem
from src.constants import TeleopDriveConstants, SwerveConstants
from src.oi import DriverInterface

class TeleopTurretlessDriveSlowCommand(Command):
    """
    Drives the robot when the turret is disabled

    This slowly drives the robot with a slew filter, 
        and automatically aims the drivetrain when the launcher has a target

    Parameters:
        - drivetrain (`DrivetrainSubsystem`): the drivetrain subsystem
        - driver_oi (`DriverInterface`): the driver interface
        - turretless_launcher_subsystem (`TurretlessLauncherSubsystem`): the turretless launcher subsystem
    """
    def __init__(self, drivetrain: DrivetrainSubsystem, driver_oi: DriverInterface, turretless_launcher_subsystem: TurretlessLauncherSubsystem):
        super().__init__()
        self.addRequirements(drivetrain)

        self._drivetrain: DrivetrainSubsystem = drivetrain
        self._oi: DriverInterface = driver_oi
        self._launcher_subsystem: TurretlessLauncherSubsystem = turretless_launcher_subsystem

        self._pivot_corner: Translation2d = Translation2d()

        self._throttle_filter: SlewRateLimiter = SlewRateLimiter(TeleopDriveConstants.SLEW_FILTER_AMOUNT)
        self._strafe_filter: SlewRateLimiter = SlewRateLimiter(TeleopDriveConstants.SLEW_FILTER_AMOUNT)
        self._rotation_pid: PIDController = PIDController(SwerveConstants.TURRETLESS_AIM_PID_VALUES.Kp, SwerveConstants.TURRETLESS_AIM_PID_VALUES.Ki, SwerveConstants.TURRETLESS_AIM_PID_VALUES.Kd)

    def execute(self):
        """
        Drives the robot slowly

        Handles reset heading, hold mode, and turretless aim

        If the launcher has a target, point the drivetrain to the target
        """
        if self._oi.get_hold_mode():
            self._drivetrain.set_module_states((
                SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
                SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)),
                SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0))
            ), True, False)
        else:
            self._throttle_filter.calculate(self._oi.get_throttle())
            self._strafe_filter.calculate(self._oi.get_strafe())

            throttle: float = self._throttle_filter.lastValue()
            strafe: float = self._strafe_filter.lastValue()

            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                throttle = -throttle
                strafe = -strafe

            if self._launcher_subsystem.target is not None:
                self._drivetrain.drive(
                    throttle * TeleopDriveConstants.SLOW_SPEED, 
                    strafe * TeleopDriveConstants.SLOW_SPEED, 
                    max(
                        -1,
                        min(
                            self._rotation_pid.calculate(
                                self._drivetrain.get_heading().radians(),
                                self._launcher_subsystem.target.turret_target.angle().radians(), 
                            ), 
                            1
                        )
                    ) * TeleopDriveConstants.SLOW_ROTATION_SPEED, 
                    False
                )

    def end(self, interrupted: bool):
        self._drivetrain.stop_motors()

    def isFinished(self) -> bool:
        return False

