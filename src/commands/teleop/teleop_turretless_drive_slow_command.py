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
    Docstring for TeleopTurretlessDriveSlowCommand

    """
    def __init__(self, drivetrain: DrivetrainSubsystem, driver_oi: DriverInterface, no_turret: TurretlessLauncherSubsystem):
        super().__init__()
        self.addRequirements(drivetrain)

        self._drivetrain: DrivetrainSubsystem = drivetrain
        self._oi: DriverInterface = driver_oi
        self._turretless: TurretlessLauncherSubsystem = no_turret

        self._pivot_corner: Translation2d = Translation2d()

        self._alliance: DriverStation.Alliance = DriverStation.Alliance.kBlue

        self._throttle_filter: SlewRateLimiter = SlewRateLimiter(TeleopDriveConstants.SLEW_FILTER_AMOUNT)
        self._strafe_filter: SlewRateLimiter = SlewRateLimiter(TeleopDriveConstants.SLEW_FILTER_AMOUNT)
        self._rotation_pid: PIDController = PIDController(SwerveConstants.TURRETLESS_AIM_PID_VALUES.Kp, SwerveConstants.TURRETLESS_AIM_PID_VALUES.Ki, SwerveConstants.TURRETLESS_AIM_PID_VALUES.Kd)

    def initialize(self):
        alliance: DriverStation.Alliance | None = DriverStation.getAlliance()
        if alliance is None:
            print('Teleop Drive Command failed to determine alliance color')
        else:
            self._alliance = alliance
    def execute(self):
            if self._oi.get_reset_heading():
                self._drivetrain.set_heading()
            elif self._oi.get_hold_mode():
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

                if self._alliance == DriverStation.Alliance.kRed:
                    throttle = -throttle
                    strafe = -strafe
            
                self._drivetrain.drive(
                    throttle * TeleopDriveConstants.SLOW_SPEED, 
                    strafe * TeleopDriveConstants.SLOW_SPEED, 
                    max(
                        -1,
                        min(
                            self._rotation_pid.calculate(
                                self._drivetrain.get_heading().radians(),
                                self._turretless.turret_to_target.angle().radians(), 
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

