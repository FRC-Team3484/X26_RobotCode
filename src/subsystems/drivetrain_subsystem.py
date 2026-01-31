import sys
from typing import Literal

from phoenix6.hardware import Pigeon2
from phoenix6.configs import Pigeon2Configuration

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig

from wpimath.units import radians_per_second, meters_per_second, degreesToRadians, volts
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpilib import SmartDashboard, Field2d, DriverStation
from wpilib.sysid import SysIdRoutineLog

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine

from frc3484.vision import SC_Vision
from src.subsystems.swerve_module import SwerveModule
from src.constants import SwerveConstants
from src.oi import OperatorInterface

class DrivetrainSubsystem(Subsystem):
    ERROR_TIMEOUT: int = 100 # Number of periodic cycles to wait between error messages during competition
    def __init__(self, oi: OperatorInterface | None, vision: SC_Vision | None) -> None:
        '''
        Swerve drivetrain subsystem

        Drivetrain configs are pulled directly from SwerveConstants

        Parameters:
            - vision: Vision subsystem (optional, for pose correction)
            - oi: Operator Interface subsystem (optional, for disabling vision)
        '''
        super().__init__()

        self._modules: list[SwerveModule] = [
            SwerveModule(
                SwerveConstants.MODULE_CONFIGS[i], 
                SwerveConstants.MODULE_CURRENTS[i],
                SwerveConstants.DRIVE_PID_CONFIGS[i],
                SwerveConstants.STEER_PID_CONFIGS[i],
                SwerveConstants.CANBUS_NAME
                ) 
            for i in range(len(SwerveConstants.MODULE_CONFIGS))]
        
        self._kinematics:SwerveDrive4Kinematics = SwerveDrive4Kinematics(*SwerveConstants.MODULE_POSITIONS)
        self._kinematics.resetHeadings((self._modules[0].get_position().angle, self._modules[1].get_position().angle, self._modules[2].get_position().angle, self._modules[3].get_position().angle))

        self._pigeon: Pigeon2 = Pigeon2(SwerveConstants.PIGEON_ID, SwerveConstants.CANBUS_NAME)
        self._pigeon.configurator.apply(Pigeon2Configuration())
        self._pigeon_offset: Rotation2d = Rotation2d()

        self._odometry: SwerveDrive4PoseEstimator = SwerveDrive4PoseEstimator(
            self._kinematics,
            self.get_heading(),
            self.get_module_positions(),
            Pose2d()
        )

        self._vision: SC_Vision | None = vision
        self._oi: OperatorInterface | None = oi
        
        self._target_position: Pose2d = Pose2d()

        # SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
        sys_id_routine_translation: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                lambda voltage: self._drive_volts(voltage),
                lambda log: None,
                self,
            ),
        )
        # SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
        sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0
            ),
            SysIdRoutine.Mechanism(
                lambda voltage: self._steer_volts(voltage),
                lambda _: None,
                self,
            ),
        )

        self._sysid_routines: dict[str, SysIdRoutine] = {
            'drive': sys_id_routine_translation,
            'steer': sys_id_routine_steer
        }

        self._robot_config = RobotConfig.fromGUISettings()
        if not AutoBuilder.isConfigured():
            AutoBuilder.configure(
                self.get_pose,
                self.reset_odometry,
                self.get_chassis_speeds,
                lambda speeds, _: self.drive_robotcentric(speeds, open_loop=False), # Pathplanner has added a parameter for module feedforwards but doesn't have an example in any language that uses it
                SwerveConstants.DRIVE_CONTROLLER,
                self._robot_config,
                lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
                self
            )

        self._field: Field2d = Field2d()
        SmartDashboard.putData('Field', self._field)
        SmartDashboard.putBoolean('Drivetrain Diagnostics', False)

        self._last_error: int = 0

    def periodic(self) -> None:
        '''
        - Updates the odometry of the drivetrain
        - If there are vision results avaliable, updates the odometry using them instead
        - Updates the field visualization on SmartDashboard
        - Outputs diagnostic information to SmartDashboard if enabled
        '''

        if self._last_error > 0:
            self._last_error -= 1

        self._odometry.update(
            self.get_heading(),
            self.get_module_positions()
        )

        if self._vision is not None:
            if self._oi is not None and self._oi.get_ignore_vision():
                try:
                    for result in self._vision.get_camera_results(self.get_pose()):
                        new_std_devs: tuple[float, float, float] = result.standard_deviation
                        self._odometry.addVisionMeasurement(
                            result.vision_measurement,
                            result.timestamp,
                            new_std_devs
                        )
                except Exception as e:
                    self._throw_error("Error getting vision results", e)

        self._field.setRobotPose(self.get_pose())
        self._field.getObject('Target Position').setPose(self._target_position)

        if SmartDashboard.getBoolean('Drivetrain Diagnostics', False):
            SmartDashboard.putNumber('Drivetrain Heading', self.get_heading().degrees())
            pose: Pose2d = self.get_pose()
            SmartDashboard.putNumber('Drivetrain X Position', pose.X())
            SmartDashboard.putNumber('Drivetrain Y Position', pose.Y())
            SmartDashboard.putNumber('FL Encoder', self._modules[SwerveConstants.FL].get_position().distance)
            SmartDashboard.putNumber('FR Encoder', self._modules[SwerveConstants.FR].get_position().distance)
            SmartDashboard.putNumber('BL Encoder', self._modules[SwerveConstants.BL].get_position().distance)
            SmartDashboard.putNumber('BR Encoder', self._modules[SwerveConstants.BR].get_position().distance)

    def drive(self, x_speed: meters_per_second, y_speed: meters_per_second, rot_speed: radians_per_second, open_loop: bool) -> None:
        '''
        Converts field-relative speeds to robot-centric speeds

        Used by teleop control

        Parameters:
            - x_speed (meters_per_second): The desired speed in the x direction (forward)
            - y_speed (meters_per_second): The desired speed in the y direction (sideways)
            - rot_speed (radians_per_second): The desired rotational speed
            - open_loop (bool):
                True: treat speed as a percent power from -1.0 to 1.0
                False: treat speed as a velocity in meters per second
        '''
        speeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            self.get_heading()
        )
        self.drive_robotcentric(speeds, open_loop)

    def drive_robotcentric(self, speeds: ChassisSpeeds, open_loop: bool) -> None:
        '''
        Converts robot-centric speeds to module states and commands the modules to drive

        Used by the autonomous drive controller

        Parameters:
            - speeds (ChassisSpeeds): The desired chassis speeds
            - open_loop (bool):
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
        '''
        states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState] = self._kinematics.toSwerveModuleStates(speeds)
        self.set_module_states(states, open_loop, optimize=True)

    def dynamic_pivot_drive(self, x_speed: meters_per_second, y_speed: meters_per_second, rot_speed: radians_per_second, center_of_rotation: Translation2d, open_loop: bool) -> None:
        '''
        Converts field-centric speeds to robot-centric speeds with a specified center of rotation

        Parameters:
            - x_speed (meters_per_second): The desired speed in the x direction (forward)
            - y_speed (meters_per_second): The desired speed in the y direction (sideways)
            - rot_speed (radians_per_second): The desired rotational speed
            - center_of_rotation (Translation2d): The point around which the robot should rotate
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
        '''
        speeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            self.get_heading()
        )
        self.dynamic_pivot_drive_robotcentric(speeds, center_of_rotation, open_loop)

    def dynamic_pivot_drive_robotcentric(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d, open_loop: bool) -> None:
        '''
        Drives the robot with a specified center of rotation

        Parameters:
            - speeds (ChassisSpeeds): The desired chassis speeds
            - center_of_rotation (Translation2d): The point around which the robot should rotate
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
        '''
        states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState] = self._kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        self.set_module_states(states, open_loop, optimize=True)

    def apply_double_cone_desaturation(self, tx, ty, r) -> tuple[float, float, float]:

        rotation = abs(r)
        translation = (tx**2, ty**2)**0.5

        scaling = translation + rotation
        if scaling > 1:
            tx /= scaling
            ty /= scaling
            r /= scaling
        return tuple(tx, ty, r)

    def set_module_states(self, desired_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState], open_loop: bool, optimize: bool) -> None:
        '''
        Sets the desired states (wheel speeds and steer angles) for all drivetrain modules
        Parameters:
            - desired_states (tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]): The desired states for all modules
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
            - optimize (bool): Whether to optimize the steering angles to minimize rotation
        '''
        chassis_speeds = self._kinematics.toChassisSpeeds(desired_states)

        if open_loop:
            tx, ty, r = self.apply_double_cone_desaturation(chassis_speeds.vx, chassis_speeds.vy, chassis_speeds.omega)
        else:
            tx, ty, r = self.apply_double_cone_desaturation(
                chassis_speeds.vx / SwerveConstants.MAX_WHEEL_SPEED,
                chassis_speeds.vy / SwerveConstants.MAX_WHEEL_SPEED,
                chassis_speeds.omega / SwerveConstants.MAX_ROTATION_SPEED
                )
            tx *= SwerveConstants.MAX_WHEEL_SPEED
            ty *= SwerveConstants.MAX_WHEEL_SPEED
            r *= SwerveConstants.MAX_ROTATION_SPEED
        states = self._kinematics.toSwerveModuleStates(ChassisSpeeds(tx, ty, r))

        for module, state in zip(self._modules, states):
            module.set_desired_state(state, open_loop, optimize)


    def get_heading(self) -> Rotation2d:
        '''
        Gets the current heading of the robot

        0 degrees is towards the red alliance wall, increasing clockwise
        '''
        return self._pigeon.getRotation2d().rotateBy(self._pigeon_offset)

    def set_heading(self, heading: Rotation2d = Rotation2d()) -> None:
        '''
        Sets the current heading of the robot to a specified value without changing its position

        Parameters:
            - heading (Rotation2d): The new heading of the robot
        '''
        self.reset_odometry(Pose2d(self._odometry.getEstimatedPosition().translation(), heading))

    def get_turn_rate(self) -> radians_per_second:
        '''
        Gets the current turn rate of the robot
        '''
        return degreesToRadians(-self._pigeon.get_angular_velocity_z_world().value)

    def get_pose(self) -> Pose2d:
        '''
        Gets the current estimated location of the robot on the field
        '''
        return self._odometry.getEstimatedPosition()
    
    def get_velocity(self) -> tuple[meters_per_second, meters_per_second]:

        chassis_speeds = self.get_chassis_speeds()
        velocity_translation = Translation2d(chassis_speeds.vx, chassis_speeds.vy)
        velocity_translation.rotateBy(self.get_heading())
        return (velocity_translation.X(), velocity_translation.Y())

    def reset_odometry(self, pose: Pose2d) -> None:
        '''
        Resets the drivetrain odometry to a specified pose

        Parameters:
            - pose (Pose2d): The new pose of the robot
        '''
        self._pigeon_offset = pose.rotation() - self._pigeon.getRotation2d()
        self._odometry.resetPosition(
            self.get_heading(),
            self.get_module_positions(),
            pose
        )

    def get_module_positions(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        '''
        Gets the current positions of all drivetrain modules
        '''
        return (
            self._modules[0].get_position(), 
            self._modules[1].get_position(), 
            self._modules[2].get_position(), 
            self._modules[3].get_position()
        )

    def get_chassis_speeds(self) -> ChassisSpeeds:
        '''
        Gets the current chassis speeds of the drivetrain
        '''
        return self._kinematics.toChassisSpeeds((
            self._modules[0].get_state(), 
            self._modules[1].get_state(), 
            self._modules[2].get_state(), 
            self._modules[3].get_state()
        ))

    def stop_motors(self) -> None:
        '''
        Stops all drivetrain motors (drive and steer)
        '''
        for module in self._modules:
            module.stop_motors()

    def reset_encoders(self) -> None:
        '''
        Resets all drivetrain module encoder positions to 0
        '''
        for module in self._modules:
            module.reset_encoder()

    def set_coast_mode(self) -> None:
        '''
        Sets all drivetrain motors to coast mode
        '''
        for module in self._modules:
            module.set_coast_mode()

    def set_brake_mode(self) -> None:
        '''
        Sets all drivetrain motors to brake mode
        '''
        for module in self._modules:
            module.set_brake_mode()

    def set_target_position(self, pose: Pose2d) -> None:
        '''
        Sets the target position for visualization on SmartDashboard

        This is purely for visualization and does not affect robot behavior

        Parameters:
            - pose (Pose2d): Where to show the target position on the field
        '''
        self._target_position = pose

    def _throw_error(self, message:str, error:Exception) -> None:
        '''
        Throw an error if an issue occurs while getting an input.
        Only halt execution if not in competition.
        '''
        if DriverStation.isFMSAttached() or 'pytest' in sys.modules:
            # Don't spam errors
            if self._last_error == 0:
                print(message)
                print(error)
                self._last_error = self.ERROR_TIMEOUT
        else:
            print(message)
            raise error
        
    def _drive_volts(self, voltage: volts) -> None:
        '''
        Drives the robot forward at a specified voltage

        Used by SysId routines

        Parameters:
            - voltage (volts): The voltage to apply to the drive motors
        '''
        for module in self._modules:
            module.set_drive_voltage(voltage)
    def _steer_volts(self, voltage: volts) -> None:
        '''
        Sets all modules to face forward at a specified voltage

        Used by SysId routines

        Parameters:
            - voltage (volts): The voltage to apply to the steer motors
        '''
        for module in self._modules:
            module.set_steer_voltage(voltage)

    def _log_motors(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the motor outputs for SysId routines

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        for i, module in enumerate(self._modules):
            log.motor(f'drive {i}') \
                .voltage(module.get_voltages()['drive']) \
                .position(module.get_position().distance) \
                .velocity(module.get_state().speed)
            
            log.motor(f'steer {i}') \
                .voltage(module.get_voltages()['steer']) \
                .angularPosition(module.get_position().angle.degrees() / 360.0) \
                .angularVelocity(module.get_steer_velocity())

    def get_sysid_command(self, motor: Literal['drive', 'steer'], mode: Literal['quasistatic', 'dynamic'], direction: SysIdRoutine.Direction) -> Command:
        '''
        Returns the SysId routine command for the specified motor and mode

        Parameters:
            - motor (Literal['drive', 'steer']): The motor to characterize
            - mode (Literal['quasistatic', 'dynamic']): The characterization mode
            - direction (SysIdRoutine.Direction): The direction to characterize (translation or steer)
        '''
        if motor not in self._sysid_routines:
            raise ValueError(f'Invalid motor for SysId: {motor}')

        if mode == 'quasistatic':
            return self._sysid_routines[motor].quasistatic(direction)
        elif mode == 'dynamic':
            return self._sysid_routines[motor].dynamic(direction)
        else:
            raise ValueError(f'Invalid mode for SysId: {mode}')