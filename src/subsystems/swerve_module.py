from typing import Literal

from phoenix6 import configs, controls
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.signals import NeutralModeValue, InvertedValue, SensorDirectionValue, FeedbackSensorSourceValue

from wpimath.units import meters, turns_per_second, volts, metersToFeet, metersToInches, inchesToMeters, rotationsToRadians, degreesToRotations, radiansToRotations
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d

from frc3484.datatypes import SC_SwerveConfig, SC_SwerveCurrentConfig, SC_DrivePIDConfig, SC_SteerPIDConfig

class SwerveModule:
    def __init__(self, config: SC_SwerveConfig, current_config: SC_SwerveCurrentConfig, drive_pid_config: SC_DrivePIDConfig, steer_pid_config: SC_SteerPIDConfig, canbus_name: str = "rio") -> None:
        '''
        Initializes a Swerve Module with the given configuration

        Parameters:
            - config (SC_SwerveConfig): Hardware CAN IDs and physical properties of the module
            - current_config (SC_SwerveCurrentConfig): Current limit settings for the drive and steer motors
            - drive_pid_config (SC_DrivePIDConfig): PID values for the drive motor
            - steer_pid_config (SC_SteerPIDConfig): PID values for the steer motor
            - canbus_name (str): The name of the CAN bus the motors and encoders are on (default: "rio")
        '''
        
        '''
        Motors and Encoders
        '''
        self._drive_motor: TalonFX = TalonFX(config.drive_can_id, canbus_name)
        self._steer_motor: TalonFX = TalonFX(config.steer_can_id, canbus_name)
        self._steer_encoder: CANcoder = CANcoder(config.encoder_can_id, canbus_name)

        self._drive_open_loop_request: controls.DutyCycleOut = controls.DutyCycleOut(0.0, enable_foc=False)
        self._drive_closed_loop_request: controls.VelocityVoltage = controls.VelocityVoltage(0.0, slot=0, enable_foc=False)
        self._steer_motor_request: controls.MotionMagicExpoVoltage = controls.MotionMagicExpoVoltage(0.0, slot=0, enable_foc=False)

        '''
        Motor and Encoder Configurations
        '''
        
        # Drive Motor Config
        self._drive_motor_config: configs.TalonFXConfiguration = configs.TalonFXConfiguration()
        self._drive_motor_config.current_limits = configs.CurrentLimitsConfigs() \
            .with_supply_current_limit_enable(current_config.current_limit_enabled) \
            .with_supply_current_limit(current_config.drive_current_limit) \
            .with_supply_current_lower_limit(current_config.drive_current_threshold) \
            .with_supply_current_lower_time(current_config.drive_current_time)
        
        # Configure pid
        self._drive_motor_config.slot0 = configs.Slot0Configs() \
            .with_k_p(drive_pid_config.Kp) \
            .with_k_i(drive_pid_config.Ki) \
            .with_k_d(drive_pid_config.Kd) \
            .with_k_v(drive_pid_config.V) \
            .with_k_a(drive_pid_config.A) \
            .with_k_s(drive_pid_config.S)

        self._drive_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = current_config.drive_open_loop_ramp
        self._drive_motor.configurator.apply(self._drive_motor_config)
        self.set_brake_mode()
        self.reset_encoder()

        # Steer Motor Config
        self._steer_motor_config: configs.TalonFXConfiguration = configs.TalonFXConfiguration()
        self._steer_motor_config.current_limits = configs.CurrentLimitsConfigs() \
            .with_supply_current_limit_enable(current_config.current_limit_enabled) \
            .with_supply_current_limit(current_config.steer_current_limit) \
            .with_supply_current_lower_limit(current_config.steer_current_threshold) \
            .with_supply_current_lower_time(current_config.steer_current_time)
        
        # Configure the steer motor to use the CANcoder as its feedback device
        self._steer_motor_config.feedback.feedback_remote_sensor_id = self._steer_encoder.device_id
        self._steer_motor_config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER
        self._steer_motor_config.feedback.rotor_to_sensor_ratio = config.steer_ratio
        self._steer_motor_config.closed_loop_general.continuous_wrap = True

        # Configure pid and motion magic
        self._steer_motor_config.slot0 = configs.Slot0Configs() \
            .with_k_p(steer_pid_config.Kp) \
            .with_k_i(steer_pid_config.Ki) \
            .with_k_d(steer_pid_config.Kd) \
            .with_k_v(steer_pid_config.V) \
            .with_k_a(steer_pid_config.A) \
            .with_k_s(steer_pid_config.S)
        self._steer_motor_config.motion_magic.motion_magic_expo_k_v = 0.12 * config.steer_ratio
        self._steer_motor_config.motion_magic.motion_magic_expo_k_a *= 0.8 / config.steer_ratio

        self._steer_motor_config.motor_output.inverted = InvertedValue(config.steer_motor_reversed)
        self._steer_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self._steer_motor.configurator.apply(self._steer_motor_config)

        # Encoder Config
        self._encoder_config: configs.CANcoderConfiguration = configs.CANcoderConfiguration()
        self._encoder_config.magnet_sensor = configs.MagnetSensorConfigs() \
            .with_magnet_offset(degreesToRotations(config.encoder_offset)) \
            .with_sensor_direction(SensorDirectionValue(config.encoder_reversed)) \
            .with_absolute_sensor_discontinuity_point(0.5)
        
        self._steer_encoder.configurator.apply(self._encoder_config)

        '''
        Constants
        '''
        self._WHEEL_RADIUS: meters = inchesToMeters(config.wheel_radius)
        self._DRIVE_GEAR_RATIO: float = config.drive_gear_ratio
        self._DRIVE_SCALING: float = config.drive_scaling

    def set_desired_state(self, state: SwerveModuleState, open_loop: bool = True, optimize: bool = True) -> None:
        '''
        Sets the desired state for the swerve module

        Parameters:
            - state (SwerveModuleState): The desired state (wheel speed and steer angle) for the module
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
            - optimize (bool): Whether to optimize the steering angle to minimize rotation
        '''
        encoder_rotation: Rotation2d = self._get_steer_angle()

        # If the wheel needs to rotate over 90 degrees, rotate the other direction and flip the output
        # This prevents the wheel from ever needing to rotate more than 90 degrees
        if optimize:
            state.optimize(encoder_rotation)

        # Scale the wheel speed down by the cosine of the angle error
        # This prevents the wheel from accelerating before it has a chance to face the correct direction
        state.speed *= (state.angle - encoder_rotation).cos()

        # In open loop, treat speed as a percent power
        # In closed loop, try to hit the actual speed
        if open_loop:
            self._drive_motor.set_control(self._drive_open_loop_request.with_output(state.speed))
        else:
            self._drive_motor.set_control(self._drive_closed_loop_request.with_velocity(radiansToRotations(state.speed / self._WHEEL_RADIUS) * self._DRIVE_GEAR_RATIO / self._DRIVE_SCALING))

        self._set_steer(state.angle)
    
    def _set_steer(self, angle: Rotation2d) -> None:
        '''
        Sets the steer motor to the specified angle

        Parameters:
            - angle (Rotation2d): The angle to set the steer motor to
        '''
        self._steer_motor.set_control(self._steer_motor_request.with_position(angle.degrees() / 360.0))

    def get_state(self) -> SwerveModuleState:
        '''
        Gets the current state of the swerve module
        '''
        return SwerveModuleState(self._get_wheel_speed('meters'), self._get_steer_angle())

    def get_position(self) -> SwerveModulePosition:
        '''
        Gets the current position of the swerve module
        '''
        return SwerveModulePosition(self._get_wheel_position('meters'), self._get_steer_angle())
    
    def _get_wheel_speed(self, distance_units: Literal['feet', 'meters'] = 'meters') -> float:
        '''
        Gets the current speed of the wheel

        Parameters:
            - distance_units (str): 'feet' or 'meters'

        Returns:
            The speed of the wheel in the specified distance units per second
        '''
        speed: meters = self._WHEEL_RADIUS * (rotationsToRadians(self._drive_motor.get_velocity().value) / self._DRIVE_GEAR_RATIO) * self._DRIVE_SCALING
        if distance_units == 'feet':
            return metersToFeet(speed)
        return speed

    def _get_wheel_position(self, distance_units: Literal['inches', 'feet', 'meters']) -> float:
        '''
        Gets the current position of the wheel

        Parameters:
            - distance_units (str): 'inches', 'feet', or 'meters'

        Returns:
            The position of the wheel in the specified distance units
        '''
        position: meters = self._WHEEL_RADIUS * (rotationsToRadians(self._drive_motor.get_position().value) / self._DRIVE_GEAR_RATIO) * self._DRIVE_SCALING
        if distance_units == 'inches':
            return metersToInches(position)
        elif distance_units == 'feet':
            return metersToFeet(position)
        return position

    def _get_steer_angle(self) -> Rotation2d:
        '''
        Gets the current angle of the steer encoder as a Rotation2d
        '''
        return Rotation2d(rotationsToRadians(self._steer_encoder.get_absolute_position().value))
    
    def get_steer_velocity(self) -> turns_per_second:
        '''
        Gets the current velocity of the steer encoder in rotations per second
        '''
        return self._steer_encoder.get_velocity().value

    def stop_motors(self) -> None:
        '''
        Stops both the drive and steer motors
        '''
        self._drive_motor.set(0)
        self._steer_motor.set(0)

    def reset_encoder(self) -> None:
        '''
        Resets the drive motor encoder to 0
        '''
        self._drive_motor.set_position(0)

    def set_coast_mode(self) -> None:
        '''
        Sets the drive motor to coast mode
        '''
        self._drive_motor_config.motor_output.neutral_mode = NeutralModeValue.COAST
        self._drive_motor.configurator.apply(self._drive_motor_config)

    def set_brake_mode(self) -> None:
        '''
        Sets the drive motor to brake mode
        '''
        self._drive_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self._drive_motor.configurator.apply(self._drive_motor_config)

    def set_drive_voltage(self, voltage: volts) -> None:
        '''
        Sets the drive motor to the specified voltage

        Used by SysId routines

        Parameters:
            - voltage (volts): The voltage to set the drive motor to
        '''
        self._drive_motor.setVoltage(voltage)
        self._set_steer(Rotation2d(0)) # Point the wheel forward when using voltage control
    def set_steer_voltage(self, voltage: volts) -> None:
        '''
        Sets the steer motor to the specified voltage

        Used by SysId routines

        Parameters:
            - voltage (volts): The voltage to set the steer motor to
        '''
        self._drive_motor.setVoltage(0) # Stop the drive motor when using voltage control
        self._steer_motor.setVoltage(voltage)

    def get_voltages(self) -> dict[str, volts]:
        '''
        Gets the current voltages of the drive and steer motors

        Returns:
            A tuple containing the drive motor voltage and steer motor voltage
        '''
        return {'drive': self._drive_motor.get_motor_voltage().value, 'steer': self._steer_motor.get_motor_voltage().value}