from phoenix6.units import rotation
from typing import TypeAlias
from math import floor, ceil, gcd

from wpimath.geometry import Translation2d
from wpimath.units import degrees, inchesToMeters, turns
from wpilib import SmartDashboard
from commands2 import Subsystem
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration, MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue

from frc3484.motion import ExpoMotor
from src.constants import TurretSubsystemConstants

# Custom datatype for gear teeth
teeth: TypeAlias = float

def wrap_range(x: float, lo: float, hi: float) -> float:
    """
    Wrap x to the range [lo, hi]

    Parameters:
        a (float): The value to wrap
        lo (float): The lower bound of the range
        hi (float): The upper bound of the range
    """
    return max(lo, min(hi, x))

def nearest_int(x: float) -> int:
    """
    Rounds x to the nearest integer

    Parameters:
        x (float): The value to round
    """
    return int(floor(x + 0.5)) if x >= 0 else int(ceil(x - 0.5))

def mod_teeth(residual_teeth: teeth, mod_teeth_count: teeth) -> teeth:
    """
    Modulo the number of teeth

    Parameters:
        residual_teeth (float): The number of teeth to modulo
        mod_teeth_count (float): The number of teeth to modulo by
    """
    m: float = float(mod_teeth_count)
    return residual_teeth - m * floor(residual_teeth / m)

def lcm(a: int, b: int) -> int:
    """
    Compute the least common multiple of two integers

    Parameters:
        a (int): The first integer
        b (int): The second integer
    """
    return abs(a * b) // gcd(a, b)

class TurretSubsystem(Subsystem):
    """
    The turret subsystem

    Handles aiming the turret to a desired target, and uses Chinese Remainder Theorem to compute the turret angle using two encoders
    """
    def __init__(self)-> None:
        super().__init__()

        # Create motor and encoders
        self._motor: ExpoMotor = ExpoMotor(
            TurretSubsystemConstants.MOTOR_CONFIG, 
            TurretSubsystemConstants.PID_CONFIG, 
            TurretSubsystemConstants.FEED_FORWARD_CONFIG,
            TurretSubsystemConstants.EXPO_CONFIG, 
            0, 
            TurretSubsystemConstants.GEAR_RATIO,
            self._encoder_a
        )

        self._encoder_a: CANcoder = CANcoder(TurretSubsystemConstants.ENCODER_A_CAN_ID, TurretSubsystemConstants.ENCODER_A_CAN_BUS_NAME)
        self._encoder_b: CANcoder = CANcoder(TurretSubsystemConstants.ENCODER_B_CAN_ID, TurretSubsystemConstants.ENCODER_B_CAN_BUS_NAME)

        # Create encoder configs
        self._encoder_a_config: CANcoderConfiguration = CANcoderConfiguration()
        self._encoder_a_config.magnet_sensor = MagnetSensorConfigs() \
            .with_magnet_offset(TurretSubsystemConstants.ENCODER_A_OFFSET) \
            .with_sensor_direction(SensorDirectionValue(TurretSubsystemConstants.ENCODER_A_REVERSED)) \
            .with_absolute_sensor_discontinuity_point(TurretSubsystemConstants.ENCODER_A_ABSOLUTE_DISCONTINUITY_POINT)

        self._encoder_b_config: CANcoderConfiguration = CANcoderConfiguration()
        self._encoder_b_config.magnet_sensor = MagnetSensorConfigs() \
            .with_magnet_offset(TurretSubsystemConstants.ENCODER_B_OFFSET) \
            .with_sensor_direction(SensorDirectionValue(TurretSubsystemConstants.ENCODER_B_REVERSED)) \
            .with_absolute_sensor_discontinuity_point(TurretSubsystemConstants.ENCODER_B_ABSOLUTE_DISCONTINUITY_POINT)

        _ = self._encoder_a.configurator.apply(self._encoder_a_config)
        _ = self._encoder_b.configurator.apply(self._encoder_b_config)

        # Variables
        self._target: Translation2d = Translation2d()
        self._tolerance: degrees = 0
        
        self._encoder_a_gear_ratio: float = TurretSubsystemConstants.TEETH_TURRET / TurretSubsystemConstants.TEETH_A
        self._encoder_b_gear_ratio: float = TurretSubsystemConstants.TEETH_TURRET / TurretSubsystemConstants.TEETH_B

        self._lcm_teeth: teeth = lcm(TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_B)
        self._max_abs_range: turns = self._lcm_teeth / TurretSubsystemConstants.TEETH_TURRET

        self._looping: bool = False

        self._min_angle: turns = TurretSubsystemConstants.MINIMUM_ANGLE / 360
        self._max_angle: turns = TurretSubsystemConstants.MAXIMUM_ANGLE / 360

        # Startup functions
        self._sanitize_range()
        self._startup_seed_relative_from_absolute()

    def aim(self, target: Translation2d) -> None:
        """
        Aims the turret to a target

        Parameters:
            target (Translation2d): The target to aim to
        """
        def get_chosen_angle(current_angle: turns) -> turns:
            """
            Returns the angle to aim to based on the current angle

            Parameters:
                current_angle (turns): The current angle

            Returns:
                turns: The angle to aim to
            """
            chosen_angle: turns | None = self._choose_nearest_equivalent_in_range(target.angle().degrees() / 360, current_angle)

            if chosen_angle is None:
                return wrap_range(target.angle().degrees(), self._min_angle, self._max_angle)

            return chosen_angle

        self._target = target
        self._tolerance = Translation2d(target.norm(), inchesToMeters(TurretSubsystemConstants.AIM_TOLERANCE)).angle().degrees()

        current_angle: turns = self._get_turret_position_turns()
        chosen_angle: turns = get_chosen_angle(current_angle)
        
        encoder_a_target_position: turns | None = self._turret_to_encoder_a_position(chosen_angle)

        move: turns = abs(chosen_angle - current_angle)
        if move > TurretSubsystemConstants.LOOPING_MOVE_THRESH_REV:
            self._looping = True

        self._motor.set_target_position(encoder_a_target_position)
        
    def set_power(self, power: float) -> None:
        """
        Sets the power of the turret motor

        Parameters:
            power (float): The power to set
        """
        self._target = Translation2d()
        self._motor.set_power(power)

    def get_position(self) -> degrees:
        """
        Returns the current position of the turret

        Returns:
            degrees: The current position of the turret
        """
        return self._get_turret_position_turns() * 360

    def at_target(self) -> bool:
        """
        Returns whether the turret is at the target or not

        Returns:
            bool: Whether the turret is at the target or not
        """
        if self._target.norm() == 0:
            return False

        return abs(self.get_position() - self._target.angle().degrees()) <= TurretSubsystemConstants.AIM_TOLERANCE

    def is_looping(self) -> bool:
        """
        Returns whether the turret is looping (currently rotating all the way around)or not

        Returns:
            bool: Whether the turret is looping or not
        """
        if self._target.norm() == 0:
            return False 
        if self.at_target():
            self._looping = False

        return self._looping
    
    def reset(self) -> None:
        """
        Resets the turret
        """
        current_estimate: float = self._get_turret_position_turns()
        absolute_angle: float | None = self._compute_absolute_turntable_angle_from_two_encoders(current_estimate)

        if absolute_angle is None :
            return
        
        error: float = absolute_angle - current_estimate
        if abs(error) > TurretSubsystemConstants.ABS_CORRECTION_MAX_JUMP_REV :
            print(f"[Turret] WARN: ABS correction jump was too large ({error}")
            return 

    def print_diagnostics(self) -> None:
        """
        Prints the turret diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Turret Angle", self._motor.get_position())
        _ = SmartDashboard.putNumber("Turret Target Angle", self._target.angle().degrees())
        _ = SmartDashboard.putNumber("Turret Angle Tolerance", self._tolerance)

    def _get_turret_position_turns(self) -> turns:
        """
        Returns the current position of the turret in turns

        Returns:
            turns: The current position of the turret
        """
        encoder_rel_a: turns = self._encoder_a.get_position().value
        angle: degrees = encoder_rel_a / self._encoder_a_gear_ratio
        return angle

    def _sanitize_range(self) -> None:
        """
        Sanitizes the turret range
        """
        desired: float = TurretSubsystemConstants.MAXIMUM_ANGLE - TurretSubsystemConstants.MINIMUM_ANGLE

        if desired <= 0:
            print("[Turret] WARN: max_angle must be > min_angle. Forcing 0.1 rev range.")
            self._max_angle = self._min_angle + 0.1
            desired = 0.1

        if desired > self._max_abs_range:
            print(f"[Turret] ERROR: Requested range {desired} rev exceeds absolute resolvable range {self._max_abs_range} rev. Trimming.")
            self._max_angle = self._min_angle + self._max_abs_range

    def _startup_seed_relative_from_absolute(self) -> None:
        """
        Seeds the turret relative encoder from the absolute encoders
        """
        hint: float = 0.5 * (TurretSubsystemConstants.MINIMUM_ANGLE + TurretSubsystemConstants.MAXIMUM_ANGLE)

        abs_angle: float | None = self._compute_absolute_turntable_angle_from_two_encoders(hint)
        if abs_angle is None:
            print("[Turret] ERROR: CRT absolute solve failed. Falling back to encoder A only.")
            a_abs: rotation = self._encoder_a.get_absolute_position().value

            base: float = ((a_abs - TurretSubsystemConstants.ENCODER_A_OFFSET) / self._encoder_a_gear_ratio)
            abs_angle = self._lift_into_range_near_hint(base, hint)

        abs_angle = wrap_range(abs_angle, TurretSubsystemConstants.MINIMUM_ANGLE, TurretSubsystemConstants.MAXIMUM_ANGLE)

        encA_rel: float = self._turret_to_encoder_a_position(abs_angle)
        _ = self._encoder_a.set_position(encA_rel)

        print(f"[Turret] Startup absolute angle: {abs_angle} rev; seeded encA_rel={encA_rel} rev")

    def _turret_to_encoder_a_position(self, turn_rev: float) -> float:
        """
        Returns the position of the turret relative to encoder A

        Parameters:
            turn_rev (float): The position of the turret in turns

        Returns:
            float: The position of the turret relative to encoder A
        """
        return self._encoder_a_gear_ratio * turn_rev

    def _compute_absolute_turntable_angle_from_two_encoders(self, hint_turn_rev: float) -> float | None:
        """
        Computes the absolute turntable angle from two encoders

        Parameters:
            hint_turn_rev (float): The hint turn in turns

        Returns:
            float | None: The absolute turntable angle
        """
        a_abs: rotation = self._encoder_a.get_absolute_position().value
        b_abs: rotation = self._encoder_b.get_absolute_position().value

        rA: teeth = mod_teeth((a_abs - TurretSubsystemConstants.ENCODER_A_OFFSET) * TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_A)
        rB: teeth = mod_teeth((b_abs - TurretSubsystemConstants.ENCODER_B_OFFSET) * TurretSubsystemConstants.TEETH_B, TurretSubsystemConstants.TEETH_B)

        x0: teeth | None = self._solve_crt_teeth(rA, rB)
        if x0 is None:
            print("[Turret] WARN: CRT teeth solve produced no solution.")
            return None

        base_turn: float = x0 / TurretSubsystemConstants.TEETH_TURRET

        lifted: float = self._lift_into_range_near_hint(base_turn, hint_turn_rev)
        x_lifted: float = lifted * TurretSubsystemConstants.TEETH_TURRET
        rB_pred: teeth = mod_teeth(x_lifted, TurretSubsystemConstants.TEETH_B)
        err_teeth: teeth = min(
            abs(rB_pred - rB),
            TurretSubsystemConstants.TEETH_B - abs(rB_pred - rB) 
        )
        if err_teeth > TurretSubsystemConstants.ABS_MATH_TOL_TEETH:
            print(f"[Turret] WARN: CRT solution inconsistent with encoder B (err {err_teeth:.3f} teeth).")
            return None

        return lifted

    def _solve_crt_teeth(self, rA: float, rB: float) -> teeth | None:
        """
        Solves CRT for teeth

        Parameters:
            rA (float): The first remainder
            rB (float): The second remainder

        Returns:
            teeth | None: The CRT solution
        """
        A: int = TurretSubsystemConstants.TEETH_A
        B: int = TurretSubsystemConstants.TEETH_B

        g: int = gcd(A, B)
        if g != 1:

            diff = mod_teeth(rB - rA, g)
            if min(diff, g - diff) > 1e-3:
                print("[Turret] ERROR: Moduli not coprime and remainders inconsistent; no solution.")
                return None

        for k in range(0, B):
            x: float = rA + k * A

            xm: teeth = mod_teeth(x, B)
            err: float = min(abs(xm - rB), B - abs(xm - rB))
            if err <= TurretSubsystemConstants.ABS_MATH_TOL_TEETH:

                return mod_teeth(x, self._lcm_teeth)

        return None

    def _lift_into_range_near_hint(self, base_turn_rev: float, hint_turn_rev: float) -> float:
        """
        Lifts the turn into the range [MIN_ANGLE, MAX_ANGLE] near the hint

        Parameters:
            base_turn_rev (float): The base turn in turns
            hint_turn_rev (float): The hint turn in turns

        Returns:
            float: The lifted turn
        """
        period: turns = self._max_abs_range

        if period <= 0:
            return wrap_range(base_turn_rev, self._min_angle, self._max_angle)

        n: int = nearest_int((hint_turn_rev - base_turn_rev) / period)
        candidate: float = base_turn_rev + n * period

        options: list[float] = [candidate, candidate - period, candidate + period]
        in_range: list[float] = [x for x in options if (self._min_angle <= x <= self._max_angle)]

        if not in_range:
            return wrap_range(candidate, self._min_angle, self._max_angle)

        best: float = min(in_range, key=lambda x: abs(x - hint_turn_rev))
        return best

    def _choose_nearest_equivalent_in_range(self, desired_heading_rev: turns, cur_turn_rev: turns) -> turns | None:
        """
        Returns the nearest equivalent angle in the range [MIN_ANGLE, MAX_ANGLE]

        Parameters:
            desired_heading_rev (turns): The desired heading in turns
            cur_turn_rev (turns): The current heading in turns

        Returns:
            turns | None: The nearest equivalent angle
        """
        n_min: int = ceil(self._min_angle - desired_heading_rev)
        n_max: int = floor(self._max_angle - desired_heading_rev)

        if n_min > n_max:
            return None

        best: float | None = None
        best_dist: float = float("inf")
        for n in range(int(n_min), int(n_max) + 1):
            cand: float = desired_heading_rev + n
            dist: float = abs(cand - cur_turn_rev)
            if dist < best_dist:
                best_dist = dist
                best = cand
        return best