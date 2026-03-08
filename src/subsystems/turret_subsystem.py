from typing import TypeAlias
from math import floor, ceil, gcd, lcm

from commands2 import Subsystem
from wpilib import SmartDashboard, DutyCycleEncoder, Timer
from wpimath.geometry import Translation2d
from wpimath.units import degrees, inchesToMeters, turns

from frc3484.motion import ExpoMotor

from src.constants import TurretSubsystemConstants

# Custom datatype for gear teeth
teeth: TypeAlias = float

def clip_range(x: float, lo: float, hi: float) -> float:
    """
    Clip x to the range [lo, hi]

    Parameters:
        x (float): The value to clip
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

def best_equivalent_angle(desired: turns, current: turns, min_angle: turns, max_angle: turns) -> turns | None:
    """
    Finds the angle equivalent to desired that is closest to current and within the range [min_angle, max_angle]

    Parameters:
        desired (turns): The desired angle in turns
        current (turns): The current angle in turns
        min_angle (turns): The minimum allowed angle in turns
        max_angle (turns): The maximum allowed angle in turns
    Returns:
        turns | None: The best equivalent angle in turns, or None if no valid angle is found
    """
    n_min: float = ceil((min_angle - desired))
    n_max: float = floor((max_angle - desired))

    best: turns | None = None
    best_dist: float = float("inf")
    for n in range(int(n_min), int(n_max) + 1):
        cand: turns = desired + n
        dist: float = abs(cand - current)
        if dist < best_dist:
            best_dist = dist
            best = cand
        elif dist > best_dist:
            break  # distances will only get larger as we move further away
    if best is None or not (min_angle <= best <= max_angle):
        return None
    return best

def crt(r1: float, r2: float, d1: int, d2: int, tol: float = 0.5) -> float | None:
    """
    Find x such that x ≡ r1 (mod d1) and x ≡ r2 (mod d2), allowing float remainders with small errors.

    Parameters:
        r1 (float): The first remainder
        r2 (float): The second remainder
        d1 (int): The first modulus
        d2 (int): The second modulus
        tol (float): The tolerance for considering two remainders equal
    Returns:
        float | None: The solution to the CRT problem, or None if no solution exists
    """

    def wrap_dist(x: float, y: float, mod: float) -> float:
        """Circular distance between x and y on [0, mod)."""
        d = abs((x - y) % mod)
        return min(d, mod - d)

    if d1 <= 0 or d2 <= 0:
        return None

    G = gcd(d1, d2)
    if G != 1:
        # compatibility condition: r1 ≡ r2 (mod G), within tolerance on the circle mod G
        if wrap_dist(r1, r2, float(G)) > tol:
            return None

    L = lcm(d1, d2)

    best = None
    best_err = float("inf")

    # Scan k; for coprime case, one of these is the exact CRT solution (up to noise)
    for k in range(d2):
        x = r1 + k * d1
        xm = x % d2
        err = wrap_dist(xm, r2, float(d2))
        if err < best_err:
            best_err = err
            best = x
            if best_err <= tol:
                break  # good enough

    if best is None:
        return None
    return best % L


class TurretSubsystem(Subsystem):
    """
    The turret subsystem

    Handles aiming the turret to a desired target, and uses Chinese Remainder Theorem to compute the turret angle using two encoders
    """
    def __init__(self)-> None:
        super().__init__()

        # Variables
        self._target: Translation2d = Translation2d()
        self._tolerance: turns = 0.0
        
        self._encoder_a_gear_ratio: float = TurretSubsystemConstants.TEETH_TURRET / TurretSubsystemConstants.TEETH_A

        self._lcm_teeth: teeth = lcm(TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_B)
        self._max_encoder_range: turns = self._lcm_teeth / TurretSubsystemConstants.TEETH_TURRET

        self._looping: bool = False

        self._min_angle: turns = TurretSubsystemConstants.MINIMUM_ANGLE / 360.0
        self._max_angle: turns = TurretSubsystemConstants.MAXIMUM_ANGLE / 360.0

        # Create motor and encoders
        self._encoder_a: DutyCycleEncoder = DutyCycleEncoder(TurretSubsystemConstants.ENCODER_A_CHANNEL)
        self._encoder_b: DutyCycleEncoder = DutyCycleEncoder(TurretSubsystemConstants.ENCODER_B_CHANNEL)

        self._encoder_a.setAssumedFrequency(1.0 / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD)
        self._encoder_b.setAssumedFrequency(1.0 / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD)

        self._motor: ExpoMotor = ExpoMotor(
            TurretSubsystemConstants.MOTOR_CONFIG, 
            TurretSubsystemConstants.PID_CONFIG, 
            TurretSubsystemConstants.FEED_FORWARD_CONFIG,
            TurretSubsystemConstants.EXPO_CONFIG,
            0, 
            TurretSubsystemConstants.MOTOR_GEAR_RATIO,
        )

        # Startup functions
        self._sanitize_range()

        self._initialization_timer: Timer = Timer()
        self._initialized: bool = False

    def periodic(self) -> None:
        """
        The periodic function for the turret subsystem. Reads the encoders after a startup delay.
        """
        if not self._initialized and self._encoder_a.isConnected() and self._encoder_b.isConnected():
            if not self._initialization_timer.isRunning():
                self._initialization_timer.start()
        if not self._initialized and self._initialization_timer.hasElapsed(TurretSubsystemConstants.ENCODER_STARTUP_DELAY):
            self.reset()
            self._initialized = True

    def get_position(self) -> degrees:
        """
        Returns the current position of the turret
        This is based on the motor encoder

        Returns:
            degrees: The current position of the turret
        """
        return self._motor.get_position()
    
    def _get_position_turns(self) -> turns:
        """
        Returns the current position of the turret in turns
        This is based on the motor encoder

        Returns:
            turns: The current position of the turret
        """
        return self.get_position() / 360.0

    def at_target(self) -> bool:
        """
        Returns whether the turret is at the target or not

        Returns:
            bool: Whether the turret is at the target or not
        """
        if self._target.norm() == 0:
            return False

        return abs(self.get_position() - self._target.angle().degrees()) <= self._tolerance

    def is_looping(self) -> bool:
        """
        Returns true if the turret is making a large movement to point at the target
        Also returns true if the target is at an angle that the turret can't point to
        It's a good idea to stop firing if the turret is looping to avoid missing shots

        Returns:
            bool: Whether the turret is looping or not
        """
        if self._target.norm() == 0:
            return False 
        if self.at_target():
            self._looping = False

        return self._looping

    def aim(self, target: Translation2d) -> None:
        """
        Aims the turret at a robot-relative target
        This must be called periodically

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
            chosen_angle: turns | None = self._choose_nearest_equivalent_in_range(target.angle().degrees() / 360.0, current_angle)
            if chosen_angle is None:
                return clip_range(target.angle().degrees() / 360.0, self._min_angle, self._max_angle)

            return chosen_angle

        self._target = target
        self._tolerance = Translation2d(target.norm(), inchesToMeters(TurretSubsystemConstants.AIM_TOLERANCE)).angle().degrees() / 360.0

        current_angle: turns = self._get_position_turns()
        target_angle: turns = get_chosen_angle(current_angle)
        
        encoder_a_target_position: turns = self._turret_to_encoder_a_position(target_angle)

        move_distance: turns = abs(target_angle - current_angle)
        if move_distance > TurretSubsystemConstants.LOOPING_DISTANCE:
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
    
    def reset(self, warn_jump: bool = True) -> None:
        """
        Resets the turret angle based on the encoders.

        Parameters:
            warn_jump (bool): Whether to print a warning if the reset causes a large jump in the turret angle
        """
        encoder_a_teeth: teeth = self._get_encoder_a_value() * TurretSubsystemConstants.TEETH_A
        encoder_b_teeth: teeth = self._get_encoder_b_value() * TurretSubsystemConstants.TEETH_B
        crt_result: teeth | None = crt(encoder_a_teeth, encoder_b_teeth, TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_B, tol=TurretSubsystemConstants.MAX_ENCODER_ERROR)

        if crt_result is None:
            print(f"[Turret] ERROR: CRT solve failed: Encoder A: {encoder_a_teeth:.3f} teeth, Encoder B: {encoder_b_teeth:.3f} teeth")
            return
        
        err: teeth = abs((crt_result % TurretSubsystemConstants.TEETH_B) - encoder_b_teeth)
        if err > TurretSubsystemConstants.MAX_ENCODER_ERROR:
            print(f"[Turret] ERROR: CRT solution exceeds max error: Encoder A: {encoder_a_teeth:.3f} teeth, Encoder B: {encoder_b_teeth:.3f} teeth, CRT Result: {crt_result:.3f} teeth, Error: {err:.3f} teeth")
            return
        
        turret_angle: turns | None = best_equivalent_angle(crt_result / TurretSubsystemConstants.TEETH_TURRET, self._get_position_turns(), self._min_angle, self._max_angle)
        if turret_angle is None:
            print(f"[Turret] ERROR: CRT solution {360.0 * crt_result / TurretSubsystemConstants.TEETH_TURRET:.3f} degrees is outside the turret travel range.")
            return
        
        if warn_jump and abs(turret_angle / 360.0 - self.get_position()) > TurretSubsystemConstants.MAX_TURRET_ERROR:
            print(f"[Turret] WARN: Reset causes large jump in turret angle: Current Angle: {self.get_position():.2f} deg, New Angle: {turret_angle:.2f} deg")
        
        self._motor.set_encoder_position(turret_angle)

    def print_diagnostics(self) -> None:
        """
        Prints the turret diagnostics to SmartDashboard
        """
        _ = SmartDashboard.putNumber("Turret Angle", self._motor.get_position())
        # _ = SmartDashboard.putNumber("Turret Target Angle", self._target.angle().degrees())
        _ = SmartDashboard.putNumber("Turret Angle Tolerance", self._tolerance)

        _ = SmartDashboard.putNumber("Encoder A Position", self._get_encoder_a_value())
        _ = SmartDashboard.putNumber("Encoder B Position", self._get_encoder_b_value())
        _ = SmartDashboard.putBoolean("Encoder A Connected", self._encoder_a.isConnected())
        _ = SmartDashboard.putBoolean("Encoder B Connected", self._encoder_b.isConnected())

    def _sanitize_range(self) -> None:
        """
        Sanitizes the turret range
        """
        desired: turns = self._max_angle - self._min_angle

        if desired <= 0:
            print("[Turret] WARN: max_angle must be > min_angle")
            if desired == 0:
                self._max_angle = self._min_angle + 0.1
                desired = 0.1
            else:
                self._max_angle, self._min_angle = self._min_angle, self._max_angle
                desired = -desired

        if desired > self._max_encoder_range:
            print(f"[Turret] ERROR: Requested range {desired} rev exceeds absolute resolvable range {self._max_encoder_range} rev. Trimming.")
            self._max_angle = self._min_angle + self._max_encoder_range

    def _startup_seed_relative_from_absolute(self) -> None:
        """
        Sets the encoder A relative position based on the turret absolute position at startup
        """
        hint: turns = 0.5 * (self._min_angle + self._max_angle)

        turret_angle: turns | None = self._calculate_turret_angle_from_encoders(hint)
        if turret_angle is None:
            print("[Turret] ERROR: CRT solve failed. Falling back to encoder A only.")
            encoder_a_position: turns = self._get_encoder_a_value()

            base: turns = (encoder_a_position / self._encoder_a_gear_ratio)
            turret_angle = self._lift_into_range_near_hint(base, hint)

        turret_angle = clip_range(turret_angle, self._min_angle, self._max_angle)

        self._motor.set_encoder_position(turret_angle)

        print(f"[Turret] Startup absolute angle: {turret_angle} rev")

    def _turret_to_encoder_a_position(self, turret_angle: turns) -> turns:
        """
        Returns the position of the turret relative to encoder A

        Parameters:
            turret_angle (turns): The position of the turret in turns

        Returns:
            turns: The position of the turret relative to encoder A
        """
        return self._encoder_a_gear_ratio * turret_angle

    def _calculate_turret_angle_from_encoders(self, hint_angle: turns) -> turns | None:
        """
        Computes the absolute turret angle from two encoders

        Parameters:
            hint_angle (turns): Rough estimate of the turret angle to help with disambiguation

        Returns:
            turns | None: The absolute turret angle
        """

        remainder_a: teeth = self._get_encoder_a_value() * TurretSubsystemConstants.TEETH_A % TurretSubsystemConstants.TEETH_A
        remainder_b: teeth = self._get_encoder_b_value() * TurretSubsystemConstants.TEETH_B % TurretSubsystemConstants.TEETH_B

        x0: teeth | None = self._solve_crt_teeth(remainder_a, remainder_b)
        if x0 is None:
            print("[Turret] WARN: CRT teeth solve produced no solution.")
            return None

        base: turns = x0 / TurretSubsystemConstants.TEETH_TURRET

        lifted: turns = self._lift_into_range_near_hint(base, hint_angle)
        x_lifted: teeth = lifted * TurretSubsystemConstants.TEETH_TURRET
        predicted_remainder_b: teeth = x_lifted % TurretSubsystemConstants.TEETH_B
        err_teeth: teeth = min(
            abs(predicted_remainder_b - remainder_b),
            TurretSubsystemConstants.TEETH_B - abs(predicted_remainder_b - remainder_b) 
        )
        if err_teeth > TurretSubsystemConstants.MAX_ENCODER_ERROR:
            print(f"[Turret] WARN: CRT solution inconsistent with encoder B (err {err_teeth:.3f} teeth).")
            return None

        return lifted

    def _solve_crt_teeth(self, rA: teeth, rB: teeth) -> teeth | None:
        """
        Solves CRT for teeth

        Parameters:
            rA (teeth): The first remainder
            rB (teeth): The second remainder

        Returns:
            teeth | None: The CRT solution
        """
        A: int = TurretSubsystemConstants.TEETH_A
        B: int = TurretSubsystemConstants.TEETH_B
    
        g: int = gcd(A, B)
        if g != 1:

            diff = (rB - rA) % g
            if min(diff, g - diff) > 1e-3:
                print("[Turret] ERROR: Moduli not coprime and remainders inconsistent; no solution.")
                return None

        for k in range(0, B):
            x: float = rA + k * A

            xm: teeth = x % B
            err: float = min(abs(xm - rB), B - abs(xm - rB))
            if err <= TurretSubsystemConstants.MAX_ENCODER_ERROR:

                return x % self._lcm_teeth

        return None

    def _lift_into_range_near_hint(self, base_angle: turns, hint_angle: turns) -> turns:
        """
        Wraps base_angle into the range of turret angles that can be read by the encoders, choosing the one closest to the hint if multiple exist

        Parameters:
            base_angle (turns): The base turn in turns
            hint_angle (turns): The hint turn in turns

        Returns:
            turns: The lifted turn
        """
        period: turns = self._max_encoder_range

        if period <= 0:
            return clip_range(base_angle, self._min_angle, self._max_angle)
        n: int = nearest_int((hint_angle - base_angle) / period)
        candidate: turns = base_angle + n * period

        options: list[turns] = [candidate, candidate - period, candidate + period]
        in_range: list[turns] = [x for x in options if (self._min_angle <= x <= self._max_angle)]

        if not in_range:
            return clip_range(candidate, self._min_angle, self._max_angle)

        best: turns = min(in_range, key=lambda x: abs(x - hint_angle))
        return best

    def _choose_nearest_equivalent_in_range(self, desired_heading: turns, current_heading: turns) -> turns | None:
        """
        If there are multiple equivalent angles for the desired heading within the turret's range, choose the one nearest to the current turret angle

        Parameters:
            desired_heading (turns): The desired heading in turns
            current_heading (turns): The current heading in turns

        Returns:
            turns | None: The nearest equivalent angle
        """
        n_min: int = ceil(self._min_angle - desired_heading)
        n_max: int = floor(self._max_angle - desired_heading)

        if n_min > n_max:
            return None

        best: turns | None = None
        best_dist: float = float("inf")
        for n in range(int(n_min), int(n_max) + 1):
            cand: turns = desired_heading + n
            dist: float = abs(cand - current_heading)
            if dist < best_dist:
                best_dist = dist
                best = cand
        return best
    
    def _get_encoder_a_value(self) -> turns:
        return (self._encoder_a.get() - TurretSubsystemConstants.ENCODER_A_OFFSET) % 1.0
    
    def _get_encoder_b_value(self) -> turns:
        return (self._encoder_b.get() - TurretSubsystemConstants.ENCODER_B_OFFSET) % 1.0