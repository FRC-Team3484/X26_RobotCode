from typing import Literal, TypeAlias
from math import floor, ceil, gcd, lcm

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import SmartDashboard, DutyCycleEncoder, Timer
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Translation2d
from wpimath.units import degrees, turns, meters, inchesToMeters, volts

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

def wrap_dist(x: float, y: float, mod: float) -> float:
    """
    Calculates the circular distance between x and y, with values wrapping back to 0 after mod. For example, with mod=360, the distance between 10 and 350 would be 20, not 340.
    
    Parameters:
        x (float): The first value
        y (float): The second value
        mod (float): The modulus (period) of the space
    Returns:
        float: The shortest distance between x and y
    """
    d = abs((x - y) % mod)
    return min(d, mod - d)

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
    if best is None or not (min_angle <= best <= max_angle):
        return None
    return best

def crt(r1: float, r2: float, d1: int, d2: int, tol: float = 0.5) -> float | None:
    """
    Find x such that x ≡ r1 (mod d1) and x ≡ r2 (mod d2), allowing float remainders with small errors.
    This function checks all values that satisfy the first congruence and finds the one that best satisfies the second congruence, within a given tolerance.

    Parameters:
        r1 (float): The first remainder
        r2 (float): The second remainder
        d1 (int): The first modulus
        d2 (int): The second modulus
        tol (float): The tolerance for considering two remainders equal
    Returns:
        float | None: The solution to the CRT problem, or None if no solution exists
    """

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
        self._aim_tolerance: meters = inchesToMeters(TurretSubsystemConstants.AIM_TOLERANCE)
        self._tolerance: degrees = 0.0
        
        self._encoder_a_gear_ratio: float = TurretSubsystemConstants.TEETH_TURRET / TurretSubsystemConstants.TEETH_A

        self._lcm_teeth: teeth = lcm(TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_B)
        self._max_encoder_range: turns = self._lcm_teeth / TurretSubsystemConstants.TEETH_TURRET

        self._looping_distance: turns = TurretSubsystemConstants.LOOPING_DISTANCE / 360.0
        self._looping: bool = False

        self._min_angle: turns = TurretSubsystemConstants.MINIMUM_ANGLE / 360.0
        self._max_angle: turns = TurretSubsystemConstants.MAXIMUM_ANGLE / 360.0

        # Create motor and encoders
        self._encoder_a: DutyCycleEncoder = DutyCycleEncoder(TurretSubsystemConstants.ENCODER_A_CHANNEL)
        self._encoder_b: DutyCycleEncoder = DutyCycleEncoder(TurretSubsystemConstants.ENCODER_B_CHANNEL)

        self._encoder_a.setAssumedFrequency(1.0 / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD)
        self._encoder_b.setAssumedFrequency(1.0 / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD)

        self._encoder_a.setDutyCycleRange(TurretSubsystemConstants.ENCODER_MINIMUM_PULSE / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD, TurretSubsystemConstants.ENCODER_MAXIMUM_PULSE / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD)
        self._encoder_b.setDutyCycleRange(TurretSubsystemConstants.ENCODER_MINIMUM_PULSE / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD, TurretSubsystemConstants.ENCODER_MAXIMUM_PULSE / TurretSubsystemConstants.ENCODER_OUTPUT_PERIOD)

        self._encoder_a.setInverted(TurretSubsystemConstants.ENCODER_A_REVERSED)
        self._encoder_b.setInverted(TurretSubsystemConstants.ENCODER_B_REVERSED)

        self._motor: ExpoMotor = ExpoMotor(
            TurretSubsystemConstants.MOTOR_CONFIG, 
            TurretSubsystemConstants.PID_CONFIG, 
            TurretSubsystemConstants.FEED_FORWARD_CONFIG,
            TurretSubsystemConstants.EXPO_CONFIG,
            0, 
            TurretSubsystemConstants.MOTOR_GEAR_RATIO,
        )

        # SysID

        self._sys_id_routine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0
            ),
            SysIdRoutine.Mechanism(
                self._set_voltage,
                self._log_motors,
                self,
                'turret'
            )
        )

        # Startup functions
        self._sanitize_range()
        SmartDashboard.putBoolean("Turret Diagnostics", False)

        self._initialization_timer: Timer = Timer()
        self._initialized: bool = False

    def print_diagnostics(self) -> None:
        """
        Prints the turret diagnostics to SmartDashboard
        """
        SmartDashboard.putNumber("Turret Angle (deg)", self._motor.get_position())
        SmartDashboard.putBoolean("Encoder A Connected", self._encoder_a.isConnected())
        SmartDashboard.putBoolean("Encoder B Connected", self._encoder_b.isConnected())
        if self.has_target():
            SmartDashboard.putNumber("Turret Target Angle (deg)", self._target.angle().degrees())
            SmartDashboard.putNumber("Turret Angle Tolerance (deg)", self._tolerance)

        if self._encoder_a.isConnected() and self._encoder_b.isConnected():
            SmartDashboard.putNumber("Encoder A Position (rev)", self._get_encoder_a_value())
            SmartDashboard.putNumber("Encoder B Position (rev)", self._get_encoder_b_value())
            crt_solution = crt(self._get_encoder_a_value() * TurretSubsystemConstants.TEETH_A, self._get_encoder_b_value() * TurretSubsystemConstants.TEETH_B, TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_B, tol=TurretSubsystemConstants.MAX_ENCODER_ERROR)
            if crt_solution is not None:
                crt_solution = self._lift_into_range(crt_solution / TurretSubsystemConstants.TEETH_TURRET)
            SmartDashboard.putNumber("Turret Angle from Encoders (deg)", crt_solution * 360.0 if crt_solution is not None else float("nan"))

    def periodic(self) -> None:
        """
        The periodic function for the turret subsystem. Reads the encoders after a startup delay.
        """
        if not self._initialized and self._encoder_a.isConnected() and self._encoder_b.isConnected():
            if not self._initialization_timer.isRunning():
                self._initialization_timer.start()
        if not self._initialized and self._initialization_timer.hasElapsed(TurretSubsystemConstants.ENCODER_STARTUP_DELAY):
            self.reset(False)
            self._initialized = True
            self._initialization_timer.stop()

        if SmartDashboard.getBoolean("Turret Diagnostics", False):
            self.print_diagnostics()

    def get_position(self) -> degrees:
        """
        Returns the current position of the turret.
        This is based on the motor, not calculated from the encoders.

        Returns:
            degrees: The current position of the turret
        """
        return self._motor.get_position()

    def _set_voltage(self, voltage: volts) -> None:
        '''
        Sets the voltage of the motor

        Parameters:
            - voltage (volts): the voltage to set the motor to
        '''
        self._motor.set_raw_voltage(voltage)

    def _log_motors(self, log: SysIdRoutineLog) -> None:
        '''
        Logs the motor output for SysId routines

        Parameters:
            - log (SysIdRoutineLog): The log to write to
        '''
        log.motor(f'turret') \
            .voltage(self._motor.get_raw_voltage()) \
            .angularPosition(self._motor.get_raw_position()) \
            .angularVelocity(self._motor.get_raw_velocity())

    def get_sysid_command(self, mode: Literal['quasistatic', 'dynamic'], direction: SysIdRoutine.Direction) -> Command:
        '''
        Returns a SysID command for the specified mode and direction

        Parameters:
            - mode (Literal['quasistatic', 'dynamic']): the mode to run the SysID routine in
            - direction (SysIdRoutine.Direction): the direction to run the SysID routine in
        '''
        if mode == "quasistatic":
            return self._sys_id_routine.quasistatic(direction)
        elif mode == "dynamic":
            return self._sys_id_routine.dynamic(direction)
    
    def _get_position_turns(self) -> turns:
        """
        Returns the current position of the turret in turns.
        This is based on the motor, not calculated from the encoders.

        Returns:
            turns: The current position of the turret
        """
        return self.get_position() / 360.0
    
    def has_target(self) -> bool:
        """
        Returns whether the turret has a target or not.

        Returns:
            bool: Whether the turret has a target or not
        """
        return self._target.norm() != 0

    def at_target(self) -> bool:
        """
        Returns whether the turret is at the target or not.

        Returns:
            bool: Whether the turret is at the target or not
        """
        if not self.has_target():
            return False

        return abs(self.get_position() - self._target.angle().degrees()) <= self._tolerance

    def is_looping(self) -> bool:
        """
        Returns true if the turret is making a large movement to point at the target.
        Also returns true if the target is at an angle that the turret can't point to.
        It's a good idea to stop firing if the turret is looping to avoid missing shots.

        Returns:
            bool: Whether the turret is looping or not
        """
        if not self.has_target():
            return False 
        if self.at_target():
            self._looping = False

        return self._looping

    def aim(self, target: Translation2d) -> None:
        """
        Sets the target for the turret to aim at.

        Parameters:
            target (Translation2d): The target to aim to
        """

        self._target = target

        if not self.has_target():
            self._tolerance = 0.0
            self._motor.set_power(0)
            return
        
        target_angle: turns = target.angle().degrees() / 360.0
        self._tolerance = Translation2d(target.norm(), self._aim_tolerance).angle().degrees()

        current_angle: turns = self._get_position_turns()
        new_angle: turns | None = best_equivalent_angle(target_angle, current_angle, self._min_angle, self._max_angle)

        if new_angle is None:
            new_angle = clip_range(target_angle, self._min_angle, self._max_angle)

        move_distance: turns = abs(new_angle - current_angle)
        if move_distance > self._looping_distance:
            self._looping = True

        self._motor.set_target_position(new_angle * 360.0)
        
    def set_power(self, power: float) -> None:
        """
        Sets the power of the turret motor.

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
        
        err: teeth = wrap_dist(encoder_b_teeth, crt_result % TurretSubsystemConstants.TEETH_B, TurretSubsystemConstants.TEETH_B) # no need to check error for encoder A since CRT uses encoder A as the base
        if err > TurretSubsystemConstants.MAX_ENCODER_ERROR:
            print(f"[Turret] ERROR: CRT solution exceeds max error: Encoder A: {encoder_a_teeth:.3f} teeth, Encoder B: {encoder_b_teeth:.3f} teeth, CRT Result: {crt_result:.3f} teeth, Error: {err:.3f} teeth")
            return
        
        new_angle: turns = self._lift_into_range(crt_result / TurretSubsystemConstants.TEETH_TURRET)
        
        if warn_jump and abs(new_angle * 360.0 - self.get_position()) > TurretSubsystemConstants.MAX_TURRET_ERROR:
            print(f"[Turret] WARN: Reset causes large jump in turret angle: Current Angle: {self.get_position():.2f} deg, New Angle: {new_angle * 360.0:.2f} deg")
        
        self._motor.set_encoder_position(new_angle)

    def _sanitize_range(self) -> None:
        """
        Makes sure the turret range is valid, and trims it if it's larger than the maximum resolvable range of the encoders. Prints warnings or errors if the range is invalid or had to be trimmed.
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
            print(f"[Turret] ERROR: Turret range {desired} rev is greater than the encoder range {self._max_encoder_range} rev. Trimming.")
            self._max_angle = self._min_angle + self._max_encoder_range

    def _lift_into_range(self, angle: turns) -> turns:
        """
        The turret angle can be negative but crt only outputs positive values.
        This function shifts the crt result so it lies in the turret travel range.

        Parameters:
            angle (turns): The angle in turns to lift into range

        Returns:
            turns: The lifted angle in turns or the closest angle in the range if the input is outside the range
        """

        period: float = self._lcm_teeth / TurretSubsystemConstants.TEETH_TURRET

        k_min: int = ceil((self._min_angle - angle) / period)
        k_max: int = floor((self._max_angle - angle) / period)

        best = 0
        best_dist = float("inf")
        for k in range(k_min, k_max + 1):
            cand = angle + k * period
            dist = min(abs(cand - self._min_angle), abs(cand - self._max_angle))
            if dist < best_dist:
                best_dist = dist
                best = cand
        
        # if range < period, there will be at most one valid k: k_min = k_max, so we can just use k_min
        return best
    
    def _get_encoder_a_value(self) -> turns:
        return (self._encoder_a.get() - TurretSubsystemConstants.ENCODER_A_OFFSET) % 1.0
    
    def _get_encoder_b_value(self) -> turns:
        return (self._encoder_b.get() - TurretSubsystemConstants.ENCODER_B_OFFSET) % 1.0