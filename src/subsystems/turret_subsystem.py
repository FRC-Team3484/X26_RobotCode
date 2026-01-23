from frc3484.motion import ExpoMotor
from commands2 import subsystem
from src.constants import TurretSubsystemConstants
from wpimath.geometry import Translation2d
from wpimath.units import degrees, inchesToMeters, turns
from wpilib import SmartDashboard
from phoenix6.hardware import CANcoder
from math import floor, ceil, gcd

def wrap_range(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def nearest_int(x: float) -> int:
    return int(floor(x + 0.5)) if x >= 0 else int(ceil(x - 0.5))

def mod_teeth(residual_teeth: float, mod_teeth_count: int) -> float:
    m = float(mod_teeth_count)
    return residual_teeth - m * floor(residual_teeth / m)

def lcm(a: int, b: int) -> int:
    return abs(a * b) // gcd(a, b)

def wrap_range(x: float, min_val: float, max_val: float):
    span = max_val - min_val
    if span <= 0:
        return x
    return (x - min_val) % span + min_val

class TurretSubsystem(subsystem) :
    def __init__(self)-> None:
        self._motor = ExpoMotor(TurretSubsystemConstants.MOTOR_CONFIG, TurretSubsystemConstants.PID_CONFIG, TurretSubsystemConstants.FEED_FORWARD_CONFIG, TurretSubsystemConstants.EXPO_CONFIG, 0, gear_ratio= TurretSubsystemConstants.GEAR_RATIO, external_encoder= self._encoder)
        self._target: Translation2d = Translation2d()
        self._tolerance: degrees = 0
        self._encoder_a: CANcoder = CANcoder(TurretSubsystemConstants.ENCODER_A_CAN_ID, TurretSubsystemConstants.ENCODER_A_CAN_BUS_NAME)
        self._encoder_b: CANcoder = CANcoder(TurretSubsystemConstants.ENCODER_B_CAN_ID, TurretSubsystemConstants.ENCODER_B_CAN_BUS_NAME)
        
        # Encoder degrees per 1 turntable revolution (direct mesh with turntable gear)
        self._encoder_a_degrees_per_turret: degrees = TurretSubsystemConstants.TEETH_TURRET / TurretSubsystemConstants.TEETH_A
        self._encoder_b_degrees_per_turret: degrees = TurretSubsystemConstants.TEETH_TURRET / TurretSubsystemConstants.TEETH_B

        self._lcm_teeth: int = lcm(a=TurretSubsystemConstants.TEETH_A, b=TurretSubsystemConstants.TEETH_B)
        self._max_abs_range_degrees: degrees = self._lcm_teeth / TurretSubsystemConstants.TEETH_TURRET

        self._encoder_rel_a: degrees | None = None
        self._looping: bool = False
        self._last_angle_set: float = 0.0
        self._last_cmd_angle: degrees | None = None

        self._sanitize_range()
        self._startup_seed_rel_from_abs()

    def periodic(self) -> None :
        if SmartDashboard.getBoolean("Turret Diagnostics", False):
            self.print_diagnostics()

    def set_target_angle(self, desired_heading_degrees: degrees) -> None :
        current_angle: degrees = self.get_current_angle()
        chosen_angle: degrees | None = self._choose_nearest_equivalent_in_range(desired_heading_degrees, current_angle)
        if chosen_angle is None:
            print("[Turret] WARN: Oh No! :( \n    No equivalent heading inside range; clamping")
            chosen_angle: degrees = wrap_range(x=desired_heading_degrees, lo=TurretSubsystemConstants.MINIMUM_ANGLE, hi=TurretSubsystemConstants.MAXIMUM_ANGLE)

        self._target.angle() = chosen_angle
        self._encoder_rel_a: degrees = self._turret_to_encA_rel(turn_degrees=chosen_angle)

        self._motor.set_target_position(self._encoder_rel_a)

        if current_angle is not None:
            move: degrees = abs(chosen_angle-current_angle)
            if move > TurretSubsystemConstants.LOOPING_MOVE_THRESH_REV:
                self._looping = True
        
        self._last_cmd_angle: degrees = chosen_angle

    def set_duty_cycle(self, duty: float) -> None :
        self._target.angle() = None
        self._encoder_rel_a = None
        self._motor.set_power(duty)

    def get_current_angle(self) -> degrees:
        encoder_rel_a: turns = self._encoder_a.get_position().value()
        angle: degrees = encoder_rel_a / self._encoder_a_degrees_per_turret
        self._last_angle_set = angle
        return angle

    def at_target(self) -> bool:
        if self._target.angle() is None :
            return False
        current_angle: float = self.get_current_angle()
        return abs(current_angle - self._target.angle()) <= TurretSubsystemConstants.AIM_TOLERANCE
    

    def is_looping(self) -> bool :
        if self._target.angle() is None :
            return False 
        if self.at_target():
            self._looping = False
        return self._looping
    
    def reset(self) -> None :
        current_estimate: float = self.get_current_angle
        absolute_angle: float | None = self._compute_absolute_turret_angle_from_two_encoders(hint_turn_rev=current_estimate)

        if absolute_angle is None :
            return
        
        error: float = absolute_angle - current_estimate
        if abs(error) > TurretSubsystemConstants.ABS_CORRECTION_MAX_JUMP_REV :
            print(f"[Turret] WARN: ABS correction jump was too large ({error}")
            return 
        
        new_encoder_rel_a: float = self._turret_to_encA_rel(turn_rev=absolute_angle)
        self._encoder_a.set_position(new_encoder_rel_a)

    # old stuff VV
        
    def aim(self, target: Translation2d, ) -> None:
        self._target = target
        self._tolerance = Translation2d(target.distance(), inchesToMeters(TurretSubsystemConstants.AIM_TOLERANCE)).angle().degrees()
        self._motor.set_target_position(wrap_range(target.angle().degrees(), TurretSubsystemConstants.MINIMUM_ANGLE, TurretSubsystemConstants.MAXIMUM_ANGLE))

    def set_power(self, power: float) -> None:
        self._motor.set_power(power)

    def is_aimed(self) -> bool:
        if self._target.X() == 0 and self._target.Y() == 0 :
            return False
        error: degrees = abs(self._motor.get_position() - self._target.angle().degrees())
        return error <= self._tolerance

    def print_diagnostics(self) -> None :
            SmartDashboard.putNumber("Turret Angle", self._motor.get_position())
            SmartDashboard.putNumber("Turret Target Angle", self._target.angle().degrees())
            SmartDashboard.putNumber("Turret Angle Tolerance", self._tolerance)

    # new stuff VV

    def _sanitize_range(self):
        desired = TurretSubsystemConstants.MAXIMUM_ANGLE - TurretSubsystemConstants.MINIMUM_ANGLE
        if desired <= 0:
            print("[Turret] WARN: max_angle must be > min_angle. Forcing 0.1 rev range.")
            self.max_angle = self.min_angle + 0.1 # TODO: ask jakob
            desired = 0.1

        if desired > TurretSubsystemConstants:
            print(f"[Turntable] ERROR: Requested range {desired:.4f} rev exceeds "
                f"absolute resolvable range {self._max_abs_range_degrees} rev. Trimming.")
            self.max_angle = self.min_angle + self._max_abs_range_degrees # TODO: ask jakob

    def _startup_seed_relative_from_absolute(self):
        hint = 0.5 * (TurretSubsystemConstants.MINIMUM_ANGLE + TurretSubsystemConstants.MAXIMUM_ANGLE)

        abs_angle = self._compute_absolute_turntable_angle_from_two_encoders(hint)
        if abs_angle is None:
            print("[Turntable] ERROR: CRT absolute solve failed. Falling back to encoder A only.")
            a_abs = self._encoder_a.get_absolute_position()

            base = ((a_abs - TurretSubsystemConstants.ABS_OFFSET_A_REV) / self._encoder_a_degrees_per_turret)
            abs_angle = self._lift_into_range_near_hint(base, hint)

        abs_angle = wrap_range(abs_angle, TurretSubsystemConstants.MINIMUM_ANGLE, TurretSubsystemConstants.MAXIMUM_ANGLE)

        encA_rel = self._turntable_to_encA_rel(abs_angle)
        self._encoder_a.set_position(encA_rel)

        self._last_angle_est = abs_angle
        print(f"[Turret] Startup absolute angle: {abs_angle} rev; seeded encA_rel={encA_rel} rev")

    def _turntable_to_encA_rel(self, turn_rev: float) -> float:
        return self._encoder_a_degrees_per_turret * turn_rev

    def _compute_absolute_turntable_angle_from_two_encoders(self, hint_turn_rev: float):

        a_abs = self._encoder_a.get_absolute_position()  
        b_abs = self._encoder_b.get_absolute_position()

        rA = mod_teeth((a_abs - TurretSubsystemConstants.ABS_OFFSET_A_REV) * TurretSubsystemConstants.TEETH_A, TurretSubsystemConstants.TEETH_A) # TODO: check this
        rB = mod_teeth((b_abs - TurretSubsystemConstants.ABS_OFFSET_B_REV) * TurretSubsystemConstants.TEETH_B, TurretSubsystemConstants.TEETH_B)

        x0 = self._solve_crt_teeth(rA, rB)
        if x0 is None:
            print("[Turret] WARN: CRT teeth solve produced no solution.")
            return None

        base_turn = x0 / TurretSubsystemConstants.TEETH_TURRET

        lifted = self._lift_into_range_near_hint(base_turn, hint_turn_rev
)
        x_lifted = lifted * TurretSubsystemConstants.TEETH_TURRET
        rB_pred = mod_teeth(x_lifted, TurretSubsystemConstants.TEETH_B)
        err_teeth = min(
            abs(rB_pred - rB),
            TurretSubsystemConstants.TEETH_B - abs(rB_pred - rB) 
        )
        if err_teeth > self.ABS_MATCH_TOL_TEETH:
            print(f"[Turret] WARN: CRT solution inconsistent with encoder B (err {err_teeth:.3f} teeth).")
            return None

        return lifted

    def _solve_crt_teeth(self, rA: float, rB: float):
        A = TurretSubsystemConstants.TEETH_A
        B = TurretSubsystemConstants.TEETH_B

        g = gcd(A, B)
        if g != 1:

            diff = mod_teeth(rB - rA, g)
            if min(diff, g - diff) > 1e-3:
                print("[Turret] ERROR: Moduli not coprime and remainders inconsistent; no solution.")
                return None

        for k in range(0, B):
            x = rA + k * A

            xm = mod_teeth(x, B)
            err = min(abs(xm - rB), B - abs(xm - rB))
            if err <= TurretSubsystemConstants.ABS_MATH_TOL_TEETH:

                return mod_teeth(x, self._lcm_teeth)

        return None

    def _lift_into_range_near_hint(self, base_turn_rev: float, hint_turn_rev: float) -> float:
        period = self._max_abs_range_degrees

        if period <= 0:
            return wrap_range(base_turn_rev, TurretSubsystemConstants.MINIMUM_ANGLE, TurretSubsystemConstants.MAXIMUM_ANGLE)

        n = nearest_int((hint_turn_rev - base_turn_rev) / period)
        candidate = base_turn_rev + n * period

        options = [candidate, candidate - period, candidate + period]
        in_range = [x for x in options if (TurretSubsystemConstants.MINIMUM_ANGLE <= x <= TurretSubsystemConstants.MAXIMUM_ANGLE)]

        if not in_range:
            return wrap_range(candidate, TurretSubsystemConstants.MINIMUM_ANGLE, TurretSubsystemConstants.MAXIMUM_ANGLE)

        best = min(in_range, key=lambda x: abs(x - hint_turn_rev))
        return best

    def _choose_nearest_equivalent_in_range(self, desired_heading_rev: float, cur_turn_rev: float):
        n_min = ceil(TurretSubsystemConstants.MINIMUM_ANGLE - desired_heading_rev)
        n_max = floor(TurretSubsystemConstants.MAXIMUM_ANGLE - desired_heading_rev)

        if n_min > n_max:
            return None

        best = None
        best_dist = float("inf")
        for n in range(int(n_min), int(n_max) + 1):
            cand = desired_heading_rev + n
            dist = abs(cand - cur_turn_rev)
            if dist < best_dist:
                best_dist = dist
                best = cand
        return best