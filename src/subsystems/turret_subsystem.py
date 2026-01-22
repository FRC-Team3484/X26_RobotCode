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


    # TODO: Next do at_target







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