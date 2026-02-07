from constants import UserInterface
from frc3484.controls import SC_Controller

_DRIVER_INPUTS: type[UserInterface.Driver] = UserInterface.Driver
_OPERATOR_INPUTS: type[UserInterface.Operator] = UserInterface.Operator

_TEST_INPUTS1: type[UserInterface.TestConstants1] = UserInterface.TestConstants1
_TEST_INPUTS2: type[UserInterface.TestConstants2] = UserInterface.TestConstants2
_DEMO_INPUTS: type[UserInterface.DemoController] = UserInterface.DemoController
_SYSID_INPUTS: type[UserInterface.SysidController] = UserInterface.SysidController

class DriverInterface:
    _controller: SC_Controller = SC_Controller(
        _DRIVER_INPUTS.CONTROLLER_PORT,
        _DRIVER_INPUTS.AXIS_LIMIT,
        _DRIVER_INPUTS.TRIGGER_LIMIT,
        _DRIVER_INPUTS.JOYSTICK_DEADBAND
    )

    def __init__(self) -> None:
        pass

    def get_throttle(self) -> float:
        return self._controller.get_axis(_DRIVER_INPUTS.THROTTLE_AXIS)
    def get_strafe(self) -> float:
        return self._controller.get_axis(_DRIVER_INPUTS.STRAFE_AXIS)
    def get_rotation(self) -> float:
        return self._controller.get_axis(_DRIVER_INPUTS.ROTATION_AXIS)
    
    def get_reset_heading(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.RESET_HEADING_BUTTON)
    
    def get_hold_mode(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.HOLD_MODE_BUTTON)
    
    def get_toggle_coast(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.TOGGLE_COAST_BUTTON)
    
    def get_low_speed_mode(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.LOW_SPEED_MODE_BUTTON)
    
    def get_dynamic_pivot(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.DYNAMIC_PIVOT_BUTTON)
    
    def get_jog_up(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_UP_BUTTON)
    def get_jog_down(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_DOWN_BUTTON)
    def get_jog_left(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_LEFT_BUTTON)
    def get_jog_right(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_RIGHT_BUTTON)
    
    def get_goto_climb(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.GOTO_CLIMB_BUTTON)
    
    def set_left_rumble(self, rumble: float) -> None:
        self._controller.set_left_rumble(rumble)
    def set_right_rumble(self, rumble: float) -> None:
        self._controller.set_right_rumble(rumble)
    def set_rumble(self, rumble: float) -> None:
        self._controller.set_rumble(rumble)

class OperatorInterface:
    _controller: SC_Controller = SC_Controller(
        _OPERATOR_INPUTS.CONTROLLER_PORT,
        _OPERATOR_INPUTS.AXIS_LIMIT,
        _OPERATOR_INPUTS.TRIGGER_LIMIT,
        _OPERATOR_INPUTS.JOYSTICK_DEADBAND
    )
    
    def get_right_feed_point(self) -> bool:
        return self._controller.get_button(_OPERATOR_INPUTS.RIGHT_FEEDER_BUTTON)
    def get_left_feed_point(self) -> bool:
        return self._controller.get_button(_OPERATOR_INPUTS.LEFT_FEEDER_BUTTON)

    def get_right_feed_point_axis_x(self) -> float:
        return self._controller.get_axis(_OPERATOR_INPUTS.RIGHT_FEEDER_AXIS_X)
    def get_right_feed_point_axis_y(self) -> float:
        return self._controller.get_axis(_OPERATOR_INPUTS.RIGHT_FEEDER_AXIS_Y)
    def get_left_feed_point_axis_x(self) -> float:
        return self._controller.get_axis(_OPERATOR_INPUTS.LEFT_FEEDER_AXIS_X)
    def get_left_feed_point_axis_y(self) -> float:
        return self._controller.get_axis(_OPERATOR_INPUTS.LEFT_FEEDER_AXIS_Y)
    
    def get_launcher(self) -> float:
        return self._controller.get_axis(_OPERATOR_INPUTS.LAUNCHER_BUTTON)
    def get_intake(self) -> float:
        return self._controller.get_axis(_OPERATOR_INPUTS.INTAKE_BUTTON)
    def get_eject(self) -> bool:
        return self._controller.get_button(_OPERATOR_INPUTS.EJECT_BUTTON)

    def get_climber_extend(self) -> bool:
        return self._controller.get_button(_OPERATOR_INPUTS.CLIMBER_EXTEND_BUTTON)
    def get_climber_retract(self) -> bool:
        return self._controller.get_button(_OPERATOR_INPUTS.CLIMBER_RETRACT_BUTTON)

    def get_ignore_vision(self) -> bool:
        return self._controller.get_button(_OPERATOR_INPUTS.IGNORE_VISION_BUTTON)

class TestInterface1:
    _controller1: SC_Controller = SC_Controller(
        _TEST_INPUTS1.CONTROLLER_PORT,
        _TEST_INPUTS1.AXIS_LIMIT,
        _TEST_INPUTS1.TRIGGER_LIMIT,
        _TEST_INPUTS1.JOYSTICK_DEADBAND
    )
    def get_wheel_power(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.FLYWHEEL_INPUT)

    def get_indexer(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.INDEXER_INPUT)
    
    def get_turret(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.TURRET_INPUT)

    def get_climber(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.CLIMBER_INPUT)
    
class TestInterface2:
    _controller2: SC_Controller = SC_Controller(
        _TEST_INPUTS2.CONTROLLER_PORT,
        _TEST_INPUTS2.AXIS_LIMIT,
        _TEST_INPUTS2.TRIGGER_LIMIT,
        _TEST_INPUTS2.JOYSTICK_DEADBAND
    )
    def get_intake_roller(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.INTAKE_ROLLER_INPUT)
    
    def get_intake_pivot(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.INTAKE_PIVOT_INPUT)
    
    def get_feeder(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.FEEDER_INPUT)

class DemoInterface:
    _demo_controller: SC_Controller = SC_Controller(
        _DEMO_INPUTS.CONTROLLER_PORT,
        _DEMO_INPUTS.AXIS_LIMIT,
        _DEMO_INPUTS.TRIGGER_LIMIT,
        _DEMO_INPUTS.JOYSTICK_DEADBAND
    )
    def demo_get_flywheel(self) -> float:
        return \
        (self._demo_controller.get_axis(_DEMO_INPUTS.FLYWHEEL_LEFT_INPUT)*(1/3)) + \
        (self._demo_controller.get_axis(_DEMO_INPUTS.FLYWHEEL_RIGHT_INPUT)*(2/3))
    
    def demo_get_turret(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.TURRET_LEFT) - self._demo_controller.get_axis(_DEMO_INPUTS.TURRET_RIGHT)
    
    def demo_get_feed(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.FEED_INPUT)
    
    def demo_get_eject_feeder(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.EJECT_FEEDER)
    
    def demo_get_intake(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.INTAKE_INPUT)
    
    def demo_get_throttle(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.THROTTLE_INPUT)
    
    def demo_get_strafe(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.STRAFE_INPUT)
    
    def demo_get_rotate(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.ROTATE_INPUT)

    def demo_get_extend_climb(self) -> float:
        return self._demo_controller.get_button(_DEMO_INPUTS.CLIMB_EXTEND)
    
    def demo_get_retract_climb(self) -> float:
        return self._demo_controller.get_button(_DEMO_INPUTS.CLIMB_RETRACT)

    def demo_get_jog_up(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.JOG_UP_BUTTON)
    
    def demo_get_jog_down(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.JOG_DOWN_BUTTON)
    
    def demo_get_jog_left(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.JOG_LEFT_BUTTON)
    
    def demo_get_jog_right(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.JOG_RIGHT_BUTTON)

    def demo_get_reset_heading(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.RESET_HEADING_BUTTON)
    
class SysIDInterface:
    _sysid_controller: SC_Controller = SC_Controller(
        _SYSID_INPUTS.CONTROLLER_PORT,
        _SYSID_INPUTS.AXIS_LIMIT,
        _SYSID_INPUTS.TRIGGER_LIMIT,
        _SYSID_INPUTS.JOYSTICK_DEADBAND
    )
        
    def get_quasistatic_forward(self) -> bool:
        if not self.get_quasistatic_reverse() and not self.get_dynamic_forward() and not self.get_dynamic_reverse():
            return self._sysid_controller.get_button(_TEST_INPUTS1.QUASI_FWD_BUTTON)
        return False

    def get_quasistatic_reverse(self) -> bool:
        if not self.get_quasistatic_forward() and not self.get_dynamic_forward() and not self.get_dynamic_reverse():
            return self._sysid_controller.get_button(_TEST_INPUTS1.QUASI_REV_BUTTON)
        return False

    def get_dynamic_forward(self) -> bool:
        if not self.get_quasistatic_forward() and not self.get_quasistatic_reverse() and not self.get_dynamic_reverse():
            return self._sysid_controller.get_button(_TEST_INPUTS1.DYNAMIC_FWD_BUTTON)
        return False

    def get_dynamic_reverse(self) -> bool:
        if not self.get_quasistatic_forward() and not self.get_quasistatic_reverse() and not self.get_dynamic_forward():
            return self._sysid_controller.get_button(_TEST_INPUTS1.DYNAMIC_REV_BUTTON)
        return False