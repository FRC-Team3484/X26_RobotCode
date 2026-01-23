from frc3484.controls import SC_Controller
from constants import UserInterface

_TEST_INPUTS1: type[UserInterface.TestConstants1] = UserInterface.TestConstants1
_TEST_INPUTS2: type[UserInterface.TestConstants2] = UserInterface.TestConstants2
_DEMO_INPUTS: type[UserInterface.DemoController] = UserInterface.DemoController

class TestInterface:
    _controller1: SC_Controller = SC_Controller(
        _TEST_INPUTS1.CONTROLLER_PORT,
        _TEST_INPUTS1.AXIS_LIMIT,
        _TEST_INPUTS1.TRIGGER_LIMIT,
        _TEST_INPUTS1.JOYSTICK_DEADBAND
    )
    _controller2: SC_Controller = SC_Controller(
        _TEST_INPUTS2.CONTROLLER_PORT,
        _TEST_INPUTS2.AXIS_LIMIT,
        _TEST_INPUTS2.TRIGGER_LIMIT,
        _TEST_INPUTS2.JOYSTICK_DEADBAND
    )

    _demo_controller: SC_Controller = SC_Controller(
        _DEMO_INPUTS.CONTROLLER_PORT,
        _DEMO_INPUTS.AXIS_LIMIT,
        _DEMO_INPUTS.TRIGGER_LIMIT,
        _DEMO_INPUTS.JOYSTICK_DEADBAND
    )

    ## CONTROLLER 1

    def get_wheel_power(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.FLYWHEEL_INPUT)

    def get_spindexer(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.SPINDEXER_INPUT)
    
    def get_turret(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.TURRET_INPUT)

    def get_climber(self) -> float:
        return self._controller1.get_axis(_TEST_INPUTS1.CLIMBER_INPUT)

    ## END CONTROLLER 1
    ## CONTROLLER 2

    def get_intake_roller(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.INTAKE_ROLLER_INPUT)
    
    def get_intake_pivot(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.INTAKE_PIVOT_INPUT)
    
    def get_feeder(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.FEEDER_INPUT)
    
    ## END CONTROLLER 2
    ## DEMO CONTROLLER

    def _demo_get_flywheel(self) -> float:
        return \
        (self._demo_controller.get_axis(_DEMO_INPUTS.FLYWHEEL_LEFT_INPUT)*(1/3)) + \
        (self._demo_controller.get_axis(_DEMO_INPUTS.FLYWHEEL_RIGHT_INPUT)*(2/3))
    
    def _demo_get_turret(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.TURRET_INPUT)

    def _demo_get_feed(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.FEED_INPUT)

    def _demo_get_eject(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.EJECT_INPUT)
    
    def _demo_get_intake(self) -> bool:
        return self._demo_controller.get_button(_DEMO_INPUTS.INTAKE_INPUT)
    
    def _demo_get_throttle(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.THROTTLE_INPUT)
    
    def _demo_get_strafe(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.STRAFE_INPUT)
    
    def _demo_get_rotate(self) -> float:
        return self._demo_controller.get_axis(_DEMO_INPUTS.ROTATE_INPUT)


    ## END DEMO CONTROLLER