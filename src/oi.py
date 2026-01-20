from frc3484.controls import SC_Controller
from constants import UserInterface

_TEST_INPUTS1: type[UserInterface.TestConstants1] = UserInterface.TestConstants1
_TEST_INPUTS2: type[UserInterface.TestConstants2] = UserInterface.TestConstants2

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

    def get_wheel_power(self) -> float:
        return self._controller2.get_axis(_TEST_INPUTS2.FLYWHEEL_INPUT)

    # add indexer controller 2