from frc3484.controls import XboxControllerMap, Input
# Constants
controller = XboxControllerMap
class UserInterface:
    class TestConstants1:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        QUASI_FWD_BUTTON: Input = XboxControllerMap.A_BUTTON
        QUASI_REV_BUTTON: Input = XboxControllerMap.B_BUTTON
        DYNAMIC_FWD_BUTTON: Input = XboxControllerMap.X_BUTTON
        DYNAMIC_REV_BUTTON: Input = XboxControllerMap.Y_BUTTON
        
        FLYWHEEL_INPUT: Input = controller.RIGHT_TRIGGER
        INDEXER_INPUT: Input = controller.LEFT_TRIGGER
        TURRET_INPUT: Input = controller.LEFT_JOY_X
        CLIMBER_INPUT: Input = controller.RIGHT_JOY_X
    
    class TestConstants2:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        INTAKE_ROLLER_INPUT: Input = controller.RIGHT_TRIGGER
        FEEDER_INPUT: Input = controller.LEFT_JOY_Y
        INTAKE_PIVOT_INPUT: Input = controller.RIGHT_JOY_Y
    
    class DemoController:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

        FLYWHEEL_LEFT_INPUT: Input = controller.LEFT_TRIGGER
        FLYWHEEL_RIGHT_INPUT: Input = controller.RIGHT_TRIGGER

        TURRET_INPUT: Input = controller.DPAD_X
        FEEDER_INPUT: Input = controller.A_BUTTON
        EJECT_INPUT: Input = controller.B_BUTTON
        INTAKE_INPUT: Input = controller.X_BUTTON
        
        THROTTLE_INPUT: Input = controller.LEFT_JOY_Y
        STRAFE_INPUT: Input = controller.LEFT_JOY_X
        ROTATE_INPUT: Input = controller.RIGHT_JOY_X




        
# Subsystems
class AgitatorSubsystemConstants:
    pass

class ClimberSubsystemConstants:
    pass

class IntakeSubsystemConstants:
    pass

class TurretSubsystemConstants:
    pass

class FlywheelSubsystemConstants:
    pass

class IndexerSubsystenConstants:
    pass

class LauncherSubsystemConstants:
    pass
