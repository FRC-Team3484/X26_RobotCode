# Subsystems
DRIVETRAIN_ENABLED: bool = True
VISION_ENABLED: bool = True

CLIMBER_ENABLED: bool = False
FLYWHEEL_ENABLED: bool = True
FEEDER_ENABLED: bool = True
INDEXER_ENABLED: bool = True
INTAKE_ENABLED: bool = True
TURRET_ENABLED: bool = True
FEED_TARGET_ENABLED: bool = True
LEDS_ENABLED: bool = True

# Commands
LAUNCH_WHILE_MOVING_ENABLED: bool = True
USE_SIMPLE_TELEOP_COMMAND: bool = False

# Data Logging
LOGGING_ENABLED: bool = True

# Demo Mode
#     If true, the robot will drive slower in demo mode (under the TestDriveCommand)
#     Used for demostrating the robot
SLOW_DRIVE_IN_DEMO: bool = True