from enum import Enum
import commands2

from robot_container import RobotContainer 

class State(Enum):
    INTAKE = 0
    FEED = 0
    SCORE = 0
    CLIMB = 0

class MyRobot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__()

        self._robot_container: RobotContainer = RobotContainer()
        self._state: State = State.INTAKE

    def robotInit(self):
        pass

    def robotPeriodic(self):
        pass

    def autonomousPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass



