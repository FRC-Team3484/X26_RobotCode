from enum import Enum
import commands2
from wpilib import DriverStation, SmartDashboard

from auton_generator import AutonGenerator
from oi import DemoInterface, DriverInterface, OperatorInterface, SysIDInterface, TestInterface1, TestInterface2
from robot_container import RobotContainer
from test_container import TestContainer 
from constants import RobotConstants

class State(Enum):
    INTAKE = 0
    FEED = 0
    SCORE = 0
    CLIMB = 0

class MyRobot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__(RobotConstants.TICK_RATE)

        self._driver_interface: DriverInterface = DriverInterface()
        self._operator_interface: OperatorInterface = OperatorInterface()

        self._test_interface_1: TestInterface1 = TestInterface1()
        self._test_interface_2: TestInterface2 = TestInterface2()
        self._demo_interface: DemoInterface = DemoInterface()
        self._sysid_interface: SysIDInterface = SysIDInterface()

        self._robot_container: RobotContainer = RobotContainer(self._driver_interface, self._operator_interface)
        self._auton_generator: AutonGenerator = AutonGenerator()
        self._test_container: TestContainer = TestContainer(self._test_interface_1, self._test_interface_2, self._demo_interface, self._sysid_interface, self._robot_container)
        self._state: State = State.INTAKE

        self._test_commands: commands2.Command = commands2.InstantCommand()

    def robotInit(self):
        pass

    def robotPeriodic(self):
        SmartDashboard.putNumber("Battery Voltage", DriverStation.getBatteryVoltage())
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime())

    def autonomousPeriodic(self):
        pass

    def autonomousInit(self):
        self._auton_generator.get_auton_command().schedule()

    def autonomousExit(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    def testInit(self):
        self._test_commands = self._test_container.get_test_command()
        self._test_commands.schedule()

    def testPeriodic(self):
        pass

    def testExit(self):
        self._test_commands.cancel()



