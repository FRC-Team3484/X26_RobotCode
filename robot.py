from enum import Enum
import commands2
from wpilib import DriverStation, SmartDashboard

from src.auton_generator import AutonGenerator
from src.oi import DemoInterface, DriverInterface, OperatorInterface, SysIDInterface, TestInterface1, TestInterface2
from src.robot_container import RobotContainer
from src.test_container import TestContainer 
from src.constants import RobotConstants

class State(Enum):
    INTAKE = 0
    FEED = 1
    SCORE = 2
    GOTO_CLIMB = 3

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
        self._state = State.INTAKE

    def teleopPeriodic(self):
        match self._state:
            case State.INTAKE:
                self.stop_teleop_commands()
                self._robot_container.teleop_intake_commands.schedule()

            case State.FEED:
                if self._operator_interface.get_left_feed_point() or self._operator_interface.get_right_feed_point():
                    self.stop_teleop_commands()
                    self._robot_container.teleop_feed_commands.schedule()
                else:
                    self._state = State.INTAKE

            case State.SCORE:
                if self._operator_interface.get_launcher():
                    self.stop_teleop_commands()
                    self._robot_container.teleop_launch_commands.schedule()
                else:
                    self._state = State.INTAKE

            case State.GOTO_CLIMB:
                if self._driver_interface.get_goto_climb():
                    self.stop_teleop_commands()
                    self._robot_container.goto_climb_commands.schedule()
                else:
                    self._state = State.INTAKE

    def teleopExit(self):
        self._state = State.INTAKE

    def testInit(self):
        self._test_commands = self._test_container.get_test_command()
        self._test_commands.schedule()

    def testPeriodic(self):
        pass

    def testExit(self):
        self._test_commands.cancel()

    def stop_teleop_commands(self) -> None:
        self._robot_container.teleop_intake_commands.cancel()
        self._robot_container.teleop_feed_commands.cancel()
        self._robot_container.teleop_launch_commands.cancel()
        self._robot_container.goto_climb_commands.cancel()


