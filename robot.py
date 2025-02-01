import wpilib as wp
from Utils.Configs.test import TestRobotConfig as RobotConfig
from Utils.constants import BRAKE, COAST



class MyRobot(wp.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.robot: RobotConfig = RobotConfig()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        wp.Timer.reset()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        self.robot.drive.tankDrive(
            -self.robot.driver.getTwist(), self.robot.driver.getY()
        )

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""
