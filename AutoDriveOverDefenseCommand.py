import wpilib
import config
from wpilib.command import Command
from wpilib.interfaces import PIDSource


class AutoDriveOverDefenseCommand(Command):
    def isFinished(self):
        return False

    class State:
        NOT_HIT_PLATFORM = 0
        ON_PLATFORM = 1
        PAST_PLATFORM = 2
        DONE = 10

    def __init__(self, robot, power=.7, dist=(3*12), holdpos=config.articulateAngleHigh, reach=False):
        super().__init__()

        self.requires(robot.subsystems['drive'])
        self.requires(robot.subsystems['shooter'])
        self.driveSubystem = robot.subsystems['drive']
        self.shooterSubsystem = robot.subsystems['shooter']

        self.isFinished = lambda: False
        self.reach = reach
        self.turnPow = 0
        self.currentState = self.State.NOT_HIT_PLATFORM
        self.drivePower = power
        self.dist = dist
        self.holdpos = holdpos

    def initialize(self):
        self.driveSubystem.resetEncoderCount()
        self.driveSubystem.resetGyro()  # only reset axis? or face straight
        self.driveSubystem.gyro.angleY = 0

        self.shooterSubsystem.setArticulateAngle(self.holdpos)

    def execute(self):
        wpilib.SmartDashboard.putNumber("Auto State", self.currentState)

        anglePitch = self.driveSubystem.gyro.getAngleY()

        self.shooterSubsystem.idle()
        self.driveSubystem.updateSmartDashboardValues()
        self.shooterSubsystem.updateSmartDashboardValues()

        if self.currentState is self.State.NOT_HIT_PLATFORM:
            self.driveSubystem.drive(self.drivePower, 0)

            if anglePitch > 10:
                self.currentState = (self.State.ON_PLATFORM if not self.reach else self.State.DONE)

        elif self.currentState is self.State.ON_PLATFORM:
            self.driveSubystem.drive(self.drivePower, 0)

            if anglePitch < -10:
                self.currentState = self.State.PAST_PLATFORM
                self.driveSubystem.resetEncoderCount()

        elif self.currentState is self.State.PAST_PLATFORM:
            self.driveSubystem.drive(self.drivePower, 0)

            if abs(self.driveSubystem.getLeftEncoderCount()) > self.dist or \
               abs(self.driveSubystem.getRightEncoderCount()) > self.dist:
                self.currentState = self.State.DONE

        elif self.currentState is self.State.DONE:
            self.driveSubystem.drive(0, 0)
            self.isFinished = lambda: True
