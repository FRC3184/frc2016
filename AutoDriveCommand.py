import wpilib
import config
from wpilib.command import Command
from wpilib.interfaces import PIDSource
from OrphanCommand import OrphanCommand


class AutoDriveCommand(OrphanCommand):
    def isFinished(self):
        return False

    class State:
        DRIVING = 0
        DONE = 10

    def __init__(self, robot, power=.7, dist=(3*12), holdpos=config.articulateAngleHigh,
                 delay=0):
        super().__init__()

        self.requires(robot.subsystems['drive'])
        self.requires(robot.subsystems['shooter'])
        self.driveSubystem = robot.subsystems['drive']
        self.shooterSubsystem = robot.subsystems['shooter']
        self.timer = wpilib.Timer()
        self.timerStarted = 0

        self.isFinished = lambda: False
        self.turnPow = 0
        self.drivePower = power
        self.dist = dist
        self.holdpos = holdpos
        self.delay = delay
        self.delayCount = 0

    def initialize(self):
        self.driveSubystem.resetEncoderCount()
        self.driveSubystem.resetGyro()  # only reset axis? or face straight
        self.driveSubystem.gyro.angleY = 0
        self.delayCount = 0

        self.shooterSubsystem.setArticulateAngle(self.holdpos)

    def execute(self):
        wpilib.SmartDashboard.putNumber("Auto State", self.currentState)

        self.shooterSubsystem.idle()
        self.driveSubystem.updateSmartDashboardValues()
        self.shooterSubsystem.updateSmartDashboardValues()

        if self.delayCount < self.delay:
            self.delayCount += 20/1000
            self.driveSubystem.drive(0, 0)
            return None

        if self.currentState is self.State.DRIVING:
            self.driveSubystem.drive(self.drivePower, 0)

            if abs(self.driveSubystem.getLeftEncoderCount()) > self.dist or \
                abs(self.driveSubystem.getRightEncoderCount()) > self.dist:
                self.currentState = self.State.DONE

        elif self.currentState is self.State.DONE:
            self.driveSubystem.drive(0, 0)
            self.isFinished = lambda: True
