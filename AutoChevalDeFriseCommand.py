import wpilib
import config

from OrphanCommand import OrphanCommand


class AutoChevalDeFriseCommand(OrphanCommand):
    def __init__(self, robot, holdpos=config.articulateAngleHigh, power=-.4, pastPower=-.6, delay=0, dist=100):
        super().__init__()

        subsystems = robot.subsystems

        self.requires(subsystems['drive'])
        self.requires(subsystems['shooter'])
        self.requires(subsystems['tomahawk'])
        self.driveSubsystem = subsystems['drive']
        self.shooterSubsystem = subsystems['shooter']
        self.tomahawkSubsystem = subsystems['tomahawk']

        self.drivePower = power
        self.pastPower = pastPower
        self.dist = dist
        self.holdpos = holdpos
        self.delay = delay
        self.delayCount = 0

        self.timer = wpilib.Timer()
        self.timerStarted = False

        self.currentState = self.State.NOT_HIT_PLATFORM

    def isFinished(self):
        return False

    class State:
        NOT_HIT_PLATFORM = 0
        ON_PLATFORM = 1
        CHEVAL = 2
        PAST_PLATFORM = 3
        DONE = 10

    def initialize(self):
        self.driveSubsystem.resetEncoderCount()
        self.delayCount = 0

        self.shooterSubsystem.setArticulateAngle(self.holdpos)

    def execute(self):
        wpilib.SmartDashboard.putNumber("Auto State", self.currentState)

        self.shooterSubsystem.idle()
        self.driveSubsystem.updateSmartDashboardValues()
        self.shooterSubsystem.updateSmartDashboardValues()

        if self.delayCount < self.delay:
            self.delayCount += 20/1000
            self.driveSubsystem.drive(0, 0)
            return None

        if self.currentState is self.State.NOT_HIT_PLATFORM:
            if self.timerStarted == 0:
                self.timer.reset()
                self.timer.start()
                self.timerStarted = True

            self.driveSubsystem.drive(self.drivePower, 0)
            if self.timer.get() >= 2:
                self.currentState = self.State.ON_PLATFORM
                self.timer.stop()
                self.timerStarted = False

        elif self.currentState is self.State.ON_PLATFORM:
            if self.timerStarted == 0:
                self.timer.reset()
                self.timer.start()
                self.timerStarted = True

            self.driveSubsystem.drive(-self.drivePower, 0)
            if self.timer.get() >= 0.2:
                self.currentState = self.State.CHEVAL
                self.timer.stop()
                self.timerStarted = False

        elif self.currentState is self.State.CHEVAL:
            if self.timerStarted == 0:
                self.timer.reset()
                self.timer.start()
                self.timerStarted = True

            self.shooterSubsystem.setArticulateAngle(-15)
            if self.timer.get() >= 1:
                self.currentState = self.State.PAST_PLATFORM
                self.timer.stop()
                self.timerStarted = False

        elif self.currentState is self.State.PAST_PLATFORM:
            self.driveSubsystem.drive(self.pastPower, 0)

            if abs(self.driveSubsystem.getLeftEncoderCount()) > self.dist or \
               abs(self.driveSubsystem.getRightEncoderCount()) > self.dist:
                self.currentState = self.State.DONE

        elif self.currentState is self.State.DONE:
            self.driveSubsystem.drive(0, 0)
            self.isFinished = lambda: True
