import wpilib
import config
import vision
from wpilib.command import Command


class AutoTargetCommand(Command):
    class State:
        SCANNING = 0
        AIMING = 1
        SHOOTING = 2
        DONE = 10

    def isFinished(self):
        return False

    def __init__(self, robot, shoot=True, position=1):
        super().__init__()
        self.driveSubsystem = robot.subsystems['drive']
        self.shooterSubsystem = robot.subsystems['shooter']
        self.shoot = shoot
        self.rate = 0.4
        self.setPosition(position)
        self.currentState = self.State.SCANNING
        self.shooterParams = None
        self.delayTime = 0
        self.turnPow = 0
        self.turnPID = wpilib.PIDController(Kp=0.05, Ki=0.001, Kd=0, source=self.driveSubsystem.gyro.xGyro,
                                            output=self.setTurnPow)

    def setTurnPow(self, turn):
        self.turnPow = turn

    def setPosition(self, position):
        self.position = position
        if position < 3:
            self.rotRate = self.rate
        else:
            self.rotRate = -self.rate

    def initialize(self):
        self.shooterSubsystem.setArticulateAngle(45)

    def execute(self):
        if self.currentState is not self.State.SHOOTING:
            self.shooterSubsystem.idle()
            self.shooterSubsystem.kickerOff()

        if self.currentState is self.State.SCANNING:
            k = vision.calculateShooterParams(self.shooterSubsystem.getAngle())
            if k is not None:
                self.shooterParams = k
                pitch, yaw, y, x = self.shooterParams
                self.shooterSubsystem.setArticulateAngle(pitch)

                self.driveSubsystem.resetGyro()
                self.turnPID.setSetpoint(yaw)
                self.turnPID.enable()

                self.currentState = self.State.AIMING
                return None
            self.driveSubsystem.drive(0, self.rotRate)

        elif self.currentState is self.State.AIMING:
            self.driveSubsystem.drive(0, self.turnPow)
            if self.shooterSubsystem.articulatePID.onTarget() and self.turnPID.onTarget():
                self.currentState = (self.State.SHOOTING if self.shoot else self.State.DONE)

        elif self.currentState is self.State.SHOOTING:
            self.shooterSubsystem.spinUp()
            if (abs(self.shooterSubsystem.tShooterL.getSpeed()) > config.shootSpeed and
               abs(self.shooterSubsystem.tShooterR.getSpeed()) > config.shootSpeed):
                self.shooterSubsystem.kickerOn()
                self.delayTime += 20/1000
                if self.delayTime >= 1:
                    self.currentState = self.State.DONE
            else:
                self.shooterSubsystem.kickerOff()
        elif self.currentState is self.State.DONE:
            self.isFinished = lambda: True


