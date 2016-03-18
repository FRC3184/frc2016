import wpilib
import config
from wpilib.command import Command


class AutoTargetCommand(Command):
    class State:
        SCANNING = 0
        AIMING = 1
        SHOOTING = 2
        DONE = 10

    def isFinished(self):
        pass

    def __init__(self, robot, shoot=True, position=1):
        super().__init__()
        self.driveSubsystem = robot.subsystems['drive']
        self.shooterSubsystem = robot.subsystems['shooter']
        self.shoot = shoot
        self.position = position
        rate = 360/20
        if position < 3:
            self.rotRate = rate
        else:
            self.rotRate = -rate
        self.currentState = self.State.SCANNING
        self.shooterParams = None

    def initialize(self):
        self.shooterSubsystem.setArticulateAngle(45)

    def execute(self):
        if self.currentState is self.State.SCANNING:
            k = self.shooterSubsystem.calculateShooterParams()
            if k is not None:
                self.shooterParams = k
                self.currentState = self.State.AIMING
                return None
            self.driveSubsystem.driveAngle += self.rotRate
        elif self.currentState is self.State.AIMING:
            if self.shooterParams is not None:
                pitch, yaw, y, x = self.shooterParams
                self.shooterSubsystem.setArticulateAngle(pitch)
                self.driveSubsystem.driveAngle = yaw
