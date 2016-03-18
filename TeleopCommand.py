import wpilib
import config
from wpilib.command import Command


def clamp(n, minN, maxN):
    if n < minN:
        return minN
    elif n > maxN:
        return maxN
    return n


class TeleopCommand(Command):
    def isFinished(self):
        return False

    def __init__(self, robot):
        super().__init__()

        subsystems = robot.subsystems
        self.pdp = robot.pdp

        self.requires(subsystems['drive'])
        self.requires(subsystems['shooter'])
        self.driveSubsystem = subsystems['drive']
        self.shooterSubsystem = subsystems['shooter']
        self.jsLeft = wpilib.Joystick(0)
        self.jsRight = wpilib.Joystick(1)
        self.jsManip = wpilib.Joystick(2)

        self.articulateAngle = 0

        self.oneeightycd = 0

    def initialize(self):
        super().initialize()
        self.shooterSubsystem.updatePID()

    def execute(self):
        super().execute()

        self.oneeightycd += 20/1000

        safePowerScale = 1.0
        if wpilib.DriverStation.getInstance().isBrownedOut():
            safePowerScale = .7

        spencerPow = 1.0 if (self.jsLeft.getRawButton(1) or self.jsRight.getRawButton(1)) else 0.75

        power = self.jsLeft.getY()**3 * spencerPow
        spin = self.jsRight.getX() * (1 if config.isPracticeBot else -1)
        if abs(spin) < 0.05:
            spin = 0

        spin **= 3
        self.driveSubsystem.drive(power, spin)  # todo implement Jaci drive

        if self.jsLeft.getRawButton(7):
            self.driveSubsystem.resetEncoderCount()

        if self.jsManip.getRawButton(9):
            self.articulateAngle = 105

        if self.jsLeft.getRawButton(8):
            self.driveSubsystem.resetGyro()
            self.driveSubsystem.driveAngle = 0

        if self.jsRight.getRawButton(2) and self.oneeightycd > 1:
            self.driveSubsystem.driveAngle += 180
            self.oneeightycd = 0
        if self.jsRight.getRawButton(5) and self.oneeightycd > 1:
            self.driveSubsystem.driveAngle += 90
            self.oneeightycd = 0
        if self.jsRight.getRawButton(4) and self.oneeightycd > 1:
            self.driveSubsystem.driveAngle -= 90
            self.oneeightycd = 0

        if self.jsManip.getRawButton(5):
            k = self.shooterSubsystem.calculateShooterParams()
            if k is not None:
                _, _, dist, _ = k
                angle = self.shooterSubsystem.shootAngle(dist)
                self.articulateAngle = angle

        if self.jsManip.getRawButton(6):
            self.shooterSubsystem.kickerOn()
        else:
            self.shooterSubsystem.kickerOff()

        if self.jsManip.getRawButton(1):
            self.shooterSubsystem.spinUp()
        elif self.jsManip.getRawButton(2):
            self.shooterSubsystem.eject()
        elif self.jsManip.getRawButton(3):
            self.shooterSubsystem.intake()
        elif self.jsManip.getRawButton(4):
            self.shooterSubsystem.spinUpBatter()
        else:
            self.shooterSubsystem.idle()

        articulatePow = (self.jsManip.getRawAxis(1) + self.jsManip.getRawAxis(3))/-2
        if abs(articulatePow) < 0.05:
            articulatePow = 0
        self.articulateAngle += articulatePow * 90/50  # degrees/hz. works, idk why
        self.articulateAngle = clamp(self.articulateAngle, config.articulateAngleLow, config.articulateAngleHigh)

        self.shooterSubsystem.setArticulateAngle(self.articulateAngle)

        wpilib.SmartDashboard.putDouble("Arm current", self.pdp.getCurrent(1))

        self.shooterSubsystem.updateSmartDashboardValues()
        self.driveSubsystem.updateSmartDashboardValues()

    def end(self):
        print("Finished Teleop Command")