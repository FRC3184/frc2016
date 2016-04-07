import wpilib
import config
import vision
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
        self.toCancel = None

        self.requires(subsystems['drive'])
        self.requires(subsystems['shooter'])
        self.requires(subsystems['tomahawk'])
        self.driveSubsystem = subsystems['drive']
        self.shooterSubsystem = subsystems['shooter']
        self.tomahawkSubsystem = subsystems['tomahawk']
        self.jsLeft = wpilib.Joystick(0)
        self.jsRight = wpilib.Joystick(1)
        self.jsManip = wpilib.Joystick(2)

        self.articulateAngle = 0

        self.oneeightycd = 0

        self.timer_var = 0.2
        self.aim_value = wpilib.SmartDashboard.getNumber("Aim at", 0)
        self.seekCenterX = -1.1
        self.seekState = 0
        self.timer = wpilib.Timer()

    def initialize(self):
        super().initialize()

    def execute(self):
        super().execute()

        if not wpilib.DriverStation.getInstance().isOperatorControl() or self.jsManip.getRawButton(8):
            return None

        self.oneeightycd += 20/1000

        safePowerScale = 1.0
        if wpilib.DriverStation.getInstance().isBrownedOut():
            safePowerScale = .7

        spencerPow = 1.0 if (self.jsLeft.getRawButton(1) or self.jsRight.getRawButton(1)) else 0.75

        power = self.jsLeft.getY()**3 * spencerPow
        spin = self.jsRight.getX() * (1 if config.isPracticeBot else -1)
        if abs(spin) < 0.05:
            spin = 0

        if spencerPow != 1:
            spin *= config.spinFactor
        self.driveSubsystem.drive(power, spin)

        if self.jsLeft.getRawButton(7):
            self.driveSubsystem.resetEncoderCount()

        if self.jsManip.getRawButton(9):
            self.articulateAngle = config.articulateAngleHigh
        elif self.jsManip.getRawButton(10):
            self.articulateAngle = config.articulateAngleLow

        if self.jsManip.getPOV() == 0:
            self.tomahawkSubsystem.set(config.tomahawkPower)
        elif self.jsManip.getPOV() == 180:
            self.tomahawkSubsystem.set(-config.tomahawkPower)
        else:
            self.tomahawkSubsystem.set(0)

        # if self.jsManip.getRawButton(8):
        #    self.seekCenterX = self.seekCenterX
        # else:
        if self.jsManip.getRawButton(6):
            self.shooterSubsystem.kickerOn()
        else:
            self.shooterSubsystem.kickerOff()

        if self.jsManip.getRawButton(7):
            self.shooterSubsystem.articulatePID.disable()
            self.shooterSubsystem.updateArticulate(self.jsManip.getY())
        else:
            self.shooterSubsystem.articulatePID.enable()

        if self.jsManip.getRawButton(1):
            self.shooterSubsystem.spinUp()
        elif self.jsManip.getRawButton(2):
            self.shooterSubsystem.eject()
        elif self.jsManip.getRawButton(3):
            self.shooterSubsystem.intake()
        elif self.jsManip.getRawButton(4):
            self.shooterSubsystem.spinUpBatter()
        elif self.jsManip.getRawButton(8):
            self.seekCenterX = self.seekCenterX
        else:
            self.shooterSubsystem.idle()

        articulatePow = (self.jsManip.getRawAxis(1) + self.jsManip.getRawAxis(3))/-2
        if abs(articulatePow) < 0.05:
            articulatePow = 0
        self.articulateAngle += articulatePow * config.articulateRate
        self.articulateAngle = clamp(self.articulateAngle, config.articulateAngleLow, config.articulateAngleHigh)

        self.shooterSubsystem.setArticulateAngle(self.articulateAngle)

        k = vision.calculateShooterParams(self.shooterSubsystem.getAngle())
        if k is not None:
            wpilib.SmartDashboard.putNumber("Shoot Angle", k[0])
            wpilib.SmartDashboard.putNumber("Distance From Tower", k[2])

        self.shooterSubsystem.updateSmartDashboardValues()
        self.driveSubsystem.updateSmartDashboardValues()

    def end(self):
        print("Finished Teleop Command")

    def setToCancel(self, command):
        self.toCancel = command
        pass
