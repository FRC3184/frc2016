import math
import subprocess
import wpilib
import time
import sys

from networktables import *
from wpilib.interfaces import PIDOutput, PIDSource

from ITG3200 import ITG3200
import ITG3200
from command_based import *


class Contour:
    def __init__(self, centerX, centerY, area, height, width):
        self.centerX = centerX
        self.centerY = centerY
        self.area = area
        self.height = height
        self.width = width


class DriveSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        tFrontLeft = wpilib.CANTalon(0)
        tRearLeft = wpilib.CANTalon(1)
        tFrontRight = wpilib.CANTalon(2)
        tRearRight = wpilib.CANTalon(3)
        self.rdRobotDrive = wpilib.RobotDrive(tFrontLeft, tRearLeft, tFrontRight, tRearRight)
        self.encRightEncoder = wpilib.Encoder(0, 1)
        self.encLeftEncoder = wpilib.Encoder(2, 3)

        encDPP = 8.5 * math.pi / 1440  # E4T has 1440 PPR
                                       # Wheels are 8.5in diameter
        self.encRightEncoder.setDistancePerPulse(encDPP)
        self.encLeftEncoder.setDistancePerPulse(encDPP)

        self.gyro = ITG3200.ITG3200(wpilib.I2C.Port.kOnboard)
        self.gyro.init()
        self.gyro.calibrate()

        self.driveAngle = 0

        # Put default config settings
        wpilib.SmartDashboard.putBoolean("Use Turn PID", False)
        wpilib.SmartDashboard.putDouble("Turn Degrees/Second", 360)
        wpilib.SmartDashboard.putDouble("Turn P", 0.3)

    def drive(self, forward, turn):
        turnPow = turn
        if wpilib.SmartDashboard.getBoolean("Use Turn PID", False):
            self.driveAngle += turn * wpilib.SmartDashboard.getDouble("Turn Degrees/Second", 360)
            deltaAngle = self.driveAngle - self.gyro.getAngleZ()
            turnPow = wpilib.SmartDashboard.getDouble("Turn P", 0.3) * deltaAngle

        self.rdRobotDrive.arcadeDrive(forward, turnPow)

    def getAverageEncoderCount(self):
        return (self.encLeftEncoder.get() + self.encRightEncoder.get())/2

    def resetEncoderCount(self):
        self.encRightEncoder.reset()
        self.encLeftEncoder.reset()

    def resetGyro(self):
        self.gyro.resetAngle()
        
    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putDouble("Left Encoder Count", self.encLeftEncoder.get())
        wpilib.SmartDashboard.putDouble("Right Encoder Count", self.encRightEncoder.get())
        wpilib.SmartDashboard.putDouble("Gyro Angle X", self.gyro.getAngleX())
        wpilib.SmartDashboard.putDouble("Gyro Angle Y", self.gyro.getAngleY())
        wpilib.SmartDashboard.putDouble("Gyro Angle Z", self.gyro.getAngleZ())


class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.tShooterL = wpilib.CANTalon(4)
        self.tShooterR = wpilib.CANTalon(5)

        self.tShooterL.setInverted(True)

        self.tShooterL.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.tShooterR.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.tShooterL.setControlMode(wpilib.CANTalon.ControlMode.Speed)
        self.tShooterR.setControlMode(wpilib.CANTalon.ControlMode.Speed)

        Kp = 0
        Ki = 0
        Kd = 0

        # set pid values
        self.tShooterL.setPID(Kp, Ki, Kd)
        self.tShooterR.setPID(Kp, Ki, Kd)

        Kp = wpilib.SmartDashboard.getDouble("Articulate P", 0.01)
        Ki = wpilib.SmartDashboard.getDouble("Articulate I", 0.0)
        Kd = wpilib.SmartDashboard.getDouble("Articulate D", 0.0)

        self.articulateEncoder = wpilib.Encoder(4, 5)
        self.articulateEncoder.reset()

        self.vArticulate = wpilib.VictorSP(1)
        self.articulateEncoder.setDistancePerPulse((1440/(360*4)) * 1.5)  # Clicks per degree
        # articulatePID = wpilib.PIDController(Kp=Kp, Ki=Ki, Kd=Kd, 
        #                                      source=lambda: articulateEncoder.getDistance(),
        #                                     output=tArticulate)
        # articulatePID.setPIDSourceType(PIDSource.PIDSourceType.kDisplacement)
        # articulatePID.setOutputRange(-0.5, +0.5)
        # articulatePID.setInputRange(-180, 180)
        # articulatePID.setSetpoint(0)
        # articulatePID.enable()
        
        self.vIntake = wpilib.VictorSP(0)
        self.sKicker = wpilib.Servo(2)

        self.limLow = wpilib.DigitalInput(6)

    def kickerOn(self):
        self.sKicker.set(.5)

    def kickerOff(self):
        self.sKicker.set(.2)

    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putBoolean("Right Shooter Encoder present", 
                                         self.tShooterR.isSensorPresent(wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Absolute) == wpilib.CANTalon.FeedbackDeviceStatus.Present)
        wpilib.SmartDashboard.putBoolean("Left Shooter Encoder present", 
                                         self.tShooterL.isSensorPresent(wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Absolute) == wpilib.CANTalon.FeedbackDeviceStatus.Present)

        wpilib.SmartDashboard.putDouble("Left Shooter Speed", self.tShooterL.getSpeed())
        wpilib.SmartDashboard.putDouble("Right Shooter Speed", self.tShooterR.getSpeed())
        wpilib.SmartDashboard.putDouble("Target Shooter Speed", self.tShooterL.getSetpoint())
        
        wpilib.SmartDashboard.putNumber("Articulate Angle", self.articulateEncoder.getDistance())

    def setPower(self, power):
        """Set shooter raw power
        :param power: Raw motor power -1 .. 1
        """

        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

        self.tShooterL.set(power)
        self.tShooterR.set(power)

    def spinUp(self):
        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.Speed)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        vel = 5400
    
        f = 0.00015
        p = 1 * 0.0000273
        self.tShooterL.setF(f)
        self.tShooterR.setF(f)
        self.tShooterL.setP(p)
        self.tShooterR.setP(p)

        self.tShooterL.set(vel)
        self.tShooterR.set(vel)
    
    def eject(self):
        self.setPower(0.6)
        intakeBarPow = -1.0
        intakeSpd = -3000
    
        self.vIntake.set(intakeBarPow)

        # f = 0.00009
        # p = 0.00003

        # self.tShooterL.setF(f)
        # self.tShooterR.setF(f)
        # self.tShooterL.setP(p)
        # self.tShooterR.setP(p)

        # self.tShooterL.set(intakeSpd)
        # self.tShooterR.set(intakeSpd)
        
    def intake(self):
        intakeBarPow = 1.0
        intakeSpd = 3000

        self.setPower(-0.6)
        self.vIntake.set(intakeBarPow)
        #
        # f = -0.00009
        # p = 0.00003
        #
        # self.tShooterL.setF(f)
        # self.tShooterR.setF(f)
        # self.tShooterL.setP(p)
        # self.tShooterR.setP(p)
        #
        # self.tShooterL.set(intakeSpd)
        # self.tShooterR.set(intakeSpd)

    def idle(self):
        self.vIntake.set(0)
        self.tShooterL.set(0)
        self.tShooterR.set(0)

    def setArticulateAngle(self, angle):
        """
        Translate angle in degrees to encoder clicks and update PID
        :param angle: -15 .. 120
        :return:
        """
        self.articulatePID.setSetpoint(angle+15)

    def getAngle(self):
        return self.articulateEncoder.getDistance()-15

    def calculateShooterParams(self):
        gripTable = NetworkTable.getTable("GRIP/myContoursReport")
        centerXs = NumberArray()
        centerYs = NumberArray()
        areas = NumberArray()
        heights = NumberArray()
        widths = NumberArray()
        gripTable.retrieveValue("centerX", centerXs)
        gripTable.retrieveValue("centerY", centerYs)
        gripTable.retrieveValue("areas", areas)
        gripTable.retrieveValue("heights", heights)
        gripTable.retrieveValue("widths", widths)

        contours = []
        for i in range(len(centerXs)):
            contours.append(Contour(centerXs[i], centerYs[i], areas[i], heights[i], widths[i]))
        if len(contours) < 1:
            return None  # Couldn't find any vision targets

        contours = sorted(contours, key=lambda x: x.area, reverse=True)  # Sort contours by area in descending size
        largest = contours[0]                                            # Maybe use width?

        # Y is long axis of field
        # X is short axis of field

        distanceY = 0  # distance to base of tower, use regression to determine from width of contour or centerY? (inches)
        distanceX = 0  # distance between robot aim plane and center of goal. calculate from centerX and distanceY (in)

        anglePitch = math.degrees(math.atan2(7*12 + 1, distanceY))  # High goal is 7 ft 1 inch above carpet. Increase to center?
        angleYawDelta = math.degrees(math.atan2(distanceY, distanceX))

        return anglePitch, angleYawDelta

    def isHittingLow(self):
        return not self.limLow.get()


class TeleopCommand(Command):
    def __init__(self, robot):
        super().__init__()

        subsystems = robot.subsystems

        self.require('drive')
        self.require('shooter')
        self.driveSubsystem = subsystems['drive']
        self.shooterSubsystem = subsystems['shooter']
        self.jsLeft = wpilib.Joystick(0)
        self.jsRight = wpilib.Joystick(1)
        self.jsManip = wpilib.Joystick(2)

    def run(self):
        safePowerScale = 1.0
        if wpilib.DriverStation.getInstance().isBrownedOut():
            safePowerScale = .7
        
        spencerPow = 1.0 if (self.jsLeft.getRawButton(1) or self.jsRight.getRawButton(1)) else 0.75

        power = self.jsLeft.getY() * spencerPow
        spin = -self.jsRight.getX()
        self.driveSubsystem.drive(power * safePowerScale, spin)

        if self.jsManip.getRawButton(1):
            self.shooterSubsystem.kickerOn()
        else:
            self.shooterSubsystem.kickerOff()

        if self.jsManip.getRawButton(2):
            self.shooterSubsystem.spinUp()
        elif self.jsManip.getRawButton(4):
            self.shooterSubsystem.eject()
        elif self.jsManip.getRawButton(5):
            self.shooterSubsystem.intake()
        else:
            self.shooterSubsystem.idle()

        articulatePow = self.jsManip.getY()
        if articulatePow < 0:
            if not self.shooterSubsystem.isHittingLow():
                self.shooterSubsystem.tArticulate.set(self.jsManip.getY())
            else:
                self.shooterSubsystem.tArticulate.set(0)
                self.shooterSubsystem.articulateEncoder.reset()
        else:
            self.shooterSubsystem.tArticulate.set(self.jsManip.getY())

        if self.jsManip.getRawButton(6):
            self.shooterSubsystem.resetArticulate()
        if self.jsManip.getRawButton(7):  # TODO potentiometer control
            self.shooterSubsystem.setArticulateAngle(0)
        if self.jsManip.getRawButton(8):
            self.shooterSubsystem.setArticulateAngle(90)
            
        self.shooterSubsystem.updateSmartDashboardValues()
        self.driveSubsystem.updateSmartDashboardValues()


class AutoDriveOverDefenseCommand(Command):
    def __init__(self, robot):
        super().__init__()

        self.require('drive')
        self.driveSubystem = robot.subsystems['drive']

        self.isFinished = lambda: False

    def start(self):
        self.driveSubystem.resetEncoders()
        self.driveSubystem.resetGyro()  # only reset axis? or face straight

    def run(self):
        axis = ITG3200.Axis.X

        self.driveSubystem.drive(.5, 0)

        if self.driveSubystem.getAverageEncoderCount() > 5 * 12:  # idk lol 5 ft or TODO timeout and gyro info
            self.isFinished = lambda: True  # I LOVE PYTHON


class MyRobot(CommandBasedRobot):
    def robotInit(self):
        subprocess.Popen("/home/lvuser/grip", shell=True)  # Start GRIP process


        self.subsystems['drive'] = DriveSubsystem()
        self.subsystems['shooter'] = ShooterSubsystem()

    def teleopInit(self):
        self.clearCommands()
        self.teleopCommand = TeleopCommand(self)
        self.registerCommand(self.teleopCommand)

    def autoAimShooter(self):
        shooter = self.subsystems['shooter']
        drive = self.subsystems['drive']

        pitch, yawDelta = shooter.calculateShooterParams()
        drive.driveAngle += yawDelta
        shooter.setArticulateAngle(pitch)

if __name__ == '__main__':
    wpilib.run(MyRobot)
