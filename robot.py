import wpilib
import subprocess
from ITG3200 import ITG3200
from command_based import *
from wpilib.interfaces import PIDOutput, PIDSource
from networktables import *


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
        self.gyro = ITG3200(wpilib.I2C.Port.kOnboard)
        self.gyro.init()

        self.driveAngle = 0

        # Put default config settings
        wpilib.SmartDashboard.putBoolean("Use Turn PID", True)
        wpilib.SmartDashboard.putDouble("Turn Degrees/Second", 360)
        wpilib.SmartDashboard.putDouble("Turn P", 0.3)

    def drive(self, forward, turn):
        turnPow = turn
        if wpilib.SmartDashboard.getBoolean("Use Turn PID", True):
            self.driveAngle += turn * wpilib.SmartDashboard.getDouble("Turn Degrees/Second", 360)
            deltaAngle = self.driveAngle - self.gyro.getAngleY()
            turnPow = wpilib.SmartDashboard.getDouble("Turn P", 0.3) * deltaAngle

        self.rdRobotDrive.arcadeDrive(forward, turnPow)
        
    def update(self):
        self.gyro.update()  # Accumulate
        
    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putDouble("Left Encoder Count", self.encLeftEncoder.get())
        wpilib.SmartDashboard.putDouble("Right Encoder Count", self.encRightEncoder.get())
        wpilib.SmartDashboard.putDouble("Gyro Angle X", self.gyro.getAngleX())
        wpilib.SmartDashboard.putDouble("Gyro Angle Y", self.gyro.getAngleY())
        wpilib.SmartDashboard.putDouble("Gyro Angle Z", self.gyro.getAngleZ())


class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        Kp = wpilib.SmartDashboard.getDouble("Shooter P", 0.3)
        Ki = wpilib.SmartDashboard.getDouble("Shooter I", 0.0)
        Kd = wpilib.SmartDashboard.getDouble("Shooter D", 0.0)

        self.tShooterL = wpilib.CANTalon(4)
        self.tShooterL.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        self.tShooterR = wpilib.CANTalon(5)
        self.tShooterR.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        # set pid values
        self.tShooterL.setPID(Kp, Ki, Kd)
        self.tShooterR.setPID(Kp, Ki, Kd)

        Kp = wpilib.SmartDashboard.getDouble("Articulate P", 0.3)
        Ki = wpilib.SmartDashboard.getDouble("Articulate I", 0.0)
        Kd = wpilib.SmartDashboard.getDouble("Articulate D", 0.0)

        self.tArticulate = wpilib.CANTalon(6)
        self.tArticulateEncoder = wpilib.Encoder(4, 5)
        self.articulatePID = wpilib.PIDController(Kp, Ki, Kd, self.tArticulateEncoder,
                                                  type("ArticulateWriter",  # Janky implicit class to hook up PID
                                                       (object, PIDOutput),
                                                       {"pidWrite",  self.tArticulate.set}))  # Python is so cool
        self.articulatePID.setPIDSourceType(PIDSource.PIDSourceType.kDisplacement)

    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putDouble("Left Shooter Speed", self.tShooterL.get())
        wpilib.SmartDashboard.putDouble("Right Shooter Speed", self.tShooterR.get())
        wpilib.SmartDashboard.putDouble("Target Shooter Speed", self.tShooterL.getSetpoint())

    def setPower(self, power):
        """Set shooter raw power
        :param power: Raw motor power -1 .. 1
        """

        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

        self.tShooterL.set(power)
        self.tShooterR.set(power)

    def setClosedLoopSpeed(self, rpm):
        """Use talon PID to set shooter speed
        :param rpm: The desired shooter RPM
        """

        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.Speed)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        # Multiply by ticks/rev (1024), divide by seconds/minute (60), divide by 100 for 1 second -> 10 ms
        vel = rpm * 1024 / (60 * 100)

        self.tShooterL.set(vel)
        self.tShooterR.set(vel)

    def setArticulateAngle(self, angle):
        """
        Translate angle in degrees to encoder clicks and update PID
        :param angle: 0 .. 180
        :return:
        """
        clicks = angle * (1024/360)  # Encoder clicks per degree
        self.articulatePID.setSetpoint(clicks)

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

        # Finish shooter, get data, and use regression
        return None


class TeleopCommand(Command):
    def __init__(self, subsystems):
        super().__init__()

        self.require('drive')
        self.require('shooter')
        self.driveSubsystem = subsystems['drive']
        self.shooterSubsystem = subsystems['shooter']
        self.jsLeft = wpilib.Joystick(0)
        self.jsRight = wpilib.Joystick(1)
        self.jsManip = wpilib.Joystick(2)

    def run(self):
        self.driveSubsystem.update()
        
        spencerPow = 1.0 if (self.jsLeft.getRawButton(1) or self.jsRight.getRawButton(1)) else 0.75

        power = self.jsLeft.getY() * spencerPow
        spin = -self.jsRight.getX()
        self.driveSubsystem.drive(power, spin)

        if self.jsManip.getRawButton(1):
            self.shooterSubsystem.setPower(self.jsManip.getRawAxis(2))
        else:
            self.shooterSubsystem.setPower(0)

        self.shooterSubsystem.updateSmartDashboardValues()
        self.driveSubsystem.updateSmartDashboardValues()


class MyRobot(CommandBasedRobot):
    def robotInit(self):
        subprocess.Popen("/home/lvuser/grip", shell=True)  # Start GRIP process

        self.subsystems['drive'] = DriveSubsystem()
        self.subsystems['shooter'] = ShooterSubsystem()

    def teleopInit(self):
        self.registerCommand(TeleopCommand(self.subsystems))


if __name__ == '__main__':
    wpilib.run(MyRobot)
