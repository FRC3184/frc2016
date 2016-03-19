import wpilib
import math
import ITG3200
import config
from wpilib.command import Subsystem
from wpilib.interfaces import PIDSource


class DriveSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        tFrontLeft = wpilib.CANTalon(0)
        tRearLeft = wpilib.CANTalon(1)
        tFrontRight = wpilib.CANTalon(2)
        tRearRight = wpilib.CANTalon(3)
        self.rdRobotDrive = wpilib.RobotDrive(tFrontLeft, tRearLeft, tFrontRight, tRearRight)
        invertDrive = config.isPracticeBot  # False for comp bot, True for practice
        self.rdRobotDrive.setInvertedMotor(wpilib.RobotDrive.MotorType.kFrontLeft, invertDrive)
        self.rdRobotDrive.setInvertedMotor(wpilib.RobotDrive.MotorType.kFrontRight, invertDrive)
        self.rdRobotDrive.setInvertedMotor(wpilib.RobotDrive.MotorType.kRearLeft, invertDrive)
        self.rdRobotDrive.setInvertedMotor(wpilib.RobotDrive.MotorType.kRearRight, invertDrive)

        self.encRightEncoder = wpilib.Encoder(0, 1)
        self.encLeftEncoder = wpilib.Encoder(2, 3)

        encDPP = 10 * 8.5/2 * math.pi / 1440  # E4T has 1440 PPR
        # Wheels are 8.5in diameter
        self.encRightEncoder.setDistancePerPulse(encDPP)
        self.encLeftEncoder.setDistancePerPulse(encDPP)

        wpilib.Timer.delay(50/1000)
        self.gyro = ITG3200.ITG3200(wpilib.I2C.Port.kOnboard)
        wpilib.Timer.delay(50/1000)
        self.gyro.init()
        wpilib.Timer.delay(50/1000)
        self.gyro.calibrate()

        self.gyro.resetAngle()

        self.lastThrottle = 0
        self.lastTurn = 0
        self.startAngle = 0

    def drive(self, forward, turn):
        turnPow = turn
        if (abs(self.lastThrottle) < 0.1 and abs(forward) > 0.1) or abs(turn) > 0.1:
            self.startAngle = self.gyro.getAngleX()
        if abs(turn) < 0.1 and abs(forward) > 0.1 and abs(self.lastTurn) < 0.1:
            turnPow = (self.startAngle - self.gyro.getAngleX())*config.driveP

        self.rdRobotDrive.arcadeDrive(forward, turnPow)
        self.lastThrottle = forward
        self.lastTurn = turnPow

    def getAverageEncoderCount(self):
        return (self.getLeftEncoderCount() + self.getRightEncoderCount())/2

    def getLeftEncoderCount(self):
        return -self.encLeftEncoder.getDistance()

    def getRightEncoderCount(self):
        return self.encRightEncoder.getDistance()

    def resetEncoderCount(self):
        self.encRightEncoder.reset()
        self.encLeftEncoder.reset()

    def resetGyro(self):
        self.gyro.resetAngle()

    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putDouble("Left Encoder Count", self.encLeftEncoder.getDistance())
        wpilib.SmartDashboard.putDouble("Right Encoder Count", self.encRightEncoder.getDistance())
        wpilib.SmartDashboard.putDouble("Gyro Angle X", self.gyro.getAngleX())
        wpilib.SmartDashboard.putDouble("Gyro Angle Y", self.gyro.getAngleY())
        wpilib.SmartDashboard.putDouble("Gyro Angle Z", self.gyro.getAngleZ())
        wpilib.SmartDashboard.putDouble("Start Angle", self.startAngle)
