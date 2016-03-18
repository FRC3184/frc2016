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

        self.turnPID = wpilib.PIDController(Kp=0.015, Ki=0.00, Kd=0.015, source=self.gyro.xGyro,
                                            output=self.setTurnPow)
        self.gyro.resetAngle()

        self.useTurnPID = True

        self.driveAngle = 0
        self.turnPow = 0
        self.turnPID.setPIDSourceType(PIDSource.PIDSourceType.kDisplacement)
        self.turnPID.setOutputRange(-1, 1)
        self.turnPID.setAbsoluteTolerance(2)
        self.turnPID.enable()

        # Put default config settings
        wpilib.SmartDashboard.putBoolean("Use Turn PID", True)
        wpilib.SmartDashboard.putDouble("Turn Degrees per Second", 1080)

    def setTurnPow(self, turnPow):
        if abs(turnPow) < 0.37:
            turnPow = 0.37 * abs(turnPow)/turnPow

        if self.turnPID.onTarget():

            turnPow = 0
        self.turnPow = turnPow

    def drive(self, forward, turn):
        turnPow = turn
        if self.useTurnPID:
            self.driveAngle += turn * wpilib.SmartDashboard.getDouble("Turn Degrees per Second", 1080) / 50
            self.turnPID.setSetpoint(self.driveAngle)
            turnPow = self.turnPow

        self.rdRobotDrive.arcadeDrive(forward, turnPow)

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
        wpilib.SmartDashboard.putDouble("Gyro Setpoint", self.turnPID.getSetpoint())
