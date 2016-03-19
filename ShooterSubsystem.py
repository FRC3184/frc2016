import wpilib
import math
import config
from wpilib.command import Subsystem
from networktables import NumberArray, NetworkTable


class Contour:
    def __init__(self, centerX, centerY, area, height, width):
        self.centerX = centerX
        self.centerY = centerY
        self.area = area
        self.height = height
        self.width = width


class ShooterSubsystem(Subsystem):
    def __init__(self, robot):
        super().__init__()

        self.pdp = robot.pdp

        self.currentAverageValues = 20
        self.armCurrentValues = [0]*self.currentAverageValues

        self.angleOffset = config.articulateAngleLow

        self.tShooterL = wpilib.CANTalon(4)
        self.tShooterR = wpilib.CANTalon(5)

        self.tShooterL.setInverted(not config.isPracticeBot)  # True for comp bot, false for practice
        self.tShooterR.setInverted(not config.isPracticeBot)  # True for comp bot, false for practice

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

        self.defaultArticulateP = 0.011
        self.defaultArticulateI = 0.001
        self.defaultArticulateD = 0.012

        self.articulateEncoder = wpilib.Encoder(4, 5)
        self.articulateEncoder.reset()

        self.vArticulate = wpilib.VictorSP(1)
        self.articulateEncoder.setDistancePerPulse((1440/(360*4)) * 1.5)  # Clicks per degree / Magic numbers
        self.articulatePID = wpilib.PIDController(Kp=config.articulateKp, Ki=config.articulateKi, Kd=config.articulateKd,
                                                  source=self.getAngle,
                                                  output=self.updateArticulate)
        self.articulatePID.setOutputRange(-1, +1)
        self.articulatePID.setInputRange(config.articulateAngleLow, config.articulateAngleHigh)
        self.articulatePID.setSetpoint(35)
        self.articulatePID.setPercentTolerance(1/130)
        self.articulatePID.enable()

        self.vIntake = wpilib.VictorSP(0)
        self.sKicker = wpilib.Servo(2)

        self.limLow = wpilib.DigitalInput(6)
        self.limHigh = wpilib.DigitalInput(7)

    def updatePID(self):
        Kp = wpilib.SmartDashboard.getDouble("Articulate P", self.defaultArticulateP)
        Ki = wpilib.SmartDashboard.getDouble("Articulate I", self.defaultArticulateI)
        Kd = wpilib.SmartDashboard.getDouble("Articulate D", self.defaultArticulateD)

        self.articulatePID.setPID(Kp, Ki, Kd)

    def updateArticulate(self, articulatePow):
        current = self.pdp.getCurrent(1)  # Channel for test bot
        self.armCurrentValues += [current]
        self.armCurrentValues = self.armCurrentValues[1:]
        avg = sum(self.armCurrentValues)/self.currentAverageValues

        if articulatePow < 0:
            if not self.isHittingLow():
                self.vArticulate.set(articulatePow)
            else:
                self.vArticulate.set(0)
                self.articulateEncoder.reset()
                self.angleOffset = config.articulateAngleLow
        elif articulatePow > 0:
            if not self.isHittingHigh():
                self.vArticulate.set(articulatePow)
            else:
                self.vArticulate.set(0)
                self.articulateEncoder.reset()
                self.angleOffset = config.articulateAngleHigh

        else:
            self.vArticulate.set(articulatePow)

    def kickerOn(self):
        self.sKicker.set(0)

    def kickerOff(self):
        self.sKicker.set(.3)

    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putBoolean("Right Shooter Encoder present",
                                         self.tShooterR.isSensorPresent(wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Absolute) == wpilib.CANTalon.FeedbackDeviceStatus.Present)
        wpilib.SmartDashboard.putBoolean("Left Shooter Encoder present",
                                         self.tShooterL.isSensorPresent(wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Absolute) == wpilib.CANTalon.FeedbackDeviceStatus.Present)

        wpilib.SmartDashboard.putDouble("Left Shooter Speed", self.tShooterL.getSpeed())
        wpilib.SmartDashboard.putDouble("Right Shooter Speed", self.tShooterR.getSpeed())
        wpilib.SmartDashboard.putDouble("Target Shooter Speed", self.tShooterL.getSetpoint())

        wpilib.SmartDashboard.putNumber("Articulate Angle", self.getAngle())
        wpilib.SmartDashboard.putNumber("Articulate Set Angle", self.articulatePID.getSetpoint())

        dist = 0
        params = self.calculateShooterParams()
        pitch = 0
        if params is not None:
            pitch, _, dist, _ = params

        wpilib.SmartDashboard.putNumber("Distance From Tower", dist)
        wpilib.SmartDashboard.putNumber("Shoot Angle", pitch)

    def setPower(self, power):
        """Set shooter raw power
        :param power: Raw motor power -1 .. 1
        """

        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

        self.tShooterL.set(power)
        self.tShooterR.set(power)

    def spinUpBatter(self):
        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.Speed)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        vel = config.batterShootSpeed

        f = config.batterShootKf
        p = config.batterShootKp
        self.tShooterL.setF(f)
        self.tShooterR.setF(f)
        self.tShooterL.setP(p)
        self.tShooterR.setP(p)

        self.tShooterL.set(vel)
        self.tShooterR.set(vel)

    def spinUp(self):
        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.Speed)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        vel = config.shootSpeed

        f = config.shootKf
        p = config.shootKp
        self.tShooterL.setF(f)
        self.tShooterR.setF(f)
        self.tShooterL.setP(p)
        self.tShooterR.setP(p)

        self.tShooterL.set(vel)
        self.tShooterR.set(vel)

    def eject(self):
        self.setPower(0.6)
        intakeBarPow = -1.0

        self.vIntake.set(intakeBarPow)

    def intake(self):
        intakeBarPow = 1.0

        self.setPower(-0.6)
        self.vIntake.set(intakeBarPow)

    def idle(self):
        self.vIntake.set(0)
        self.tShooterL.set(0)
        self.tShooterR.set(0)

    def setArticulateAngle(self, angle):
        """
        Translate angle in degrees to encoder clicks and update PID
        :param angle: -15 .. 105
        :return:
        """
        self.articulatePID.setSetpoint(angle)

    def getAngle(self):
        return self.articulateEncoder.getDistance()+self.angleOffset

    def calculateShooterParams(self):
        hullTable = NetworkTable.getTable("GRIP/goalConvexHulls")
        centerXs = NumberArray()
        centerYs = NumberArray()
        areas = NumberArray()
        heights = NumberArray()
        widths = NumberArray()
        hullTable.retrieveValue("centerX", centerXs)
        hullTable.retrieveValue("centerY", centerYs)
        hullTable.retrieveValue("area", areas)
        hullTable.retrieveValue("height", heights)
        hullTable.retrieveValue("width", widths)

        avgLen = (len(centerXs) + len(centerYs) + len(areas) + len(heights) + len(widths))/5
        if round(avgLen) != avgLen:  # It happens. I don't know why.
            return None

        contours = []
        for i in range(len(centerXs)):
            contours.append(Contour(centerXs[i], centerYs[i], areas[i], heights[i], widths[i]))
        if len(contours) < 1:
            return None  # Couldn't find any vision targets

        contours = sorted(contours, key=lambda x: x.area, reverse=True)  # Sort contours by area in descending size
        largest = contours[0]                                            # Maybe use width?

        # Y is long axis of field
        # X is short axis of field

        distanceY = self.distanceFromTower(largest.width)
        distanceX = 0  # distance between robot aim plane and center of goal. calculate from centerX and distanceY (in)

        anglePitch = self.angleFromGoalWidth(largest.width)
        angleYawDelta = math.degrees(math.atan2(distanceY, distanceX))

        return anglePitch, angleYawDelta, distanceY, distanceX

    def angleFromGoalWidth(self, goalw):
        return -0.000020941*(goalw**3) + 0.0088104*(goalw**2) - 0.95513*goalw + 66.004

    def distanceFromTower(self, goalw):
        return 0.0068478*(goalw**2) - 2.9095*goalw + 346.76

    def shootAngle(self, distance):
        return 0.001458*(distance**2) - 0.48272*distance + 75.393

    def isHittingLow(self):
        return not self.limLow.get()

    def isHittingHigh(self):
        return self.limHigh.get()