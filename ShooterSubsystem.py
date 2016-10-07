import wpilib
import math
import config
from wpilib.interfaces import PIDSource
from wpilib.command import Subsystem


class ShooterSubsystem(Subsystem):
    def __init__(self, robot):
        super().__init__()

        self.pdp = robot.pdp
        self.datalogger = robot.datalogger

        self.currentAverageValues = 20
        self.armCurrentValues = [0]*self.currentAverageValues

        self.angleOffset = config.articulateAngleLow

        self.tShooterL = wpilib.CANTalon(4)
        self.tShooterR = wpilib.CANTalon(5)

        self.tShooterL.setInverted(not config.isPracticeBot)  # True for comp bot, false for practice
        self.tShooterR.setInverted(config.isPracticeBot)  # True for comp bot, false for practice

        self.shooterEncoderType = wpilib.CANTalon.FeedbackDevice.CtreMagEncoder_Relative

        self.tShooterL.setFeedbackDevice(self.shooterEncoderType)
        self.tShooterR.setFeedbackDevice(self.shooterEncoderType)

        self.shooterLPID = wpilib.PIDController(Kp=config.shooterLKp, Ki=config.shooterLKi, Kd=0,
                                                kf=config.shooterLKf,
                                                source=lambda: self.tShooterL.getSpeed() *
                                                (-1 if config.isPracticeBot else 1),
                                                output=self.tShooterL.set, period=20/1000)  # Fast PID Loop
        self.shooterLPID.setOutputRange(-1, 1)
        self.shooterLPID.setInputRange(-18700/3, 18700/3)

        self.shooterRPID = wpilib.PIDController(Kp=config.shooterRKp, Ki=config.shooterRKi, Kd=0.000,
                                                kf=config.shooterRKf,
                                                source=lambda: self.tShooterR.getSpeed() *
                                                (-1 if not config.isPracticeBot else 1),
                                                output=self.tShooterR.set, period=20/1000)  # Fast PID Loop
        self.shooterRPID.setOutputRange(-1, 1)
        self.shooterRPID.setInputRange(-18700/3, 18700/3)

        self.shooterLPID.setSetpoint(0)
        self.shooterRPID.setSetpoint(0)
        self.shooterLPID.enable()
        self.shooterRPID.enable()

        self.articulateEncoder = wpilib.Encoder(4, 5)
        self.articulateEncoder.reset()

        self.vArticulate = wpilib.VictorSP(1)
        self.articulateEncoder.setDistancePerPulse(
            (1440/(360*4)) * 1.5)  # Clicks per degree / Magic numbers
        self.articulatePID = wpilib.PIDController(Kp=config.articulateKp, Ki=config.articulateKi, Kd=config.articulateKd,
                                                  source=self.getAngle,
                                                  output=self.updateArticulate)
        self.articulatePID.setOutputRange(-1, +1)
        self.articulatePID.setInputRange(config.articulateAngleLow, config.articulateAngleHigh)
        self.articulatePID.setSetpoint(35)
        self.articulatePID.setPercentTolerance(100/130)
        self.articulatePID.enable()

        self.vIntake = wpilib.VictorSP(0)
        self.sKicker = wpilib.Servo(2)

        self.limLow = wpilib.DigitalInput(6)
        self.limHigh = wpilib.DigitalInput(7)

        self.lastKicker = False

        # Data logger
        self.datalogger.add_data_source("Left Shooter Motor", self.tShooterL.getSpeed)
        self.datalogger.add_data_source("Right Shooter Motor", self.tShooterR.getSpeed)
        self.datalogger.add_data_source("Articulate Angle", self.getAngle)

    def updateArticulate(self, articulatePow):
        """
        Used as an output for the arm PID loop. Checks limit switches before outputting power.
        :param articulatePow: power: -1..1
        """
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
        """
        Set the kicker servo to on state
        """
        self.sKicker.set(.3 if not config.isPracticeBot else 0)
        if not self.lastKicker:
            self.datalogger.event("Fire!")
        self.lastKicker = True

    def kickerOff(self):
        """
        Set the kicker servo to idle state
        """
        self.sKicker.set(.6 if not config.isPracticeBot else .3)
        self.lastKicker = False

    def updateSmartDashboardValues(self):
        """
        Update the SmartDashboard values
        """
        present_status = wpilib.CANTalon.FeedbackDeviceStatus.Present
        wpilib.SmartDashboard.putBoolean("Right Shooter Encoder present",
                                         self.tShooterR.isSensorPresent(self.shooterEncoderType) ==
                                         present_status)
        wpilib.SmartDashboard.putBoolean("Left Shooter Encoder present",
                                         self.tShooterL.isSensorPresent(self.shooterEncoderType) ==
                                         present_status)

        wpilib.SmartDashboard.putDouble("Left Shooter Speed", self.tShooterL.getSpeed())
        wpilib.SmartDashboard.putDouble("Right Shooter Speed", self.tShooterR.getSpeed())

        wpilib.SmartDashboard.putNumber("Articulate Angle", self.getAngle())
        wpilib.SmartDashboard.putNumber("Articulate Set Angle", self.articulatePID.getSetpoint())

    def setPower(self, power):
        """
        Set shooter raw power
        :param power: Raw motor power -1 .. 1
        """
        if self.shooterLPID.enabled:
            self.shooterLPID.disable()
        if self.shooterRPID.enabled:
            self.shooterRPID.disable()

        self.tShooterL.set(power)
        self.tShooterR.set(power)

    def setVelocity(self, vel):
        """
        Use PID control to set shooter speed
        :param vel: The velocity setpoint, in RPM
        """
        self.shooterLPID.enable()
        self.shooterRPID.enable()

        self.shooterLPID.setSetpoint(vel)
        self.shooterRPID.setSetpoint(vel)

    def spinUpBatter(self):
        """
        SpinUp at a lower speed, tuned for the batter shot
        """
        self.setVelocity(config.batterShootSpeed)

    def spinUp(self):
        """
        SpinUp State
        """
        self.setVelocity(config.shootSpeed)

    def eject(self):
        """
        Eject state
        """
        self.setPower(0.6)
        self.vIntake.set(config.intake_bar_pow)

    def intake(self):
        """
        Intake state
        """
        self.setPower(-1 * config.shooter_intake_pow)
        self.vIntake.set(-1 * config.intake_bar_pow)

    def idle(self):
        """
        Shooter idle state
        Set shooter to 0
        """
        self.shooterLPID.disable()
        self.shooterRPID.disable()
        self.vIntake.set(0)
        self.tShooterL.set(0)
        self.tShooterR.set(0)

    def setArticulateAngle(self, angle):
        """
        Translate angle in degrees to encoder clicks and update PID
        :param angle: degrees: -15 .. 105
        """
        self.articulatePID.setSetpoint(angle)

    def getAngle(self):
        """
        Get the angle of the shooter arm, as reported by the encoder
        :return: The angle in degrees of the shooter arm
        """
        return self.articulateEncoder.getDistance()+self.angleOffset

    def isHittingLow(self):
        """
        Bottom arm limit switch state
        :return: Whether the arm is at the bottom of its arc
        """
        return not self.limLow.get()

    def isHittingHigh(self):
        """
        Top arm limit switch state
        :return: Whether the arm is at the top of its arc
        """
        # The switch on the practice bot has opposite configuration (NC vs NO)
        return (self.limHigh.get()) if config.isPracticeBot else (not self.limHigh.get())
