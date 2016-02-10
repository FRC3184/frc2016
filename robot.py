import wpilib
import subprocess
from ITG3200 import ITG3200
from command_based import *


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

    def drive(self, forward, turn):
        self.rdRobotDrive.arcadeDrive(forward, turn)
        
    def update(self):
        self.gyro.update()
        
    def updateSmartDashboardValues(self):
        wpilib.SmartDashboard.putDouble("Left Encoder Count", self.encLeftEncoder.get())
        wpilib.SmartDashboard.putDouble("Right Encoder Count", self.encRightEncoder.get())
        wpilib.SmartDashboard.putDouble("Gyro Angle X", self.gyro.getAngleX())
        wpilib.SmartDashboard.putDouble("Gyro Angle Y", self.gyro.getAngleY())
        wpilib.SmartDashboard.putDouble("Gyro Angle Z", self.gyro.getAngleZ())

class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        p = .3  # Set these for testing
        i = 0
        d = 0

        self.tShooterL = wpilib.CANTalon(4)
        self.tShooterL.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        self.tShooterL.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        self.tShooterR = wpilib.CANTalon(5)
        self.tShooterR.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        self.tShooterR.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

        # set pid values
        self.tShooterL.setPID(p, i, d)
        self.tShooterR.setPID(p, i, d)

        self.tArticulate = wpilib.CANTalon(6)

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

        vel = rpm * 1024 / (60 * 100)  # Multiply by ticks/rev, divide by seconds/minute, divide by 100 for 1 second -> 10 ms

        self.tShooterL.set(vel)
        self.tShooterR.set(vel)


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
        subprocess.Popen("/home/lvuser/grip", shell=True)

        self.subsystems['drive'] = DriveSubsystem()
        self.subsystems['shooter'] = ShooterSubsystem()

    def teleopInit(self):
        self.registerCommand(TeleopCommand(self.subsystems))


if __name__ == '__main__':
    wpilib.run(MyRobot)
