import wpilib
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

        self.gyro = wpilib.ADXRS450_Gyro()
        #self.gyro.calibrate()
        #self.gyro.reset()

        self.accel = wpilib.ADXL345_SPI(1, 3)
    def drive(self, forward, turn):
        self.rdRobotDrive.arcadeDrive(forward, turn)
        angle = self.gyro.getRate()
        accel = self.accel.getAcceleration(0)
        if angle != 0:
            wpilib.DriverStation.reportError("Angle: " + str(angle), False)
        if accel != 0:
            wpilib.DriverStation.reportError("Accel: " + str(accel), False)
class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.tShooter = wpilib.Talon(0)
    def setPower(self, power):
        self.tShooter.set(power)
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
        spencerPow = 1.0 if (self.jsLeft.getRawButton(1) or self.jsRight.getRawButton(1)) else 0.5
        
        power = self.jsLeft.getY() * spencerPow
        spin = -self.jsRight.getX()
        self.driveSubsystem.drive(power, spin)

        if self.jsManip.getRawButton(1):
            self.shooterSubsystem.setPower(self.jsManip.getRawAxis(2))
        else:
            self.shooterSubsystem.setPower(0)
class MyRobot(CommandBasedRobot):
    def robotInit(self):
        self.subsystems['drive'] = DriveSubsystem()
        self.subsystems['shooter'] = ShooterSubsystem()   
        
    def teleopInit(self):
        self.registerCommand(TeleopCommand(self.subsystems))        

if __name__ == '__main__':
    wpilib.run(MyRobot)
