import subprocess
import wpilib
import config
from wpilib.command import Scheduler
from DriveSubsystem import DriveSubsystem
from ShooterSubsystem import ShooterSubsystem
from TeleopCommand import TeleopCommand
from AutonomousCommand import AutonomousCommand
from AutoDriveOverDefenseCommand import AutoDriveOverDefenseCommand


class MyRobot(wpilib.IterativeRobot):
    def __init__(self):
        super().__init__()
        self.pdp = wpilib.PowerDistributionPanel()
        self.subsystems = {}

    def robotInit(self):
        subprocess.Popen("/home/lvuser/grip", shell=True)  # Start GRIP process

        self.subsystems['drive'] = DriveSubsystem()
        self.subsystems['shooter'] = ShooterSubsystem(self)

        self.teleopCommand = TeleopCommand(self)
        self.autonomousCommand = AutonomousCommand(self)

        self.autoPositionChooser = wpilib.SendableChooser()
        self.autoPositionChooser.addObject("Position 1", 1)
        self.autoPositionChooser.addObject("Position 2", 2)
        self.autoPositionChooser.addObject("Position 3", 3)
        self.autoPositionChooser.addObject("Position 4", 4)
        self.autoPositionChooser.addObject("Position 5", 5)

        self.autoDefenseChooser = wpilib.SendableChooser()
        self.autoDefenseChooser.addObject("Nothing", None)
        self.autoDefenseChooser.addObject("Reach", AutoDriveOverDefenseCommand(self, power=.5, reach=True))
        self.autoDefenseChooser.addObject("Rough Terrain", AutoDriveOverDefenseCommand(self, power=.7))
        self.autoDefenseChooser.addObject("Rock Wall", AutoDriveOverDefenseCommand(self, power=.9))
        self.autoDefenseChooser.addObject("Moat", AutoDriveOverDefenseCommand(self, power=.9))
        self.autoDefenseChooser.addObject("Ramparts", AutoDriveOverDefenseCommand(self, power=.7))
        self.autoDefenseChooser.addObject("Low Bar", AutoDriveOverDefenseCommand(self, power=.5,
                                                                                 holdpos=config.articulateAngleLow))

        wpilib.SmartDashboard.putData(self.autoDefenseChooser)

        self.autoActionChooser = wpilib.SendableChooser()
        self.autoActionChooser.addObject("Nothing", None)
        self.autoActionChooser.addObject("Aim", None)
        self.autoActionChooser.addObject("Shoot", None)

        wpilib.SmartDashboard.putData(self.autoActionChooser)

    def teleopPeriodic(self):
        Scheduler.getInstance().run()

    def autonomousPeriodic(self):
        Scheduler.getInstance().run()

    def teleopInit(self):
        self.autonomousCommand.cancel()
        self.teleopCommand.start()

    def autonomousInit(self):
        self.autonomousCommand.start()

    def autoAimShooter(self):
        shooter = self.subsystems['shooter']
        drive = self.subsystems['drive']

        pitch, yawDelta = shooter.calculateShooterParams()
        drive.driveAngle += yawDelta
        shooter.setArticulateAngle(pitch)

if __name__ == '__main__':
    wpilib.run(MyRobot)
