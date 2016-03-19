import subprocess
import wpilib
import config
from wpilib.command import Scheduler, CommandGroup
from DriveSubsystem import DriveSubsystem
from ShooterSubsystem import ShooterSubsystem
from TeleopCommand import TeleopCommand
from AutonomousCommand import AutonomousCommand
from AutoDriveOverDefenseCommand import AutoDriveOverDefenseCommand
from AutoTargetCommand import AutoTargetCommand


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
        self.autonomousCommand = CommandGroup()

        self.autoPositionChooser = wpilib.SendableChooser()
        self.autoPositionChooser.getName = lambda: "Position"
        self.autoPositionChooser.addDefault("Position 1", 1)
        self.autoPositionChooser.addObject("Position 2", 2)
        self.autoPositionChooser.addObject("Position 3", 3)
        self.autoPositionChooser.addObject("Position 4", 4)
        self.autoPositionChooser.addObject("Position 5", 5)

        wpilib.SmartDashboard.putData(self.autoPositionChooser)

        self.autoDefenseChooser = wpilib.SendableChooser()
        self.autoDefenseChooser.getName = lambda: "Defense"
        self.autoDefenseChooser.addDefault("Nothing", None)
        self.autoDefenseChooser.addObject("Reach", AutoDriveOverDefenseCommand(self, power=.5, reach=True))
        self.autoDefenseChooser.addObject("Rough Terrain", AutoDriveOverDefenseCommand(self, power=.7))
        self.autoDefenseChooser.addObject("Rock Wall", AutoDriveOverDefenseCommand(self, power=.9))
        self.autoDefenseChooser.addObject("Moat", AutoDriveOverDefenseCommand(self, power=.9))
        self.autoDefenseChooser.addObject("Ramparts", AutoDriveOverDefenseCommand(self, power=.7))
        self.autoDefenseChooser.addObject("Low Bar", AutoDriveOverDefenseCommand(self, power=.5,
                                                                                 holdpos=config.articulateAngleLow))

        wpilib.SmartDashboard.putData(self.autoDefenseChooser)

        self.autoActionChooser = wpilib.SendableChooser()
        self.autoActionChooser.getName = lambda: "Action"
        self.autoActionChooser.addDefault("Nothing", None)
        self.autoActionChooser.addObject("Aim", AutoTargetCommand(self, shoot=False))
        self.autoActionChooser.addObject("Shoot", AutoTargetCommand(self, shoot=True))

        wpilib.SmartDashboard.putData(self.autoActionChooser)

    def teleopPeriodic(self):
        Scheduler.getInstance().run()

    def autonomousPeriodic(self):
        Scheduler.getInstance().run()

    def teleopInit(self):
        self.autonomousCommand.cancel()
        self.teleopCommand.start()

    def autonomousInit(self):
        defense = self.autoDefenseChooser.getSelected()
        action = self.autoActionChooser.getSelected()
        position = self.autoPositionChooser.getSelected()

        k = []
        if defense is not None:
            k += [defense]
        if action is not None:
            action.setPosition(position)
            k += [action]
        self.autonomousCommand = AutonomousCommand(k)
        self.autonomousCommand.start()

    def autoAimShooter(self):
        shooter = self.subsystems['shooter']
        drive = self.subsystems['drive']

        pitch, yawDelta = shooter.calculateShooterParams()
        drive.driveAngle += yawDelta
        shooter.setArticulateAngle(pitch)

if __name__ == '__main__':
    wpilib.run(MyRobot)
