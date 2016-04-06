import subprocess
import wpilib
import config
import OrphanCommand
# import soul

from DataLogger import DataLogger
from wpilib.buttons import JoystickButton
from wpilib.command import Scheduler, CommandGroup
from DriveSubsystem import DriveSubsystem
from ShooterSubsystem import ShooterSubsystem
from TomahawkSubsystem import TomahawkSubsystem
from TeleopCommand import TeleopCommand
from AutonomousCommand import AutonomousCommand
from AutoDriveCommand import AutoDriveCommand
from AutoTargetCommand import AutoTargetCommand
from AutoChevalDeFriseCommand import AutoChevalDeFriseCommand


class MyRobot(wpilib.IterativeRobot):
    def __init__(self):
        super().__init__()
        self.pdp = wpilib.PowerDistributionPanel()
        self.subsystems = {}
        self.datalogger = DataLogger(logfile=open("robot.log", mode='a'),
                                     curtime=wpilib.DriverStation.getInstance().getMatchTime)

    def robotInit(self):
        subprocess.Popen("/home/lvuser/grip", shell=True)  # Start GRIP process

        self.subsystems['drive'] = DriveSubsystem()
        self.subsystems['shooter'] = ShooterSubsystem(self)
        self.subsystems['tomahawk'] = TomahawkSubsystem()

        self.teleopCommand = TeleopCommand(self)
        self.autonomousCommand = CommandGroup()

        self.autoPositionChooser = wpilib.SendableChooser()
        self.autoPositionChooser.addDefault("Position 1", 1)
        self.autoPositionChooser.addObject("Position 2", 2)
        self.autoPositionChooser.addObject("Position 3", 3)
        self.autoPositionChooser.addObject("Position 4", 4)
        self.autoPositionChooser.addObject("Position 5", 5)

        wpilib.SmartDashboard.putData("PositionChooser", self.autoPositionChooser)

        self.autoDefenseChooser = wpilib.SendableChooser()
        self.autoDefenseChooser.addDefault("Nothing", None)
        self.autoDefenseChooser.addObject("Reach", AutoDriveCommand(self, power=.5, dist=100))
        self.autoDefenseChooser.addObject("Rough Terrain", AutoDriveCommand(self, power=.7, dist=200))
        self.autoDefenseChooser.addObject("Rock Wall", AutoDriveCommand(self, power=.9, dist=200,
                                                                        holdpos=config.articulateAngleHigh))
        self.autoDefenseChooser.addObject("Moat", AutoDriveCommand(self, power=-.9, dist=270))
        self.autoDefenseChooser.addObject("Ramparts", AutoDriveCommand(self, power=-.7,
                                                                       dist=200,
                                                                       holdpos=config.articulateAngleHigh))
        self.autoDefenseChooser.addObject("Low Bar", AutoDriveCommand(self, power=.6,
                                                                      dist=200,
                                                                      holdpos=config.articulateAngleLow))
        self.autoDefenseChooser.addObject("Portcullis", AutoDriveCommand(self, power=.6,
                                                                         dist=200,
                                                                         holdpos=config.articulateAngleLow))
        self.autoDefenseChooser.addObject("Cheval de Frise", AutoChevalDeFriseCommand(self))

        wpilib.SmartDashboard.putData("DefenseChooser", self.autoDefenseChooser)

        self.autoActionChooser = wpilib.SendableChooser()
        self.autoActionChooser.addDefault("Nothing", None)
        self.autoActionChooser.addObject("Aim", AutoTargetCommand(self, shoot=False))
        self.autoActionChooser.addObject("Shoot", AutoTargetCommand(self, shoot=True))

        wpilib.SmartDashboard.putData("ActionChooser", self.autoActionChooser)

        wpilib.SmartDashboard.putNumber("Aim at", 150)

        self.datalogger.ready()

    def teleopPeriodic(self):
        Scheduler.getInstance().run()
        self.datalogger.log_data()

    def autonomousPeriodic(self):
        Scheduler.getInstance().run()
        self.datalogger.log_data()

    def teleopInit(self):
        self.autonomousCommand.cancel()
        self.jsAutoTargetButton = JoystickButton(wpilib.Joystick(2), 8)
        self.jsAutoTargetButton.whenPressed(AutoTargetCommand(self))
        self.jsAutoTargetButton.whenReleased(self.teleopCommand)

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
        for cmd in k:
            cmd.setParent(None)
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
