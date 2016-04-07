import wpilib
import config
import vision
from wpilib.command import Command
from OrphanCommand import OrphanCommand


class AutoTargetCommand(OrphanCommand):
    class State:
        SCANNING = 0
        AIMING = 1
        SHOOTING = 2
        DONE = 10

    def isFinished(self):
        return False

    def __init__(self, robot, shoot=True, position=1):
        super().__init__()
        self.driveSubsystem = robot.subsystems['drive']
        self.shooterSubsystem = robot.subsystems['shooter']
        self.shoot = shoot
        self.rate = 0.45
        self.delayTime = 0
        self.setPosition(position)
        self.currentState = self.State.SCANNING
        self.shooterParams = None

        self.timer_var = 0.2
        self.aim_value = wpilib.SmartDashboard.getNumber("Aim at", 0)
        self.seekCenterX = -1.1
        self.seekState = 0
        self.timer = wpilib.Timer()

        self.jsManip = wpilib.Joystick(2)
        self.lastPress = False

    def setPosition(self, position):
        self.position = position
        if position < 3:
            self.rotRate = self.rate
        else:
            self.rotRate = -self.rate

    def initialize(self):

        self.currentState = self.State.SCANNING
        self.timer_var = 0.2
        self.aim_value = wpilib.SmartDashboard.getNumber("Aim at", 0)
        self.seekCenterX = -1.1
        self.seekState = 0
        self.timer.reset()

    def execute(self):
        wpilib.SmartDashboard.putNumber("Auto State", self.currentState)

        if self.jsManip.getRawButton(5):
            self.initialize()

        if not (wpilib.DriverStation.getInstance().isAutonomous() or self.jsManip.getRawButton(8)):
            return None

        if self.currentState is not self.State.SHOOTING:
            pass
            #self.shooterSubsystem.idle()
            #self.shooterSubsystem.kickerOff()

        if self.currentState is self.State.SCANNING:
            self.shooterSubsystem.setArticulateAngle(config.angle_seek)
            k = vision.calculateShooterParams(self.shooterSubsystem.getAngle())
            if k is not None:
                self.currentState = self.State.AIMING
                return None
            self.driveSubsystem.drive(0, self.rotRate)

        elif self.currentState is self.State.AIMING:
            self.aim_value = wpilib.SmartDashboard.getNumber("Aim at", 155)
            self.do_aim = True

            k = vision.calculateShooterParams(self.shooterSubsystem.getAngle())
            if k is not None:
                pitch, yaw, dist, center_x = k
                self.shooterSubsystem.setArticulateAngle(pitch)
                wpilib.SmartDashboard.putNumber("Distance From Tower", dist)
                wpilib.SmartDashboard.putNumber("Shoot Angle", pitch)
                wpilib.SmartDashboard.putNumber("Azimuth", yaw)
                wpilib.SmartDashboard.putNumber("CenterX", center_x)
                if self.do_aim:
                    if center_x < self.aim_value - config.targeting_max_error and self.seekState < 2:
                        if self.seekCenterX != center_x:
                            self.seekCenterX = center_x
                            if self.seekState == 0:
                                self.seekState = 1
                                self.timer.reset()
                                self.timer.start()
                    elif center_x > self.aim_value + config.targeting_max_error and self.seekState < 2:
                        if self.seekCenterX != center_x:
                            self.seekCenterX = center_x
                            if self.seekState == 0:
                                self.seekState = 1
                                self.timer.reset()
                                self.timer.start()
                    elif self.aim_value - config.targeting_max_error <= \
                            center_x <= self.aim_value + config.targeting_max_error:
                        if self.seekState != 2:
                            self.seekState = 2
                            self.timer.stop()
                            self.timer.reset()
                            self.timer.start()
                    else:
                        self.seekState = 0
            else:
                wpilib.SmartDashboard.putNumber("CenterX", -1.1)
                self.seekCenterX = -1.1

            if self.do_aim:
                if self.seekState == 1:
                    if self.timer.get() > self.timer_var:
                        self.driveSubsystem.drive(0, 0)
                        if self.timer.get() > self.timer_var + .5:
                            self.seekState = 0
                            self.timer.stop()
                            self.timer.reset()
                    else:
                        # self.shooterSubsystem.setArticulateAngle(config.highgoal_outerworks_angle)
                        if self.seekCenterX < self.aim_value - 15:
                            if self.seekCenterX < 0:
                                self.timer_var = 0.1
                            else:
                                self.timer_var = 0.2
                            self.driveSubsystem.drive(0, -.5)
                        elif self.seekCenterX < self.aim_value - config.targeting_max_error:
                            self.timer_var = 0.1
                            self.driveSubsystem.drive(0, -.4)
                        elif self.seekCenterX > self.aim_value + 15:
                            self.timer_var = 0.2
                            self.driveSubsystem.drive(0, .5)
                        elif self.seekCenterX > self.aim_value + config.targeting_max_error:
                            self.timer_var = 0.1
                            self.driveSubsystem.drive(0, .4)
                        else:
                            self.driveSubsystem.drive(0, 0)
                if self.seekState == 2:
                    if self.timer.get() >= 1 and self.seekState == 2:
                        self.shooterSubsystem.spinUp()
                        if self.timer.get() >= 3.5 and self.seekState == 2:
                            self.shooterSubsystem.kickerOn()
                            if self.timer.get() >= 4:
                                self.timer.stop()
                                self.timer.reset()
                                self.timer.start()
                                self.currentState = self.State.DONE
                                self.shooterSubsystem.kickerOff()
                                self.shooterSubsystem.idle()
                                self.seekState = 3

            else:
                self.seekState = 0
                self.seekCenterX = -1.1
                self.timer.stop()
                self.timer.reset()

            if self.do_aim:
                self.seekCenterX = self.seekCenterX

        elif self.currentState is self.State.SHOOTING:
            pass
            self.shooterSubsystem.spinUp()
            if self.timer.get() > 3.5:
                self.shooterSubsystem.kickerOn()
                self.delayTime += 20 / 1000
                if self.delayTime >= 1:
                    self.currentState = self.State.DONE
                    self.timer.reset()
            else:
                self.shooterSubsystem.kickerOff()
        elif self.currentState is self.State.DONE:
            self.shooterSubsystem.idle()
            self.shooterSubsystem.kickerOff()
            if self.timer.get() > 1:
                self.currentState = self.State.SCANNING

        self.shooterSubsystem.updateSmartDashboardValues()
