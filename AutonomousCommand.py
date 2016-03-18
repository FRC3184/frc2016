from wpilib.command import CommandGroup
from AutoDriveOverDefenseCommand import AutoDriveOverDefenseCommand


class AutonomousCommand(CommandGroup):
    def __init__(self, robot):
        super().__init__()
        self.addSequential(AutoDriveOverDefenseCommand(robot))
