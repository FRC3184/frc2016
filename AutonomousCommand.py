from wpilib.command import CommandGroup
from AutoDriveOverDefenseCommand import AutoDriveOverDefenseCommand


class AutonomousCommand(CommandGroup):
    def __init__(self, l):
        super().__init__()
        for k in l:
            self.addSequential(k)
