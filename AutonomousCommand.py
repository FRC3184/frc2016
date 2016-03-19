from wpilib.command import CommandGroup


class AutonomousCommand(CommandGroup):
    def __init__(self, l):
        super().__init__()
        for k in l:
            self.addSequential(k)
