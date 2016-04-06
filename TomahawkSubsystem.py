import wpilib
from wpilib.command import Subsystem


class TomahawkSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.tTomahawk = wpilib.Talon(3)

    def set(self, power):
        self.tTomahawk.set(power)
