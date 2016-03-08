import wpilib


class BlazeTalon(wpilib.CANTalon):
    def __init__(self, deviceNumber, encoder=None):
        super().__init__(deviceNumber)
        self.encoder = encoder

        self.autoStallProtect = True

        self._zeroVoltageThreshold = .1
        self._zeroSpeedThreshold = 10  # pretty gratuitous

    def _configZeroVoltageThreshold(self, zeroVoltageThreshold):
        self._zeroVoltageThreshold = zeroVoltageThreshold

    def _configZeroSpeedThreshold(self, zeroSpeedThreshold):
        self._zeroSpeedThreshold = zeroSpeedThreshold

    def autoStallProtect(self, autoStallProtect=None):
        if autoStallProtect is not None:
            self.autoStallProtect = autoStallProtect
        return self.autoStallProtect

    def isStalled(self):
        if self.encoder is None:
            return False  # No way of knowing. Check current?

        if abs(self.getOutputVoltage()) > self._zeroVoltageThreshold and\
           abs(self.encoder.getSpeed()) < self._zeroSpeedThreshold:
            return True
        return False
