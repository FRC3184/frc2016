import wpilib



class ITG3200(wpilib.SensorBase):
    def __init__(self, port, addr=DEFAULT_ADDR):
        self.i2c = wpilib.I2C(port, addr)
        self.addr = addr
    def init(self):
        pass

    def read_short_from_register(self, register, count):
        buf = self.i2c.read(register, count)
        return (buf[0] << 8) | buf[1]
    def writeBits(self, register, bit, numBits, value):
        rawData = self.i2c.read(register, 1)
        newValue = self.updateByte(rawData[0], bit, numBits, value)
        self.i2c.write(register, newValue)
    def get_rotation_x(self):
        return self.read_short_from_register(RA_GYRO_XOUT, 2)
    def get_rotation_y(self):
        return self.read_short_from_register(RA_GYRO_YOUT, 2)
    def get_rotation_z(self):
        return self.read_short_from_register(RA_GYRO_ZOUT, 2)

    def getSmartDashboardType(self):
        pass

    def updateByte(self, param, bit, numBits, value):
        pass


ADDR_AD0_LOW = 0x68
ADDR_AD0_HIGH = 0x69
DEFAULT_ADDR = ADDR_AD0_LOW

RA_GYRO_XOUT = 0x1D
RA_GYRO_YOUT = 0x1F
RA_GYRO_ZOUT = 0x21