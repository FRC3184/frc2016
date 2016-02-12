# Original: https://github.com/bussell/SparkFun6Dof/blob/master/GyroITG3200.java
# Translated by Nick Schatz for FRC 3184 Blaze Robotics
# All credit is given to original author

# ============================================
# GyroITG3200 device library code is placed under the MIT license
# Copyright (c) 2011 by Jeff Rowberg
# Copyright (c) 2015 Joe Bussell
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ===============================================

import wpilib
import sys
from wpilib.interfaces import PIDSource

# Constants
ADDR_AD0_LOW = 0x68
ADDR_AD0_HIGH = 0x69
DEFAULT_ADDR = ADDR_AD0_LOW

RA_GYRO_XOUT_H = 0x1D
RA_GYRO_XOUT_L = 0x1E
RA_GYRO_YOUT_H = 0x1F
RA_GYRO_YOUT_L = 0x20
RA_GYRO_ZOUT_H = 0x21
RA_GYRO_ZOUT_L = 0x22
RA_TEMP_OUT_H = 0x1B
RA_TEMP_OUT_L = 0x1C
RA_WHO_AM_I = 0x00
RA_DLPF_FS = 0x16
RA_PWR_MGM = 0x3E
RA_INT_CFG = 0x17
RA_SMPLRT_DIV = 0x15

DF_FS_SEL_BIT = 4
DF_FS_SEL_LENGTH = 2

INTCFG_ITG_RDY_EN_BIT = 2
INTCFG_RAW_RDY_EN_BIT = 0

PWR_CLK_SEL_BIT = 2
PWR_CLK_SEL_LENGTH = 3
PWR_SLEEP_BIT = 6
PWR_STBY_XG_BIT = 5
PWR_STBY_YG_BIT = 4
PWR_STBY_ZG_BIT = 3

DEVID_BIT = 6
DEVID_LENGTH = 6

FULLSCALE_2000 = 0x03

CLOCK_PLL_XGYRO = 0x01


def rshift(val, n):
    return (val % 0x100000000) >> n


class ITG3200(PIDSource):
    def __init__(self, port, addr=DEFAULT_ADDR):
        self.i2c = wpilib.I2C(port, addr)
        self.addr = addr
        self.pidAxis = self.Axis.X
        self.setPIDSourceType(PIDSource.PIDSourceType.kRate)
        self.reset()

    def readI2C(self, register, count):
        return self.i2c.read(register, count)

    def writeI2C(self, register, data):
        try:
            self.i2c.write(register, data)
        except:
            e = sys.exc_info()[0]
            wpilib.DriverStation.reportError("Unhandled Exception when writing to gyro: {}".format(e), False)

    def reset(self):
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0

    def update(self, dt=20):
        """

        :param dt: Delta time in milliseconds
        :return:
        """
        
        # TODO: Filter out data? Use accel?
        
        real_dt = dt/1000 # Turn ms into seconds

        self.angleX += self.getRateX() * real_dt
        self.angleY += self.getRateY() * real_dt
        self.angleZ += self.getRateZ() * real_dt

    # PID stuff    
    class Axis:
        X = 0
        Y = 1
        Z = 2

    def getPIDSourceType(self):
        return self.pidSourceType

    def setPIDSourceType(self, pidSource):
        self.pidSourceType = pidSource

    def setPIDAxis(self, axis):
        self.pidAxis = axis

    def pidGet(self):
        if self.getPIDSourceType() == PIDSource.PIDSourceType.kRate:
            return self.getRate(self.pidAxis)
        else:
            return self.getAngle(self.pidAxis)

    # Power on and prepare for general usage.
    # This will activate the gyroscope, so be sure to adjust the power settings
    # after you call this method if you want it to enter standby mode, or another
    # less demanding mode of operation. This also sets the gyroscope to use the
    # X-axis gyro for a clock source. Note that it doesn't have any delays in the
    # routine, which means you might want to add ~50ms to be safe if you happen
    # to need to read gyro data immediately after initialization. The data will
    # flow in either case, but the first reports may have higher error offsets.
    def init(self):
        if not self.testConnection():
            wpilib.DriverStation.reportError("Could not connect to ITG3200", False)
        self.setFullScaleRange(FULLSCALE_2000)
        self.setClockSource(CLOCK_PLL_XGYRO)
        self.setIntDeviceReadyEnabled(True)
        self.setIntDataReadyEnabled(True)
    
    def getAngleX(self):
        return self.angleX
        
    def getAngleY(self):
        return self.angleY
        
    def getAngleZ(self):
        return self.angleZ
        
    def getAngle(self, axis):
        if axis == self.Axis.X:
            return self.getAngleX()
        if axis == self.Axis.Y:
            return self.getAngleY()
        if axis == self.Axis.Z:
            return self.getAngleZ()
    
    def getRate(self, axis):
        if axis == self.Axis.X:
            return self.getRateX()
        if axis == self.Axis.Y:
            return self.getRateY()
        if axis == self.Axis.Z:
            return self.getRateZ()

    # Gyro Interface

    def readShortFromRegister(self, register, count):
        buf = self.readI2C(register, count)
        return (buf[0] << 8) | buf[1]

    def writeBits(self, register, bit, numBits, value):
        rawData = self.readI2C(register, 1)
        newValue = self.updateByte(rawData[0], bit, numBits, value)
        self.writeI2C(register, newValue)

    def readBit(self, register, bit):
        return (self.readI2C(register, bit) & bit) != 0
    
    def writeBit(self, register, bit, value):
        """

        :type value: bool
        """
        buf = self.readI2C(register, 1)
        newValue = (buf[0] | (1 << bit)) if value else (buf[0] & ~(1 << bit))
        self.writeI2C(register, newValue)

    # this routine should update the original byte with the new data properly shifted to the correct bit location
    def updateByte(self, original, bit, numBits, value):
        if numBits > 7 or bit > 7 or bit < numBits-1 or bit < 0 or numBits < 0:
            raise ValueError("Use 8-bit bytes: numBits: {} bit: {}".format(numBits, bit))
        if value > 2**numBits:
            raise ValueError("Value too large")
        mask = self.getMask(bit, numBits)
        maskedOriginal = (original & mask) & 0xFF
        shiftedValue = (value << (1 + bit - numBits)) & 0xFF
        return (shiftedValue | maskedOriginal) & 0xFF

    def setFullScaleRange(self, range):
        self.writeBits(RA_DLPF_FS, DF_FS_SEL_BIT, DF_FS_SEL_LENGTH, range)
        
    def getFullScaleRange(self):
        return self.getRegisterBits(RA_DLPF_FS, DF_FS_SEL_BIT, DF_FS_SEL_LENGTH)

    def setClockSource(self, source):
        self.writeBits(RA_PWR_MGM, PWR_CLK_SEL_BIT, PWR_CLK_SEL_LENGTH, source)

    def setIntDeviceReadyEnabled(self, enabled):
        self.writeBit(RA_INT_CFG, INTCFG_ITG_RDY_EN_BIT, enabled)

    def setIntDataReadyEnabled(self, enabled):
        self.writeBit(RA_INT_CFG, INTCFG_RAW_RDY_EN_BIT, enabled)

    def getRateX(self):
        return self.readShortFromRegister(RA_GYRO_XOUT_H, 2)

    def getRateY(self):
        return self.readShortFromRegister(RA_GYRO_YOUT_H, 2)

    def getRateZ(self):
        return self.readShortFromRegister(RA_GYRO_ZOUT_H, 2)
        
    def getTemperature(self):
        return self.readShortFromRegister(RA_TEMP_OUT_H, 2)

    def testConnection(self):
        return self.getDeviceID() == 0b110100
    
    def getSampleRate(self):
        return self.getRegisterByte(RA_SMPLRT_DIV)
        
    def setSampleRate(self, rate):
        self.writeI2C(RA_SMPLRT_DIV, rate)

    # This register is used to verify the identity of the device
    def getDeviceID(self):
        return self.getRegisterBits(RA_WHO_AM_I, DEVID_BIT, DEVID_LENGTH)

    # Gets the bit mask for the given bit and number of bits
    def getMask(self, bit, numBits):
        newMask = 0
        for i in range(7+1):
            if i > bit or i <= bit - numBits:
                newMask += 2**i
        return newMask & 0xFF

    # Get n bits from the byte to form a byte slice
    def getBits(self, bitField, bit, numBits):
        if numBits > 7 or bit > 7 or bit < numBits - 1 or bit < 0 or numBits < 0:
            raise ValueError("Use 8-bit bytes")
        mask = ~self.getMask(bit, numBits) & 0xFF
        maskedInput = (bitField & mask) & 0xFF
        return rshift(maskedInput, (1 + bit - numBits)) & 0xFF

    def getRegisterByte(self, register):
        return self.readI2C(register, 1)[0]

    def getRegisterBits(self, register, bit, numBits):
        containingByte = self.getRegisterByte(register)
        return self.getBits(containingByte, bit, numBits)
    
    def getSleepEnabled(self):
        return self.readBit(RA_PWR_MGM, PWR_SLEEP_BIT)
    
    def setSleepEnabled(self, enabled):
        self.writeBit(RA_PWR_MGM, PWR_SLEEP_BIT, enabled)
        
    def setStandByXEnabled(self, enabled):
        self.writeBit(RA_PWR_MGM, PWR_STBY_XG_BIT, enabled)
    
    def setStandByYEnabled(self, enabled):
        self.writeBit(RA_PWR_MGM, PWR_STBY_YG_BIT, enabled)
    
    def setStandByZEnabled(self, enabled):
        self.writeBit(RA_PWR_MGM, PWR_STBY_ZG_BIT, enabled)
        
    def getStandByXEnabled(self):
        return self.readBit(RA_PWR_MGM, PWR_STBY_XG_BIT)
    
    def getStandByYEnabled(self):
        return self.readBit(RA_PWR_MGM, PWR_STBY_YG_BIT)
    
    def getStandByZEnabled(self):
        return self.readBit(RA_PWR_MGM, PWR_STBY_ZG_BIT)
    
