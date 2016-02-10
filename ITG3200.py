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
import wpilib.interfaces

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
RA_WHO_AM_I = 0x00
RA_DLPF_FS = 0x16
RA_PWR_MGM = 0x3E
RA_INT_CFG = 0x17

DF_FS_SEL_BIT = 4
DF_FS_SEL_LENGTH = 2

INTCFG_ITG_RDY_EN_BIT = 2
INTCFG_RAW_RDY_EN_BIT = 0

PWR_CLK_SEL_BIT = 2
PWR_CLK_SEL_LENGTH = 3

DEVID_BIT = 6
DEVID_LENGTH = 6

FULLSCALE_2000 = 0x03

CLOCK_PLL_XGYRO = 0x01


def rshift(val, n):
    return (val % 0x100000000) >> n


class ITG3200(wpilib.interfaces.PIDSource):
    def __init__(self, port, addr=DEFAULT_ADDR):
        self.i2c = wpilib.I2C(port, addr)
        self.addr = addr
        self.pidAxis = self.Axis.X
        self.setPIDSourceType(wpilib.interfaces.PIDSource.PIDSourceType.kRate)
        self.reset()

    def reset(self):
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0

    def update(self, dt=20):
        """

        :param dt: Delta time in milliseconds
        :return:
        """
        real_dt = 1000/dt

        self.angleX += self.getRotationX() * real_dt
        self.angleY += self.getRotationY() * real_dt
        self.angleZ += self.getRotationZ() * real_dt

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
        if self.getPIDSourceType() == wpilib.interfaces.PIDSource.PIDSourceType.kRate:
            if self.pidAxis == self.Axis.X:
                return self.getRotationX()
            if self.pidAxis == self.Axis.Y:
                return self.getRotationY()
            if self.pidAxis == self.Axis.Z:
                return self.getRotationZ()
        else:
            if self.pidAxis == self.Axis.X:
                return self.angleX
            if self.pidAxis == self.Axis.Y:
                return self.angleY
            if self.pidAxis == self.Axis.Z:
                return self.angleZ

    def init(self):
        if not self.testConnection():
            wpilib.DriverStation.reportError("Could not connect to ITG3200", False)
        self.setFullScaleRange(FULLSCALE_2000)
        self.setClockSource(CLOCK_PLL_XGYRO)
        self.setIntDeviceReadyEnabled(True)
        self.setIntDataReadyEnabled(True)

    def readShortFromRegister(self, register, count):
        buf = self.i2c.read(register, count)
        return (buf[0] << 8) | buf[1]

    def writeBits(self, register, bit, numBits, value):
        rawData = self.i2c.read(register, 1)
        newValue = self.updateByte(rawData[0], bit, numBits, value)
        self.i2c.write(register, newValue)

    def writeBit(self, register, bit, value):
        """

        :type value: bool
        """
        buf = self.i2c.read(register, 1)
        newValue = (buf[0] | (1 << bit)) if value else (buf[0] & ~(1 << bit))
        self.i2c.write(register, newValue)

    def setFullScaleRange(self, range):
        self.writeBits(RA_DLPF_FS, DF_FS_SEL_BIT, DF_FS_SEL_LENGTH, range)

    def setClockSource(self, source):
        self.writeBits(RA_PWR_MGM, PWR_CLK_SEL_BIT, PWR_CLK_SEL_LENGTH, source)

    def setIntDeviceReadyEnabled(self, enabled):
        self.writeBit(RA_INT_CFG, INTCFG_ITG_RDY_EN_BIT, enabled)

    def setIntDataReadyEnabled(self, enabled):
        self.writeBit(RA_INT_CFG, INTCFG_RAW_RDY_EN_BIT, enabled)

    def getRotationX(self):
        return self.readShortFromRegister(RA_GYRO_XOUT_H, 2)

    def getRotationY(self):
        return self.readShortFromRegister(RA_GYRO_YOUT_H, 2)

    def getRotationZ(self):
        return self.readShortFromRegister(RA_GYRO_ZOUT_H, 2)

    def testConnection(self):
        return self.getDeviceID() == 0b110100

    def getDeviceID(self):
        return self.getRegisterBits(RA_WHO_AM_I, DEVID_BIT, DEVID_LENGTH)

    def getMask(self, bit, numBits):
        newMask = 0
        for i in range(7+1):
            if i > bit or i <= bit - numBits:
                newMask += 2**i
        return newMask & 0xFF

    def getBits(self, bitField, bit, numBits):
        if numBits > 7 or bit > 7 or bit < numBits - 1 or bit < 0 or numBits < 0:
            raise ValueError("Use 8-bit bytes")
        mask = ~self.getMask(bit, numBits) & 0xFF
        maskedInput = (bitField & mask) & 0xFF
        return rshift(maskedInput, (1 + bit - numBits)) & 0xFF

    def getRegisterByte(self, register):
        return self.i2c.read(register, 1)[0]

    def getRegisterBits(self, register, bit, numBits):
        containingByte = self.getRegisterByte(register)
        return self.getBits(containingByte, bit, numBits)

    def updateByte(self, original, bit, numBits, value):
        if numBits > 7 or bit > 7 or bit < numBits-1 or bit < 0 or numBits < 7:
            raise ValueError("Use 8-bit bytes")
        if value > 2**numBits:
            raise ValueError("Value too large")
        mask = self.getMask(bit, numBits)
        maskedOriginal = (original & mask) & 0xFF
        shiftedValue = (value << (1 + bit - numBits)) & 0xFF
        return (shiftedValue | maskedOriginal) & 0xFF
