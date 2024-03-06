from machine import Pin, I2C

# AS5600 specific
# AS5600_SCL = const(17)
# AS5600_SDA = const(16)
AS5600_ADDRESS = const(0x36)   # AS5600 has a fixed address (so can only use one per I2C bus?)
#AS5600_RAW_ANGLE = const(0x0C)
ANGLE_H	= const(0x0E)          # Angle register (high byte)
ANGLE_L	= const(0x0F)          # Angle register (low byte)
#AS5600_MAGNET_HIGH   = const(0x08)
#AS5600_MAGNET_LOW    = const(0x10)
#AS5600_MAGNET_DETECT = const(0x20)
#AS5600_STATUS    = const(0x0B)

class AS5600():
    def __init__(self, I2C_Adress, AS5600_SDA, AS5600_SCL):
        # I2C bus init for AS5600
        self.i2c = I2C(I2C_Adress, scl=Pin(AS5600_SCL), sda=Pin(AS5600_SDA))
    
    def setup(self, minAngle, maxAngle):
        self.minAngle = minAngle
        self.maxAngle = maxAngle
    
    def getnReg(self, reg, n):
        self.i2c.writeto(AS5600_ADDRESS, bytearray([reg]))
        t =	self.i2c.readfrom(AS5600_ADDRESS, n)
        return t
    
    def getAngle(self):
        buf = self.getnReg(ANGLE_H, 2)
        return ((buf[0]<<8) | buf[1])
