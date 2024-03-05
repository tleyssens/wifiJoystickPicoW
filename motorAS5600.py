from machine import Pin, I2C, PWM
from time import sleep
from time import ticks_ms
from PID import PID
from math import fmod

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

# I2C bus init for AS5600
# i2c = I2C(0, scl=Pin(AS5600_SCL), sda=Pin(AS5600_SDA))

# Minimum Potentiometer Input Value (determine through experimentation)
pot_min = 0
# Maximum Potentiometer Input Value (determine through experimentation)
pot_max = 4096

Kp = 10000  # Proportional Gain
Ki = 7000  # Integral Gain
Kd = 5  # Derivitive Gain
interval = 20
neutraalTijd = 1000

def convert(x, i_m, i_M, o_m, o_M):
    return max(min(o_M, (x - i_m) * (o_M - o_m) / (i_M - i_m) + o_m), o_m)

# def getnReg(reg, n):
#     i2c.writeto(AS5600_ADDRESS, bytearray([reg]))
#     t =	i2c.readfrom(AS5600_ADDRESS, n)
#     return t

def getAngle360():
    buf = getnReg(ANGLE_H, 2)
    return ((buf[0]<<8) | buf[1])/ 4096.0*360

class MotorAS5600():
    def __init__(self, I2C_Adress, AS5600_SCL, AS5600_SDA, PWMpin, DIRpin):
        # I2C bus init for AS5600
        self.i2c = I2C(I2C_Adress, scl=Pin(AS5600_SCL), sda=Pin(AS5600_SDA))

        self.PWMpin = PWM(Pin(PWMpin, Pin.OUT), freq=50, duty_u16=0)
        self.DIRpin = Pin(DIRpin, Pin.OUT)

    def getnReg(self, reg, n):
        self.i2c.writeto(AS5600_ADDRESS, bytearray([reg]))
        t =	self.i2c.readfrom(AS5600_ADDRESS, n)
        return t
    
    def getAngle(self):
        buf = self.getnReg(ANGLE_H, 2)
        return ((buf[0]<<8) | buf[1])
    
    def setup(self, minAngle, maxAngle, pwmOutMax):
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.previousMillis = 0
        
        #Initialize PID Controller
        self.myPID = PID(Kp, Ki, Kd, setpoint=1, scale='us')
        self.myPID.auto_mode = True
        # Max snelheid stuurmotor
        self.myPID.output_limits = [-pwmOutMax, pwmOutMax]
        self.myPID.sample_time = 20

    def process(self):
        # global previousMillis
        # Reset value of currentMillis
        self.currentMillis = ticks_ms()
        #See if interval period has expired, if it has then reset value
        if (self.currentMillis - self.previousMillis >= interval):
            self.previousMillis = self.currentMillis

            # Meet de hoek van de wielen met de magsensor
            input_val = self.getAngle()

            # Establish Input value for PID
            input_pid = convert(input_val , pot_min, pot_max, -180, 180)

            # Compute new output from the PID according to the systems current value
            output_pid = self.myPID(input_pid)

            if (output_pid > 0):
                # Need to move motor forward
                pwm_val = int(output_pid)
                self.DIRpin.value(0)
            elif (output < 0):
                pwm_val = int(abs(output_pid))
                self.DIRpin.value(1)
            self.PWMpin.duty_u16(pwm_val)

            # print("gewenste positie => ", self.myPID.setpoint, " | ", input, " <= gemeten positie" ," | ", "stuurdir / pwm => ", self.stuurDIR.value(), " | ", self.stuurPWM.duty_u16())

    def rijden(self, stuur, richting, reserve1, reserve2):
        # print("in rijden : stuur = ", stuur, " | richting = ", richting)
        stuur = convert(stuur, -100, 100, self.minStuur, self.maxStuur)
        self.myPID.setpoint = stuur

