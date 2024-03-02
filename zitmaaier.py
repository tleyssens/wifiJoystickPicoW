from machine import Pin, I2C, PWM
from time import sleep
from time import ticks_ms
from PID import PID

# AS5600 specific
AS5600_SCL = const(17)
AS5600_SDA = const(16)
AS5600_ADDRESS = const(0x36)   # AS5600 has a fixed address (so can only use one per I2C bus?)
AS5600_RAW_ANGLE = const(0x0C)
ANGLE_H	= const(0x0E)          # Angle register (high byte)
ANGLE_L	= const(0x0F)          # Angle register (low byte)
AS5600_MAGNET_HIGH   = const(0x08)
AS5600_MAGNET_LOW    = const(0x10)
AS5600_MAGNET_DETECT = const(0x20)
AS5600_STATUS    = const(0x0B)

# I2C bus init for AS5600
i2c = I2C(0, scl=Pin(AS5600_SCL), sda=Pin(AS5600_SDA))

# Minimum Potentiometer Input Value (determine through experimentation)
pot_min = 0
# Maximum Potentiometer Input Value (determine through experimentation)
pot_max = 360

Kp = 7;  # Proportional Gain
Ki = 7;  # Integral Gain
Kd = 0.05;  # Derivitive Gain
interval = 20
neutraalTijd = 1000

def convert(x, i_m, i_M, o_m, o_M):
    return max(min(o_M, (x - i_m) * (o_M - o_m) // (i_M - i_m) + o_m), o_m)

def getnReg(reg, n):
    i2c.writeto(AS5600_ADDRESS, bytearray([reg]))
    t =	i2c.readfrom(AS5600_ADDRESS, n)
    return t

def getAngle360():
    buf = getnReg(ANGLE_H, 2)
    return ((buf[0]<<8) | buf[1])/ 4096.0*360


class Zitmaaier():
    def __init__(self, minStuur, maxStuur, pwmMaxStuur):
        self.minStuur = minStuur
        self.maxStuur = maxStuur
        self.previousMillis = 0
        self.stuurPWM = PWM(Pin(14, Pin.OUT), freq=500, duty_u16=0)
        self.stuurDIR = Pin(15, Pin.OUT)

        self.neutraal = Pin(9, Pin.IN,)
        self.vooruit = Pin(10, Pin.OUT)
        self.achteruit = Pin(11, Pin.OUT, value=0)
        self.naarNeutraalTijd = 0
        self.previousRichting = 0
        self.stop = False
        #Initialize PID Controller
        self.myPID = PID(Kp, Ki, Kd, setpoint=1, scale='us')
        self.myPID.auto_mode = True
        # Max snelheid stuurmotor
        self.myPID.output_limits = [-pwmMaxStuur, pwmMaxStuur]
        self.myPID.sample_time = 20

    def process(self):
        # global previousMillis
        # Reset value of currentMillis
        self.currentMillis = ticks_ms()
        #See if interval period has expired, if it has then reset value
        if (self.currentMillis - self.previousMillis >= interval):
            self.previousMillis = self.currentMillis

            # Meet de hoek van de wielen met de magsensor
            input_val = getAngle360()

            # Establish Input value for PID
            input = convert(input_val , pot_min, pot_max, -180, 180)

            # Compute new output from the PID according to the systems current value
            output = self.myPID(input)

            if (output > 0):
                # Need to move motor forward
                pwm_val = int(output)
                self.stuurDIR.value(0)
            elif (output < 0):
                pwm_val = int(abs(output))
                self.stuurDIR.value(1)
            self.stuurPWM.duty_u16(pwm_val)

            # print("gewenste positie => ", self.myPID.setpoint, " | ", input, " <= gemeten positie" ," | ", "stuurdir / pwm => ", self.stuurDIR.value(), " | ", self.stuurPWM.duty_u16())
            print("Vooruit / Achteruit ", self.vooruit.value(), " | ", self.achteruit.value(), " | tijdStopCommando / tijd", self.naarNeutraalTijd , " | ", self.currentMillis)
            if self.naarNeutraalTijd + neutraalTijd <= self.currentMillis and self.stop == True:
                print("tijd ", self.naarNeutraalTijd + neutraalTijd)
                self.vooruit.value(0)
                self.achteruit.value(0)
                self.stop = False

    def rijden(self, stuur, richting, reserve1, reserve2):
        # print("in rijden : stuur = ", stuur, " | richting = ", richting)
        stuur = convert(stuur, -100, 100, self.minStuur, self.maxStuur)
        self.myPID.setpoint = stuur
        
        if richting < -20:
            self.stop = False
            self.naarNeutraalTijd = 0
            # Vooruit
            self.achteruit.value(0)
            self.vooruit.value(1)
        elif richting > 20:
            self.stop = False
            self.naarNeutraalTijd = 0
            # Achteruit
            self.vooruit.value(0)
            self.achteruit.value(1)
        else:
            if self.stop == True:
                return
            if self.previousRichting < -20:
                # terugdraaien tot middenpositie
                self.naarNeutraalTijd = ticks_ms()
                self.vooruit.value(0)
                self.achteruit.value(0)
                sleep(0.1)
                self.achteruit.value(1)
            if self.previousRichting > 20:
                # terugdraaien tot middenpositie
                self.naarNeutraalTijd = ticks_ms()
                self.vooruit.value(0)
                self.achteruit.value(0)
                sleep(0.1)
                self.vooruit.value(1)
            self.stop = True

        self.previousRichting = richting

