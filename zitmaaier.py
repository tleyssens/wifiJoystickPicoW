from machine import Pin, I2C, PWM
from time import sleep
from time import ticks_ms
from PID import PID
from math import fmod

# Minimum Potentiometer Input Value (determine through experimentation)
pot_min = 0
# Maximum Potentiometer Input Value (determine through experimentation)
pot_max = 4096

Kp = 7;  # Proportional Gain
Ki = 7;  # Integral Gain
Kd = 0.05;  # Derivitive Gain
interval = 20

def convert(x, i_m, i_M, o_m, o_M):
    return max(min(o_M, (x - i_m) * (o_M - o_m) / (i_M - i_m) + o_m), o_m)

class Zitmaaier():
    def __init__(self, motorStuur, motorVoorAchter, AS5600Stuur, AS5600VoorAchter):
        self.motorStuur = motorStuur
        self.motorVoorAchter = motorVoorAchter
        self.AS5600Stuur = AS5600Stuur
        self.AS5600VoorAchter = AS5600VoorAchter

        #Initialize PID Controllers
        self.motorStuur.myPID = PID(Kp, Ki, Kd, setpoint=1, scale='us')
        self.motorStuur.myPID.auto_mode = True
        self.motorVoorAchter.myPID = PID(Kp, Ki, Kd, setpoint=1, scale='us')
        self.motorVoorAchter.myPID.auto_mode = True
        self.previousMillis = 0

                 
    def setMinMaxStuur(self, minStuur, maxStuur):
        self.motorStuur.minStuur = minStuur
        self.motorStuur.maxStuur = maxStuur

    def setMinMaxVoorAchter(self, minVoorAchter, maxVoorAchter):
        self.motorVoorAchter.minVoorAchter = minVoorAchter
        self.motorVoorAchter.maxVoorAchter = maxVoorAchter

    def setMaxPwmStuur(self, pwmMax):
        # Max snelheid stuurmotor
        self.motorStuur.myPID.output_limits = [-pwmMax, pwmMax]
        self.motorStuur.myPID.sample_time = 20

    def setMaxPwmVoorAchter(self, pwmMax):
        # Max snelheid voorachtermotor
        self.motorVoorAchter.myPID.output_limits = [-pwmMax, pwmMax]
        self.motorVoorAchter.myPID.sample_time = 20

    def process(self):
        # Reset value of currentMillis
        self.currentMillis = ticks_ms()
        #See if interval period has expired, if it has then reset value
        if (self.currentMillis - self.previousMillis >= interval):
            self.previousMillis = self.currentMillis

            # Meet de hoek van de wielen met de magsensor
            input_val_stuur = self.AS5600Stuur.getAngle()
            # Meet de hoek van de voorAchterPedaal met de magsensor
            input_val_voorAchter = self.AS5600VoorAchter.getAngle()

            # Establish Input value for PID
            inputStuurPid = convert(input_val_stuur , pot_min, pot_max, -180, 180)
            inputVoorAchterPid = convert(input_val_voorAchter, pot_min, pot_max, -180, 180)

            # Compute new output from the PID according to the systems current value
            outputStuurPid = self.motorStuur.myPID(inputStuurPid)
            outputVoorAchterPid = self.motorVoorAchter.myPID(inputVoorAchterPid)
            
            #Stuur
            if (outputStuurPid > 0):
                # Need to move motor forward
                pwm_val = int(outputStuurPid)
                self.motorStuur.DIR = 0
            elif (outputStuurPid < 0):
                pwm_val = int(abs(outputStuurPid))
                self.motorStuur.DIR = 1
            self.motorStuur.PWM = pwm_val
            
            #VoorAchter
            if (outputVoorAchterPid > 0):
                # Need to move motor forward
                pwm_val = int(outputVoorAchterPid)
                self.motorVoorAchter.DIR = 0
            elif (outputVoorAchterPid < 0):
                pwm_val = int(abs(outputVoorAchterPid))
                self.motorVoorAchter.DIR = 1
            self.motorVoorAchter.PWM = pwm_val
            
            restje = fmod(self.currentMillis, 500)
            if restje < 300 or restje > 320:
                pass
            else :
                print("gewenste stuurpositie => ", self.motorStuur.myPID.setpoint, " | ", inputStuurPid, " <= gemeten positie" ," | ", "dir / pwm => ", self.motorStuur.DIR, " | ", self.motorStuur.PWM)
                print("gewenste voorachterpositie => ", self.motorVoorAchter.myPID.setpoint, " | ", inputVoorAchterPid, " <= gemeten positie" ," | ","dir / pwm", self.motorVoorAchter.DIR, " | ", self.motorVoorAchter.PWM)

    def rijden(self, stuur, voorachter, reserve1, reserve2):
        # print("in rijden : stuur = ", stuur, " | richting = ", richting)
        stuur = convert(stuur, -100, 100, self.motorStuur.minStuur, self.motorStuur.maxStuur)
        self.motorStuur.myPID.setpoint = stuur

        voorachter = convert(voorachter, -100, 100, self.motorVoorAchter.minStuur, self.motorVoorAchter.maxStuur)
        self.motorVoorAchter.myPID.setpoint = voorachter