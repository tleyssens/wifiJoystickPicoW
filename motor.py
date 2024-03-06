from machine import Pin, I2C, PWM
from time import sleep
from time import ticks_ms
from PID import PID
from math import fmod

class Motor():
    def __init__(self, PWMpin, DIRpin):
        self.PWMpin = PWM(Pin(PWMpin, Pin.OUT), freq=50, duty_u16=0)
        self.DIRpin = Pin(DIRpin, Pin.OUT)
        self.previousMillis = 0

    @property
    def PWM(self):
        return self.PWMpin.duty_u16()

    @PWM.setter
    def PWM(self, value):
        self.PWMpin.duty_u16(value)

    @property
    def DIR(self):
        return self.DIRpin.value()

    @DIR.setter
    def DIR(self, value):
        self.DIRpin.value(value)