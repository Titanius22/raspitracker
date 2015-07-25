# -*- coding: utf-8 -*-
import numpy as np
import time
import math
import RPi.GPIO as GPIO

# add paths to make some things work
import sys

sys.path.append('/home/pi/Adafruit-Raspberry-Pi-Python-Code,Adafruit_PWM_Servo_Driver')
from Adafruit_PWM_Servo_Driver import PWM

StepsPM = 3 # 11.27µs . Servo steps per move
ServoFreq = 65 # Hz. gives 3.756 microsecond per step resolution (4096 steps)
ServoTurnMargin = 4 # degrees or greater before servo moves.

ServoLR = PWM(0x40)
ServoLR.setPWMFreq(ServoFreq)
ServoLRpin = 12
ServoLRmiddle = 400 # step count. 1502µs
ServoLRmin = ServoLRmiddle - 117 # step count. 1063µs. 79° from middle
ServoLRmax =  ServoLRmiddle + 117 # step count. 1942µs. 79° from middle
ServoLRpos = ServoLRmiddle
ServoLR.setPWM(ServoLRpin, 1024, 3072) # channel, on, off

ServoUD = PWM(0x40)
ServoUD.setPWMFreq(ServoFreq)
ServoUDpin = 12
ServoUDmiddle = 400 # step count. 1502µs
ServoUDmin = ServoUDmiddle - 117 # step count. 1063µs. 79° from middle
ServoUDmax =  ServoUDmiddle + 117 # step count. 1942µs. 79° from middle
ServoUDpos = ServoUDmiddle
ServoUD.setPWM(ServoUDpin, 1024, 3072) # channel, on, off

step = ServeLRmin
while (step <=  ServoLRmax):
    ServoLR.setPWM(ServoLRpin, step, (4095 - step)) # channel, on, off
    step += StepsPM
    time.sleep(.1)
    
while (step >=  ServoLRmin):
    ServoLR.setPWM(ServoLRpin, step, (4095 - step)) # channel, on, off
    step -= StepsPM
    time.sleep(.1)