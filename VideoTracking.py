# -*- coding: utf-8 -*-
"""
TODO
-make function to tell servos to move 
-make option to switch to face tracking
-make the servo's shake when track is reset so the user knows to pull the object back
-find numerical center of servos and calibrate 'center' variable
-add tolerance for servo moving. 4° or greater
-add checks for sero max and min limits
-Check servooUD setting

"""
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import RPi.GPIO as GPIO

# add paths to make some things work
import sys

sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2

sys.path.append('/home/pi/Adafruit-Raspberry-Pi-Python-Code/Adafruit_PWM_Servo_Driver')
from Adafruit_PWM_Servo_Driver import PWM

########################################################################################
                    # Servo settings

#ServoDeadBand = 10 # microseconds
StepsPM = 3 # 11.27µs . Servo steps per move
ServoFreq = 65 # Hz. gives 3.756 microsecond per step resolution (4096 steps)
ServoTurnMargin = 4 # degrees or greater before servo moves.

ServoLR = PWM(0x40)
ServoLR.setPWMFreq(ServoFreq)
ServoLRpin = 12
ServoLRmiddle = 496 # step count. 1502µs
ServoLRmin = ServoLRmiddle - 237 # step count. 1063µs. 79° from middle
ServoLRmax =  ServoLRmiddle + 237 # step count. 1942µs. 79° from middle
ServoLRpos = ServoLRmiddle
#ServoLR.setPWM(ServoLRpin, 0, ServoLRmiddle) # channel, time it turns on, time it turns off

ServoUD = PWM(0x40)
ServoUD.setPWMFreq(ServoFreq)
ServoUDpin = 13
ServoUDmiddle = 496 # step count. 1502µs
ServoUDmin = ServoUDmiddle - 237 # step count. 1063µs. 79° from middle
ServoUDmax =  ServoUDmiddle + 237 # step count. 1942µs. 79° from middle
ServoUDpos = ServoUDmiddle
#ServoUD.setPWM(ServoUDpin, 0, ServoUDmiddle) # channel, time it turns on, time it turns off

# I am writing this to explain my math and reasoning.
#
# With a resolution of 4096 steps and a chosen frequency of 65 Hz,
# the pulse length of each step is 3.756 microseconds. So...
#     Deadband is 10µs so each move will be 3 steps or 11.27µs
#     Covers 79° either direction within 117 steps
#     39 moves one direction (117 steps / 3 steps per mover) makes 2.03° per move
#     Middle is 1500 microseconds or 400 steps (1502µs)
#     One extreme is 280 steps (1063µs)
#     The other is 517 steps (1942µs)
#
# ASSUME LEFT IS MIN
# ASSUME DOWN IS MIN

# EXPERIMENTALLY TESTED RESULTS
#     Declared 259 as min, 733 as max
#   
#
#

########################################################################################
#                                    Fun Math
#       ___x____
#       \     /
#     y \   /
#       \θ/


def calc_width_theta(pixelWidth):
    y = (0.5 * camWidth)/math.sin(math.radians(26.75)) # The 26.75 is half the width angle of the field of view of the camera
    theta = math.degrees(math.asin(pixelWidth/y))
    return theta
    
def calc_height_theta(pixelHeight):
    y = (0.5 * camHeight)/math.sin(math.radians(20.7)) # The 20.7 is half the width angle of the field of view of the camera
    theta = math.degrees(math.asin(pixelHeight/y))
    return theta
    
def round_to_even(x):
    return round(x/2.0)*2

########################################################################################


# How ofter will it check for 'reset'
resetCounter = 5

# Resolution (native 2592 x 1944)
# Field of View 53.5 x 41.4
camWidth = 324
camHeight = 243

#Track box size
trackWidth = 50
trackHeight = 40


# Hue detction range
hueMin = 0
hueMax = 180
lower_hue = np.array([hueMin, 50, 50], dtype=np.uint8)
upper_hue = np.array([hueMax,255,255], dtype=np.uint8)

"""
HUE VALUES
Red = 0-15
Orangle 15-25
Yellow 25-35
Green 35-80
Cyan 85-95
Blue 100-135
Purple 140-155
Red 160-180
"""

# Use GPIO numbering
GPIO.setmode(GPIO.BCM)
 
# Set GPIO for camera LED
# Use 5 for Model A/B and 32 for Model B+
CAMLED = 32
 
# Set GPIO to output
GPIO.setup(CAMLED, GPIO.OUT, initial=False) 
 

def Calibrate_NewObject():
    
    # Tells user reset was set and to pull object away
    GPIO.output(CAMLED,False) # Off
    
    # Give time for object to back away
    time.sleep(2)
    
    # Tells user that camera is active again and trying to find a good frame
    GPIO.output(CAMLED,True) # On
    time.sleep(.5)
    GPIO.output(CAMLED,False) # Off
    
    while(True):
        # empty image buffer
        rawCapture.seek(0) 
        rawCapture.truncate()
    
        # Capture a frame of the video
        camera.capture(rawCapture, 'bgr', use_video_port=True)     
        frame = rawCapture.array
        
        if (Get_Frame_StdDev(frame) > 30):
            break
    
    # Tells user that camera is active found a good frame        
    GPIO.output(CAMLED,True) # On

    # setup initial location of window
    r,h,c,w = int(.5*(camHeight-trackHeight)), trackHeight, int(.5*(camWidth-trackWidth)), trackWidth  # Defined above. used int() to keep type integer
    track_window = (c,r,w,h)

    # set up the ROI for tracking
    roi = frame[r:r+h, c:c+w] #creat smaller frame
    hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) #convert to HSV

    mask = cv2.inRange(hsv_roi, lower_hue, upper_hue) #np.array((0., 60.,32.)), np.array((180.,255.,255.))
    roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180]) #([hsv_roi],[0],mask,[180],[0,180])
    cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)
    
    return track_window, roi_hist


def Get_Frame_StdDev(chosenFrame):
    test = cv2.cvtColor(chosenFrame, cv2.COLOR_BGR2GRAY)
    return int((cv2.meanStdDev(test))[1])

# Tells user camera is active again
GPIO.output(CAMLED,True) # Turns on light

# Sets servos to all direction center
ServoLR.setPWM(ServoLRpin, 0, ServoLRmiddle) # channel, on, off
ServoUD.setPWM(ServoUDpin, 0, ServoUDmiddle) # channel, on, off

# initialize the camera and grab a referance to the raw camera capture
with PiCamera() as camera:
    with PiRGBArray(camera) as rawCapture:
        camera.resolution = (camWidth, camHeight)

        # take first frame of the video
        camera.capture(rawCapture, 'bgr', use_video_port=True)     
        frame = rawCapture.array

        # setup initial location of window
        r,h,c,w = int(.5*(camHeight-trackHeight)), trackHeight, int(.5*(camWidth-trackWidth)), trackWidth  # Defined above. used int() to keep type integer
        track_window = (c,r,w,h)

        # set up the ROI for tracking
        roi = frame[r:r+h, c:c+w] #create smaller frame
        hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) #convert to HSV

        mask = cv2.inRange(hsv_roi, lower_hue, upper_hue) #np.array((0., 60.,32.)), np.array((180.,255.,255.))
        roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180]) #([hsv_roi],[0],mask,[180],[0,180])
        cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(frame,frame, mask= mask)

        # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

        while True:
    
            # empty image buffer
            rawCapture.seek(0) 
            rawCapture.truncate()
            
            # capture and retrieve frame
            camera.capture(rawCapture, 'bgr', use_video_port=True)
            frame = rawCapture.array

            if frame.size: #if array has numbers then its true
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

                # Apply meanshift to get the new location
                ret, track_window = cv2.meanShift(dst, track_window, term_crit)
        
                # Calculates dx and dy in pixels of box with respect to original bottom left corner of box. Positive is up and to the right
                dx = track_window[0] - c # calculate dx in pixels of box with respect to center at beginning
                dy = track_window[1] - r # calculate dy in pixels of box with respect to center at beginning
                
                # Calculates dθx and dθy then rounds to nearest even number
                dThetaX = round_to_even(calc_width_theta(dx))
                dThetaY = round_to_even(calc_height_theta(dy))
                
                # Calculates steps and new location of servos. 3 (StepsPM) steps per 2°
                xSteps = (dThetaX/2)*StepsPM
                ySteps = (dThetaY/2)*StepsPM
                
                # Tells servos to move
                if (xSteps >= ServoTurnMargin):
                    ServoLRpos = ServoLRpos + xSteps
                    if (ServoLRpos < ServoLRmin): # Assumes left is min
                        ServoLRpos = ServoLRmin
                    if (ServoLRpos > ServoLRmax):
                        ServoLRpos = ServoLRmax
                    #elif ((ServoLRpos == ServoLRmax) or (ServoLRpos == ServoLRmax)):
                        # If they equal either limit
                    ServoLR.setPWM(ServoLRpin, ServoLRpos, (4095 - ServoLRpos)) # channel, on, off
                    
                if (ySteps >= ServoTurnMargin):
                    ServoUDpos = ServoUDpos + ySteps
                    if (ServoUDpos < ServoUDmin): # Assumes left is min
                        ServoUDpos = ServoUDmin
                    if (ServoUDpos > ServoUDmax):
                        ServoUDpos = ServoUDmax
                    #elif ((ServoUDpos == ServoUDmax) or (ServoUDpos == ServoUDmax)):
                        # If they equal either limit
                    ServoLR.setPWM(ServoUDpin, ServoUDpos, (4095 - ServoUDpos)) # channel, on, off

                # Draw it on image
                x,y,w,h = track_window
                cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        
                cv2.imshow('Tracking',frame)

                k = cv2.waitKey(60) & 0xff
                if k == 27:
                    print "Escape key was pressed"
                    break
                    
                # Used as a 'reset' button to start tracking new object TODO
                resetCounter = resetCounter -1
                if resetCounter == 0:
                    test = cv2.cvtColor(frame[x:x+w,y:y+h], cv2.COLOR_BGR2GRAY)
                    #frameMean = int((cv2.meanStdDev(test))[0])
                    frameStdDev = Get_Frame_StdDev(frame)
                    if frameStdDev < 10:
                        track_window, roi_hist = Calibrate_NewObject()
                    resetCounter = 5

            else:
                print "Frame array was empty"
                break

        cv2.destroyAllWindows()
        camera.close()