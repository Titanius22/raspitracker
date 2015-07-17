"""
TODO
-make function to tell servos to move 
-make option to switch to face tracking
-make the servo's shake when track is reset so the user knows to pull the object back



"""
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO

# wierd thing required for cv2 to work
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2

# Servo settings
#ServoLR = PWM(0x40)
#ServoLR.setPWMFreq(50)
#ServoLRmin = 1055 # microsecond 80 degress from center

#ServoLeftRight.setPWM(15, 1024, 3072) # channel, on, off

#ServoUD = PWM(0x40)
#ServoUD.setPWMFreq(50)
#ServoUpDown.setPWM(15, 1024, 3072) # channel, on, off

# How ofter will it check for 'reset'
resetCounter = 5

#Resolution
camWidth = 320
camHeight = 240

#Track box size
trackWidth = 60
trackHeight = 50


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
    
    # empty image buffer
    rawCapture.seek(0) 
    rawCapture.truncate()
    
    # Give time for object to back away
    time.sleep(2)
    
    # Tells user camera is active again
    GPIO.output(CAMLED,True) # Off
    
    # campure a frame of the video
    camera.capture(rawCapture, 'bgr', use_video_port=True)     
    frame = rawCapture.array

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
        
                preShift = track_window

                # Apply meanshift to get the new location
                ret, track_window = cv2.meanShift(dst, track_window, term_crit)
        
                # The change  X and Y. posative isup and to the right
                dx = track_window[0] - preShift[0]
                dy = track_window[1] - preShift[1]
                
                # Tells servos to move

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
                    test = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    frameMean = int((cv2.meanStdDev(test))[0])
                    frameStdDev = int((cv2.meanStdDev(test))[1])
                    if frameStdDev < 10:
                        track_window, roi_hist = Calibrate_NewObject()
                        resetCounter = 5

            else:
                print "Frame array was empty"
                break

        cv2.destroyAllWindows()
        camera.close()