import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
# from matplotlib import pyplot as plt

# wierd thing required for cv2 to work
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2

#Resolution
camWidth = 320
camHeight = 240

#Track box size
trackWidth = 60
trackHeight = 50


# Hue detction range
hueMin = 0
hueMax = 15
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

# initialize the camera and grab a referance to the raw camera capture
with PiCamera() as camera:
    with PiRGBArray(camera) as rawCapture:
        camera.resolution = (camWidth, camHeight)

        # take first frame of the video
        camera.capture(rawCapture, 'bgr', use_video_port=True)     
        frame = rawCapture.array

        # setup initial location of window
        r,h,c,w = (.5*(camHeight-trackHeight)), trackHeight, (.5*(camWidth-trackWidth)), trackWidth  # Defined above
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
            #camera.capture(rawCapture, use_video_port=True, format="bgr")
            frame = rawCapture.array

            if frame.size: #if array has number then its true
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
        
                #preShift = track_window

                # Apply meanshift to get the new location
                ret, track_window = cv2.meanShift(dst, track_window, term_crit)
                if ret != True:
                    print "ret is not True"
                    break
        
                # The change  X and Y 
                #deltaPos = cv2.subtract((track_window[0],track_window[1]), (preShift[0], preShift[1]))

                # Draw it on image
                x,y,w,h = track_window
                cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        
                cv2.imshow('Tracking',frame)

                k = cv2.waitKey(60) & 0xff
                if k == 27:
                    print "Escape key was pressed"
                    break
                else:
                    cv2.imwrite(chr(k)+".jpg",img2)

            else:
                print "Frame array was empty"
                break

        cv2.destroyAllWindows()
        camera.close()