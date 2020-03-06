# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 16:29:34 2020

@author: stefa
"""

import numpy as np
import cv2
import pafy
import threading 
import time

global tijdinterval
tijdinterval = 1


#def timestamp():
#    global tijdinterval
#    tijdinterval = 1
#timer = threading.Timer(0.02,timestamp) 
#timer.start() 

#url = "https://youtu.be/aUAVLhfVE5U"
url = "https://www.youtube.com/watch?v=nVutZktUxfY"


video = pafy.new(url)
best = video.getbest(preftype = "mp4")

capture = cv2.VideoCapture()
capture.open(best.url)
starttime=time.time() 
while(True):    

    capture = cv2.VideoCapture()
    capture.open(best.url)
    ret, frame = capture.read()
    
    if (frame is not None):
        r=800/frame.shape[1]
        dim = (800,int (frame.shape[0]*r))
        resized = cv2.resize(frame,dim, interpolation = cv2.INTER_AREA)    
        cv2.imshow('output',resized)
    else:
        
        print ("geen frame")
        
    if (cv2.waitKey(1)== ord ('q')):
        break
    time.sleep((1/25) - ((time.time() - starttime) % (1/25)))

capture.release()
cv2.destroyAllWindows()
#timer.cancel()
