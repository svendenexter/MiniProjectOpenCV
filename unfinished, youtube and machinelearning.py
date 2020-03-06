# -*- coding: utf-8 -*-
"""
Created on Tue Mar  3 13:50:44 2020

@author: stefa
"""

import cv2
import numpy as np
import pafy
import threading
import time


global timeinterval
timeinterval = 1

url = "https://www.youtube.com/watch?v=nVutZktUxfY"

video=pafy.new(url)
best=video.getbest(preftype="mp4")

capture = cv2.VideoCapture()
capture.open(best.url)
starttime=time.time()


#video from webcam
#cap=cv2.VideoCapture(0)
cap = cv2.VideoCapture('video.mp4')
#car classifier
car_cascade=cv2.CascadeClassifier('cars.xml')


while (True) :
    # framerate 25 FPS (als code het bijhoud)
    time.sleep((1/25) - ((time.time() - starttime) % (1/25))) 
    capture = cv2.VideoCapture()
    capture.open(best.url)
    ret, frame = capture.read()
    frame = frame[300:, 700:1300]
#    cv2.imshow("cropped", crop_img)
    
    if (frame is not None):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        scaleFactor = 2
        minNeighbors = 0
        flags = None
        minSize = 0
        maxSize = 200
#        cv2.CascadeClassifier.detectMultiScale(gray, scaleFactor, minNeighbors, flags, minSize, maxSize) 
#        cv2.CascadeClassifier.detectMultiScale(gray, scaleFactor, minNeighbors,flags, minSize, maxSize) 
        cars=car_cascade.detectMultiScale(gray,2,1)
    
    for(x,y,w,h) in cars:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
        font=cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame,'car',(x+6,y-6),font,0.5,(0,0,255),1)
        
#        r=800/frame.shape[1]
#        dim = (800,int (frame.shape[0]*r))
#        resized = cv2.resize(frame,dim, interpolation = cv2.INTER_AREA)    
#        cv2.imshow('output',resized)
        
        cv2.imshow('Car Detection',frame)
    else:
        
        print ("geen frame")
        
    if (cv2.waitKey(1)== ord ('q')):
        break
    
cap.release()
cv2.destroyAllWindows()
