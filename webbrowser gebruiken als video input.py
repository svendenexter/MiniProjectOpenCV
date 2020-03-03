# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 16:29:34 2020

@author: stefa
"""

import numpy as np
import cv2

import pafy

import threading 

tijdinterval = 1

def getframe(): 
    print("get frame aangeroepen") 
    capture = cv2.VideoCapture()
    capture.open(best.url)
    ret, frame = capture.read()
    return (frame)

def timestamp():
    global tijdinterval
    tijdinterval = 1

#url = "https://youtu.be/aUAVLhfVE5U"
url = "https://www.youtube.com/watch?v=nVutZktUxfY"

video = pafy.new(url)
best = video.getbest(preftype = "mp4")

capture = cv2.VideoCapture()
capture.open(best.url)

while(True):
    timer = threading.Timer(0.01,timestamp) 
    timer.start() 
    
    if (tijdinterval == 1):
        frame=getframe()       
    r=800/frame.shape[1]
    dim = (800,int (frame.shape[0]*r))
    resized = cv2.resize(frame,dim, interpolation = cv2.INTER_AREA)    
    
    cv2.imshow('output',resized)
    if (cv2.waitKey(1)== ord ('q')):
        break

capture.release()
cv2.destroyAllWindows()
timer.cancel()
