# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 12:08:53 2020

@author: stefa
"""

#import cv2
#
#
#cap=cv2.VideoCapture(0)
#while (True):
#    
#    ret, frame = cap.read()
#    cv2.imshow('Car Detection',frame)
#    if (cv2.waitKey(1)== ord ('q')):
#        break
#cap.release()
#cv2.destroyAllWindows()


import numpy as np
import cv2
cap = cv2.VideoCapture('road.mp4')
#cap = cv2.VideoCapture('road.mp4')



fgbg = cv2.createBackgroundSubtractorMOG2()
while(1):
    ret, frame = cap.read()
    frame = frame[300:, 700:1300]
    if (ret == True):        
        fgmask = fgbg.apply(frame)
        cv2.imshow('frame',fgmask)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

cap.release()
cv2.destroyAllWindows()
