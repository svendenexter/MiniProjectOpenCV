# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 12:57:56 2020

@author: jerqq
"""

import numpy as np
import cv2

cap = cv2.VideoCapture('cars.mp4')
frame = cap[300:, 700:1300]

#fgbg = cv2.createBackgroundSubtractorMOG()
fgbg = cv2.createBackgroundSubtractorMOG2()

while(1):
    ret, frame = cap.read()

    fgmask = fgbg.apply(frame)

    cv2.imshow('frame',fgmask)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()