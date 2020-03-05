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
cap = cv2.VideoCapture('cars.mp4')
#cap = cv2.VideoCapture('road.mp4')

kernel = np.ones((8,8),np.uint8)

detector = cv2.SimpleBlobDetector_create()
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()


# Change thresholds
params.minThreshold = 150;
params.maxThreshold = 255;
# Filter by Area
params.filterByArea = True
params.minArea = 9000
params.maxArea = 90000
# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.5
#Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.87
# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0

ver = (cv2.__version__).split('.')

# Create a detector with the parameters
if int(ver[0]) < 3 :
    detector = cv2.SimpleBlobDetector(params)
else :
    detector = cv2.SimpleBlobDetector_create(params)

fgbg = cv2.createBackgroundSubtractorMOG2(255,255,0)
car_point = 0,0
new_car = 0
car_counter = 0
while(1):   
    ret, frame = cap.read()
#    if (frame.empty()
    if (ret == True):
        #    framecropped = frame[300:600, 100:1800]
#        frame = frame[200:1000, 600:780]
        h,w=frame.shape[0:2]
        base_size=h+100,w+100,3
#        # make a 3 channel image for base which is slightly larger than target img
        base=np.zeros(base_size,dtype=np.uint8)
#      #  cv2.rectangle(base,(0,0),(w+20,h+20),(255,255,255),30) # really thick white rectangle
        base[50:h+50,50:w+50]=frame # this works
        
        fgmask = fgbg.apply(base) # background filter
        closing = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel) # closing the blob
#        erosion = cv2.erode(fgmask,kernel,iterations =1)   # is not used
        
        ret,thresh1 = cv2.threshold(closing,10,255,cv2.THRESH_BINARY_INV) # inverse the picture
        keypoints = detector.detect(thresh1)    # find keypoints
        im_with_keypoints = cv2.drawKeypoints(thresh1, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
#        cv2.imshow("Keypoints", im_with_keypoints)
#        cv2.imshow('erosion',erosion)
        print( len(keypoints))
        print (keypoints)
        if (len(keypoints) >0):
            last_car_point = car_point
            car_point = keypoints[0].pt
            print (car_point)
            cv2.circle(im_with_keypoints,(int (car_point[0]),int (car_point[1])), 63, (0,0,255), -1)
            
            if (car_point[1] > last_car_point[1]):
                new_car = 1
            if (new_car == 1):
                car_counter = car_counter + 1
                new_car = 0
            cv2.putText(im_with_keypoints, str(car_counter),(int(car_point[0]-20),int (car_point[1]+20)), cv2.FONT_ITALIC, 2,(0,0,0),2,cv2.LINE_AA)     
        
        
        print(new_car)
        print("counted cars  = ",car_counter)

        cv2.imshow('base',base)
        cv2.imshow('frame',fgmask)
        cv2.imshow('frame1',closing)    
        cv2.imshow("Keypoints", im_with_keypoints)
            
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
