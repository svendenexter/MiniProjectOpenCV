# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 12:08:53 2020

@author: stefa
"""

import numpy as np
import cv2
cap = cv2.VideoCapture('cars.mp4')

kernel = np.ones((9,9),np.uint8)
kernel1 = np.ones((10,5),np.uint8)
kernel2 = np.ones((30,1),np.uint8)

detector = cv2.SimpleBlobDetector_create()
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
# Change thresholds
params.minThreshold = 150;
params.maxThreshold = 255;
# Filter by Area
params.filterByArea = True
params.minArea = 500
params.maxArea = 50000
# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.5
#Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.87
# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0

detector = cv2.SimpleBlobDetector_create(params)

fgbg = cv2.createBackgroundSubtractorMOG2(255,255,0)

car_point = 0,0
new_car = 0
car_counter = 0

last_car_y1 = 0
cars_lane1 = 0
last_car_y2 = 0
cars_lane2 = 0
last_car_y3 = 0
cars_lane3 = 0
last_car_y4 = 0
cars_lane4 = 0
last_car_y5 = 0
cars_lane5 = 0

nextframe = 1
array = []
while(1):    
    if (nextframe == 1): # if w is pressed get a frame
        ret, frame = cap.read() # read next frame
        nextframe = 0
        if (ret == True): # if frame is not empty
            framecropped = frame[200:400, 100:1800] # slice out the detect area
            fgmask = fgbg.apply(framecropped) # background filter (only display the movening objects)
            dilation = cv2.dilate(fgmask, kernel1, iterations = 3) # make the found blobs bigger 3 times
            erosion = cv2.erode(dilation,kernel2,iterations = 3) # make the blob smaller 3 times
            dilation2 = cv2.dilate(erosion, kernel, iterations = 1) # make the blob bigger 1 time     
            ret,thresh1 = cv2.threshold(dilation2,5,255,cv2.THRESH_BINARY_INV) # inverse the picture
            keypoints = detector.detect(thresh1)    # find keypoints
            im_with_keypoints = cv2.drawKeypoints(thresh1, keypoints, np.array([]), # draw green cirkel keypoint
                                                  (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            #empty the array of car X and y, is needed because you find sometimes 1 car and sometimes 3
            car_x = [0,0,0,0,0,0,0,0,0]
            car_y = [0,0,0,0,0,0,0,0,0]
            
            for i in range ((len(keypoints))):        
                car_point = keypoints[i].pt  # get the x and y positions
                car_x[i] = int (car_point[0]) # place the x position in an array
                car_y[i] = int (car_point[1]) # place the y position in an array
#                car_x = (car_x)
#                car_y = (car_y)
            #place the array in 1 a 2 dimensional array
            array = [car_x ,car_y]    
            
            for i in range (len(array)): # for every car found display the lane and count them in each lane
                if (array[1][i] != 0):
                    if (array[0][i]> 100 and array[0][i] < 350):                    
                        cv2.putText(im_with_keypoints, "lane 1",(200,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y1):
                            cars_lane1 = cars_lane1 + 1
                        last_car_y1 = array[1][i]                    
                    if (array[0][i]> 350 and array[0][i] < 500):    
                        cv2.putText(im_with_keypoints, "lane 2",(400,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y2):
                            cars_lane2 = cars_lane2 + 1
                        last_car_y2 = array[1][i]
                    if (array[0][i]> 500 and array[0][i] < 650):
                        cv2.putText(im_with_keypoints, "lane 3",(600,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y3):
                            cars_lane3 = cars_lane3 + 1
                        last_car_y3 = array[1][i]
                    if (array[0][i]> 650 and array[0][i] < 850):
                        cv2.putText(im_with_keypoints, "lane 4",(800,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y4):
                            cars_lane4 = cars_lane4 + 1
                        last_car_y4 = array[1][i]
                    if (array[0][i]> 850 and array[0][i] < 1300):
                        cv2.putText(im_with_keypoints, "lane 5",(1000,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y5):
                            cars_lane5 = cars_lane5 + 1
                        last_car_y5 = array[1][i]
                        
            #display the amount of cars seen each lane
            cv2.putText(im_with_keypoints, str(cars_lane1),(225,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA) 
            cv2.putText(im_with_keypoints, str(cars_lane2),(425,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA) 
            cv2.putText(im_with_keypoints, str(cars_lane3),(575,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA) 
            cv2.putText(im_with_keypoints, str(cars_lane4),(750,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)             
            cv2.putText(im_with_keypoints, str(cars_lane5),(1000,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)  
           
            #calculate the total cars and display
            total_cars = cars_lane1 +cars_lane2 + cars_lane3 + cars_lane4 + cars_lane5
            cv2.putText(im_with_keypoints, str(total_cars),(50,50), cv2.FONT_ITALIC, 2,(255,0,0),2,cv2.LINE_AA) 
           
            #draw lanes
            cv2.line(im_with_keypoints,(100,0),(100,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(350,0),(350,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(500,0),(500,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(650,0),(650,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(850,0),(850,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(1300,0),(1300,500),(255,0,0),5)
            
            # show frames
            cv2.imshow('frame',frame)
            cv2.imshow('framecropped',framecropped)
            cv2.imshow("Keypoints", im_with_keypoints)
            
    k = cv2.waitKey(30) #waits 30 miliseconds for a key to be pressed
    
    if (k == ord("w")): # press W to continue to the next frame or hold it
        nextframe = 1    
    if (k == ord("q")): # pres q to quit the video
        break
cap.release()
cv2.destroyAllWindows()
