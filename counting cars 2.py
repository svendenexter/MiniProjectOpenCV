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

    if (nextframe == 1):
        ret, frame = cap.read()
        nextframe = 0
#    if (frame.empty()
        if (ret == True):
            frame = frame[350:550, 100:1800]
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
            # delete last keypoints
            car_x = [0,0,0,0,0,0,0,0,0]
            car_y = [0,0,0,0,0,0,0,0,0]
            i=0
            while (i < (len(keypoints))):            
                last_car_point = car_point
                
                car_point = keypoints[i].pt
                car_x[i] = int (car_point[0])
                car_y[i] = int (car_point[1])
    #            array = [car_x,car_y]
    #            car_x = car_x.insert(i,int (car_point[0]))
    #            car_y = car_y.insert(i,int (car_point[1]))
                car_x = (car_x)
                car_y = (car_y)
                i = i +1         
               
            array = [car_x ,car_y]
    
            print (array)
            
            for i in range (len(array)):
                if (array[1][i] != 0):
                    print( "geen nul = ",array[1][i] )
                    
                    if (array[0][i]> 100 and array[0][i] < 300):                    
                        cv2.putText(im_with_keypoints, "lane 1",(200,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        print ("last_car_y1 = ", last_car_y1,"    ,  ", "array[1][i] = ",array[1][i]) 
                        if (array[1][i] > last_car_y1):
                            cars_lane1 = cars_lane1 + 1
                        last_car_y1 = array[1][i]                    
                    if (array[0][i]> 300 and array[0][i] < 500):    
                        cv2.putText(im_with_keypoints, "lane 2",(400,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y2):
                            cars_lane2 = cars_lane2 + 1
                        last_car_y2 = array[1][i]
                    if (array[0][i]> 500 and array[0][i] < 700):
                        cv2.putText(im_with_keypoints, "lane 3",(600,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y3):
                            cars_lane3 = cars_lane3 + 1
                        last_car_y3 = array[1][i]
                    if (array[0][i]> 700 and array[0][i] < 900):
                        cv2.putText(im_with_keypoints, "lane 4",(800,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y4):
                            cars_lane4 = cars_lane4 + 1
                        last_car_y4 = array[1][i]
                    if (array[0][i]> 1100 and array[0][i] < 1300):
                        cv2.putText(im_with_keypoints, "lane 5",(1000,50), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)
                        if (array[1][i] > last_car_y5):
                            cars_lane5 = cars_lane5 + 1
                        last_car_y5 = array[1][i]
                     
            cv2.putText(im_with_keypoints, str(cars_lane1),(200,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA) 
            cv2.putText(im_with_keypoints, str(cars_lane2),(400,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA) 
            cv2.putText(im_with_keypoints, str(cars_lane3),(600,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA) 
            cv2.putText(im_with_keypoints, str(cars_lane4),(800,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)             
            cv2.putText(im_with_keypoints, str(cars_lane5),(1000,100), cv2.FONT_ITALIC, 1,(0,255,0),2,cv2.LINE_AA)  
           
            total_cars = cars_lane1 +cars_lane2 + cars_lane3 + cars_lane4 + cars_lane5
            cv2.putText(im_with_keypoints, str(total_cars),(50,50), cv2.FONT_ITALIC, 2,(255,0,0),2,cv2.LINE_AA) 
           
            cv2.line(im_with_keypoints,(100,0),(100,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(300,0),(300,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(500,0),(500,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(700,0),(700,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(900,0),(900,500),(255,0,0),5)
            cv2.line(im_with_keypoints,(1100,0),(1100,500),(255,0,0),5)
            
            for i in range (len(keypoints)):
                cv2.circle(im_with_keypoints,(car_x[i],car_y[i]), 63, (0,0,255), -1)            
    
                if (car_point[1] > last_car_point[1]):
                    new_car = 1
                if (new_car == 1):
                    car_counter = car_counter + 1
                    new_car = 0
                cv2.putText(im_with_keypoints, str(car_counter),(int(car_point[0]-20),int (car_point[1]+20)), cv2.FONT_ITALIC, 2,(0,0,0),2,cv2.LINE_AA)     
            
            
            print("counted cars  = ",car_counter)
    
            cv2.imshow('base',base)
            cv2.imshow('frame',fgmask)
            cv2.imshow('frame1',closing)    
            cv2.imshow("Keypoints", im_with_keypoints)
                
    k = cv2.waitKey(30)
    if (k == ord("w")):
#    if (cv2.waitKey(1) == ord("w")):
        nextframe = 1
    
    if (k == ord("q")):
        break
cap.release()
cv2.destroyAllWindows()
