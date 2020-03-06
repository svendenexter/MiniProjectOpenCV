

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


while True :
    ret, frames = cap.read()
    gray = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)
    cars=car_cascade.detectMultiScale(gray,2,1)
    
    for(x,y,w,h) in cars:
        cv2.rectangle(frames,(x,y),(x+w,y+h),(0,0,255),2)
        font=cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frames,'car',(x+6,y-6),font,0.5,(0,0,255),1)
        
        cv2.imshow('Car Detection',frames)
        
        if cv2.waitKey(33) ==13:
            break
cap.release()
cv2.destroyAllWindows
