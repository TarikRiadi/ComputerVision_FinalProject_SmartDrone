# -*- coding: utf-8 -*-
"""
Object Detection by Shape & Color v3
Created on Thu Jun  6 12:28:14 2019

@author: tarik
"""

import cv2
import numpy as np
import time

#cap = cv2.VideoCapture('video5.avi') #Capture video through webcam.
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_COMPLEX #Font to write in image.
fps = 30
#video_name = 'video5_Detected_area-5000.avi'
#video = cv2.VideoWriter(video_name,cv2.VideoWriter_fourcc(*'DIVX'),fps,(960,720)) #Create video object. Drone cam dims: (960,720)

#Define color ranges in HSV
lower =  {'red': (0,170,70),   'red2': (160,130,50),  'green':(35,86,37),   'blue':(110,50,0),   'yellow':(16,100,100)}
upper =  {'red': (10,255,255), 'red2': (180,255,255), 'green':(70,250,194), 'blue':(125,255,255),'yellow':(24,255,255)}
colors = {'red':(0,0,255),     'red2':(0,0,255),      'green':(0,255,0),    'blue':(255,0,0),    'yellow':(0,255,255)}

#Begin Live Detection
while True:
    _,frame = cap.read() #Get frame matrix.
    orig_frame = frame.copy()
    frame = cv2.GaussianBlur(frame,(15,15),0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #Transform colorspace from BGR to HSV.
    #Threshold the original image
    for key, value in upper.items():
        kernel = np.ones((1,1), np.uint8) #Kernel of 1s in uint8.
        mask = cv2.inRange(hsv, lower[key], upper[key]) #Find objects that match the color range defined. Matched objects are white.
        #Morphology transformations to "erase" holes in the images.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        rows = mask.shape[0]
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, rows,
                                   param1=200, param2=15,
                                   minRadius=30, maxRadius=80) #param1: CannyEdge; param2: radius size.
        if circles is not None: #First Circles
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(orig_frame, center, 1, colors[key], 3) #Draw the center.
                # circle outline
                radius = i[2]
                cv2.circle(orig_frame, center, radius, colors[key], 2) #Draw the circle with recognized color.
                cv2.putText(orig_frame, "Circle", center, font, 0.7, colors[key], 2)

        for cnt in contours: #After detecting circles, detect triangles and squares.
            area = cv2.contourArea(cnt) #Get the area of the object in pixels.
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True) #"Resolution" of polygons.
            x = approx.ravel()[0] #Determine horizontal position.
            y = approx.ravel()[1] #Determine vertical postition.
            if area > 3000: #Only detects objects larger than. 5000 close, 1500 avoid background, 10000 very close.
                #Draw the contour on top of the frame and write the shape detected.
                if len(approx) == 3:
                    cv2.drawContours(orig_frame, [approx], 0, colors[key], 2)
                    cv2.putText(orig_frame, "Triangle", (x,y), font, 0.7, colors[key], 2)
                elif len(approx) == 4:
                    (x,y,w,h) = cv2.boundingRect(approx)
                    ar = w/float(h)
                    if ar >= 0.95 and ar <= 1.05:
                        cv2.drawContours(orig_frame, [approx], 0, colors[key], 2)
                        cv2.putText(orig_frame, "Square", (x,y), font, 0.7, colors[key], 2)
                    #else:
                    #    cv2.putText(frame, "Rectangle", (x,y), font, 0.7, colors[key], 2)

    #video.write(frame)
    cv2.imshow("Frame",orig_frame) #Show the live frame.
    key = cv2.waitKey(1) #If esc is pressed, stop capturing.
    if key == 27: #Key 27 = Esc
        break
    time.sleep(1/fps)
#video.release()
cap.release()
cv2.destroyAllWindows()
