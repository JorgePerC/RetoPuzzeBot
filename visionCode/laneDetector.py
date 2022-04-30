from unittest import result
import cv2

from cv2 import threshold
from cv2 import line
import numpy as np
import math

def addMarker(img, p1, p2):
    centerX = (p1[0] +p2[0])/2
    centerY = (p1[1] +p2[1])/2
    cv2.line(img, (centerX+5, centerY), (centerX-5, centerY), (0,0,255), 2)#), cv2.LINE_AA)
    cv2.line(img, (centerX, centerY+5), (centerX, centerY-5), (0,0,255), 2)#), cv2.LINE_AA)

def lines(frame):

    result = frame.copy()

    result =cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)

    lines =  cv2.HoughLines(frame, rho = 5, theta = np.pi/180, threshold = 400, srn=0, stn=0 ) #, min_theta = (0.6)*np.pi, max_theta = 3*np.pi/2

    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)

            x0 = a*rho
            y0 = b*rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(result, pt1, pt2, (0,255,255), 3, cv2.LINE_AA)
            addMarker(result, pt1, pt2)
    
    return result 

path = r"./visionCode/sampleImages/normalSlow.mp4"
cap = cv2.VideoCapture(path) 
white1 = np.array([0, 0, 0], np.uint8)
white2 = np.array([180, 10, 0], np.uint8)
while cap.isOpened():
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Limit colors:
    hueThreshold = cv2.inRange(hsv,white1, white2 )
    # RGB -> Gray
    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Gray dodoos

    # Gray + blur
    grayImg = cv2.medianBlur(grayImg, 5)

    # Gray -> Threshold
    ret, thresh = cv2.threshold(grayImg, 180, 255, cv2.THRESH_TOZERO) # , works great
                                                #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
    canny = cv2.Canny(thresh, 100, 200) 
    hough = lines(canny)

    # Merge all images into a sigle one
    grayImg = cv2.cvtColor(grayImg, cv2.COLOR_GRAY2BGR)
    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    hueThreshold = cv2.cvtColor(hueThreshold, cv2.COLOR_GRAY2BGR)
    #hough = cv2.cvtColor(hough, cv2.COLOR_GRAY2BGR)
    

    row1 = np.concatenate((frame, hueThreshold), axis= 1)
    cv2.imshow("Frame", row1)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()