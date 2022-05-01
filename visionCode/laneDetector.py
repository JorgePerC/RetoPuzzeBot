from unittest import result
import cv2

from cv2 import threshold
from cv2 import line
import numpy as np
import math

from rospy import on_shutdown

def addMarker(img, p1, p2):
    centerX = int((p1[0] + p2[0])/2)
    centerY = int((p1[1] + p2[1])/2)
    cv2.line(img, (centerX + 10, centerY), (centerX - 10, centerY), (0,0,255), 2)#), cv2.LINE_AA)
    cv2.line(img, (centerX, centerY + 10), (centerX, centerY - 10), (0,0,255), 2)#), cv2.LINE_AA)

def lines(frame):
    # https://medium.com/@tomasz.kacmajor/hough-lines-transform-explained-645feda072ab
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

def linesProbalistic(frame):
    
    img = frame.copy()
    img =cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    lines = cv2.HoughLinesP(frame, rho = 6, theta = np.pi/180, threshold = 350, maxLineGap=80)

    # draw Hough lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (255, 255 , 0), 3)
    
    return img


def verticalSplitImg(frame, ratio):

    dif = ratio-1
    # height calculus:
    h = frame.shape[0]/ratio
    ceros = np.zeros((int(h), frame.shape[1]), dtype = np.uint8)

    ones = np.ones((int(h)*dif, frame.shape[1]), dtype = np.uint8)

    myMask = np.concatenate((ceros, ones), axis = 0)

    frame = cv2.bitwise_and(frame, frame, mask = myMask)

    return frame

path = r"./visionCode/sampleImages/normalSlow.mp4"
cap = cv2.VideoCapture(path) 
white1 = np.array([0, 0, 0], np.uint8)
white2 = np.array([180, 10, 0], np.uint8)

while cap.isOpened():
    ret, frame = cap.read()

    # To limit our working area
    # In this case, we'll be working with only
    # the upper part of the image, to reduce noise
    frame = verticalSplitImg(frame, 3)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Limit colors:
    hueThreshold = cv2.inRange(hsv,white1, white2 )
    
    # Gray dodoos ================================
    # RGB -> Gray
    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Gray + blur
    grayImg = cv2.medianBlur(grayImg, 5)
    # Gray -> Threshold
    ret, thresh = cv2.threshold(grayImg, 180, 255, cv2.THRESH_TOZERO) # , works great
                                                #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
                            
    # Find inital line segments ==================
    canny = cv2.Canny(thresh, 100, 200) 
    # Find lines
    hough = linesProbalistic(canny)

    # Merge all images into a sigle one
    grayImg = cv2.cvtColor(grayImg, cv2.COLOR_GRAY2BGR)
    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    hueThreshold = cv2.cvtColor(hueThreshold, cv2.COLOR_GRAY2BGR)
    canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
    

    row1 = np.concatenate((frame, hough), axis= 1)
    cv2.imshow("Frame", row1)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()