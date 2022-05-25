#!/usr/bin/env python

from threading import ThreadError
from cv2 import split, threshold
from yaml import load
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import numpy as np

import gi
gi.require_version('Gtk', '3.0')

class LineDetector:

    def __init__(self, inputTopic, outputTopic):

        # Bridge
        self.bridge = CvBridge()
        
        # Subscriber: (to recieve an image) from the camera
        self.image_intake = rospy.Subscriber(inputTopic, Image, self.callback)
        
        # Publisher: (to make the image cv2 compatible)
        self.image_res = rospy.Publisher(outputTopic, Image, queue_size=10)

        self.count = 0
        #setup node
        rospy.init_node("LineDetector")
        self.rate = rospy.Rate(10)
        

    def callback(self, data):
        # Input reciever:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #TODO: MANIPULATE THE IMAGE AS YOU'D LIKE
        result = self.sectionControl(cv_image, 10, 3)

        cv2.imshow("Image window", result)
        cv2.waitKey(3)
        self.count += 1
        # Output publisher
        try:
            self.image_res.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def sectionControl(self, frame, splits, sections2Analyse = 1):
        frame = cv2.resize(frame, (720,480))
        frame = self.verticalSplitImg(frame, splits, splits- sections2Analyse)
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gray + blur
        grayImg = cv2.medianBlur(grayImg, 7)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 90, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) 
        
        kernel = np.ones((5, 5), np.uint8)

        thresh = cv2.erode(thresh, kernel, iterations= 2)
        
        sectors = []
        for i in range(sections2Analyse): 
            section = self.verticalSplitImg_gray(thresh, ratio= sections2Analyse, specificSegm= i)
            sectionRes = self.drawMiddleLineBlob(section)
            sectors.append(sectionRes)
        print("=====")
        #sectors = sectors[-1: : -1]        

        result = np.concatenate(sectors, axis = 0)

        
        return result

    def linesProbalistic(self, frame):
        frame = cv2.resize(frame, (720,480))
        frame = self.verticalSplitImg(frame, 10, 9)
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gray + blur
        grayImg = cv2.medianBlur(grayImg, 7)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 90, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) # , works great THRESH_TOZERO_INV
                                                    #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                    # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
                                
        # Find inital line segments ==================
        # canny = cv2.Canny(thresh, 100, 200)
        kernel = np.ones((5, 5), np.uint8)

        thresh = cv2.erode(thresh, kernel, iterations= 3)


        img = thresh.copy()
        img =cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        lines = cv2.HoughLinesP(thresh, rho = 20, theta = np.pi/15, threshold = 300, minLineLength=120, maxLineGap = 5)#
        #lines = cv2.HoughLinesP(canny, rho = 1, theta = np.pi/180, threshold = 30 , maxLineGap=20) #, minLineLength= 25

        last_slope = 1
        newlines = []
        # draw Hough lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                this_slope = (y2-y1)/(x2-x1)

                # If slopes are similar
                if (last_slope - 2.05) <= this_slope <= (last_slope + 2.05):
                    pass
                else:
                    cv2.line(img, (x1, y1), (x2, y2), (255, 100 , len(newlines)*30), 3)
                    newlines.append(line)
                    
                last_slope = this_slope
        
        #cv2.imwrite("./img_d2{}.jpeg".format(self.count), img)

        return img, newlines
    
    
    def lines(self, frame):
        frame = cv2.resize(frame, (720,480))
        frame = self.verticalSplitImg(frame, 10, 4)

        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gray + blur
        grayImg = cv2.medianBlur(grayImg, 5)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 90, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) # , works great THRESH_TOZERO_INV
                                                    #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                    # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
                                
        # Find inital line segments ==================
        canny = cv2.Canny(thresh, 100, 200)

        img = canny.copy()
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        lines =  cv2.HoughLines(canny, rho = 5, theta = np.pi/180, threshold = 200, srn=0, stn=0 ) #, min_theta = (0.6)*np.pi, max_theta = 3*np.pi/2

        if lines is not None:
            idx = 0
            while idx < len(lines):
                rho = lines[idx][0][0]
                theta = lines[idx][0][1]
                a = np.cos(theta)
                b = np.sin(theta)

                x0 = a*rho
                y0 = b*rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(img, pt1, pt2, (0,255,255), 3, cv2.LINE_AA)

                idx += 1
        return img  

    def verticalSplitImg(self, frame, ratio, remSegm = 1):
        
        dif = ratio-remSegm
        
        # height calculus:
        h = int(frame.shape[0]/ratio)
        
        frame = frame[h*remSegm : -1, :, :] 
        
        return frame
    
    def verticalSplitImg_gray(self, frame, ratio, specificSegm = 1):
        # height calculus:
        h = int(frame.shape[0]/ratio)

        d = specificSegm+1  
        frame = frame[h*(specificSegm): h*d, :]
        print(frame.shape, ratio, specificSegm-1, d)
        return frame
    
    
    def drawMiddleLineBlob(self, thresh):
        # Erode one more time
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.erode(thresh, kernel, iterations= 1)
        
        # Get the upper x coordinates
        xFirstChange = []
        xLastChange = []
        last_p = 0
        for p in range(thresh.shape[1]):
            # Add a change when last one is 1
            if p == thresh.shape[1]-1 and thresh[0, p] == 1:
                xFirstChange.append(p)
            # Chech if a change is present
            if last_p != thresh[0, p]:
                xFirstChange.append(p)
            last_p = thresh[0, p]
        
        # Second part
        last_p = 0
        for p in range(thresh.shape[1]):
            # Add a change when last one is 1
            if p == thresh.shape[1]-1 and thresh[-1, p] >= 200:
                xLastChange.append(p)
            if last_p != thresh[-1, p]:
                xLastChange.append(p)
            last_p = thresh[-1, p]

        # Return if # changes is different        
        print(len(xFirstChange), len(xLastChange))
        if len(xFirstChange) != len(xLastChange) or (len(xFirstChange)% 2 != 0 ):
            return cv2.cvtColor(thresh, cv2.COLOR_BGRA2BGR)

        # Convert coordinates to middle points
        upprdXs = []
        lowerdXs = []

        upprdXs = [(int((xFirstChange[x] + xFirstChange[x+1])/2), 0) for x in range(0, len(xFirstChange), 2)]
        lowerdXs = [(int((xLastChange[x] + xLastChange[x+1]))/2, thresh.shape[0]) for x in range(0, len(xLastChange), 2)]

        # Draw lines
        res  = cv2.cvtColor(thresh, cv2.COLOR_BGRA2BGR)
        slopes = []
        for i in range(len(lowerdXs)):
            cv2.line(res, upprdXs[i], lowerdXs[i], (255, 255 , 0), 3)
            # Calculate and append slope
            slopes.append( float( (upprdXs[i][1]+lowerdXs[i][1]) /(upprdXs[i][0]+lowerdXs[i][0])))

        return res
    def stop (self):
        pass

if __name__ == "__main__":

    subsChannel = "video_source/raw"
    publishChannel =  "cv2RawImg"

    vision = LineDetector(subsChannel, publishChannel)
    
    rospy.spin()

    try:
        pass
    except rospy.ROSInterruptException:
        None
