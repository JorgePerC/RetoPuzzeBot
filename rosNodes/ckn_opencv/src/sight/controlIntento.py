#!/usr/bin/env python

from threading import ThreadError
from cv2 import threshold
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
        result = self.sectionControl(cv_image)

        cv2.imshow("Image window", result)
        cv2.waitKey(3)
        self.count += 1
        # Output publisher
        try:
            self.image_res.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def sectionControl(self, frame):
        frame = cv2.resize(frame, (720,480))
        frame = self.verticalSplitImg(frame, 10, 9)
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gray + blur
        grayImg = cv2.medianBlur(grayImg, 7)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 90, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) # , works great THRESH_TOZERO_INV
                                                    #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                    # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
        
        kernel = np.ones((5, 5), np.uint8)

        thresh = cv2.erode(thresh, kernel, iterations= 3)
        
        res = self.drawMiddleLineBlob(thresh)

        return res

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
    
    def drawMiddleLineBlob(self, thresh):
        
        # Get the upper x coordinates
        xFirstChange = []
        xLastChange = []
        last_p = 0
        for p in range(thresh.shape[1]):
            if last_p != thresh[0, p]:
                xFirstChange.append(p)
            last_p = thresh[0, p]
        
        # Second part
        last_p = 0
        for p in range(thresh.shape[1]):
            if last_p != thresh[-1, p]:
                xLastChange.append(p)
            last_p = thresh[-1, p]
        print(len(xFirstChange), len(xLastChange))

        upprdXs = []
        lowerdXs = []

        for x in range(0, thresh.shape[1], 2):
            print("   ", x)
            upprdXs.append(((xFirstChange[x] + xFirstChange[x+1])/2, 0))
        lowerdXs = [((xLastChange[x] + xLastChange[x+1])/2, 0) for x in range(0, thresh.shape[1], 2)]

        res  = cv2.cvtColor(thresh, cv2.COLOR_BGRA2BGR)
        for i in len(lowerdXs):
            cv2.line(res, (upprdXs[i], 0), (lowerdXs[i], thresh.shape[0]), (255, 100 , 0), 3)

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
