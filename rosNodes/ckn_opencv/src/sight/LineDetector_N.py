#!/usr/bin/env python

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
        result = self.linesProbalistic(cv_image)

        cv2.imshow("Image window", result)
        cv2.waitKey(3)
        
        # Output publisher
        try:
            self.image_res.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except CvBridgeError as e:
            print(e)

    
    def linesProbalistic(self, frame):
        frame = cv2.resize(frame, (720,480))
        frame = self.verticalSplitImg(frame, 5, 3)
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gray + blur
        grayImg = cv2.medianBlur(grayImg, 5)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 90, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) # , works great THRESH_TOZERO_INV
                                                    #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                    # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
                                
        # Find inital line segments ==================
        canny = cv2.Canny(thresh, 100, 200)
        kernel = np.ones((7, 7), np.uint8)

        canny = cv2.dilate(canny, kernel, iterations= 1)


        img = canny.copy()
        img =cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        lines = cv2.HoughLinesP(canny, rho =3, theta = np.pi/180, threshold = 300, minLineLength=90, maxLineGap = 10)#
        #lines = cv2.HoughLinesP(canny, rho = 1, theta = np.pi/180, threshold = 30 , maxLineGap=20) #, minLineLength= 25

        # draw Hough lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (255, 255 , 0), 3)
        
        return img
    
    
    def lines(self, frame):
        frame = cv2.resize(frame, (720,480))
        frame = self.verticalSplitImg(frame, 5, 3)

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

    def verticalSplitImg(self, frame, ratio, quarter = 1):
        
        dif = ratio-quarter
        # height calculus:
        h = int(frame.shape[0]/ratio)
        ceros = np.zeros((h*dif, frame.shape[1]), dtype = np.uint8)

        ones = np.ones((h*quarter, frame.shape[1]), dtype = np.uint8)

        myMask = np.concatenate((ceros, ones), axis = 0)

        frame = cv2.bitwise_and(frame, frame, mask = myMask)

        frame = frame[h*quarter : -1, :, :]

        return frame


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
