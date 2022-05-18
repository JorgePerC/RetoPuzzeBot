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
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gray + blur
        grayImg = cv2.medianBlur(grayImg, 5)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 110, 0, cv2.THRESH_TOZERO_INV) # , works great
                                                    #cv2.THRESH_OTSU+cv2.THRESH_TOZERO) # cv2.THRESH_TOZERO, works great
                                                    # cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
                                
        # Find inital line segments ==================
        canny = cv2.Canny(thresh, 100, 200)

        img = canny.copy()
        img =cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        lines = cv2.HoughLinesP(canny, rho = 6, theta = np.pi/180, threshold = 350, maxLineGap=80)

        # draw Hough lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (255, 255 , 0), 3)
        
        return canny




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
