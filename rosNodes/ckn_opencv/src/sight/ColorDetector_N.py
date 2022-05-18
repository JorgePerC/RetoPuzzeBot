#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import numpy as np

import gi
gi.require_version('Gtk', '3.0')

class ColorThreshold:

    def __init__(self, inputTopic, outputTopic, t1, t2):
        """
        The thresholds should be formated as: 
            redBajo2=np.array([175, 100, 20], np.uint8)
            redAlto2=np.array([179, 255, 255], np.uint8)
        """

        # Bridge
        self.bridge = CvBridge()
        
        # Subscriber: (to recieve an image) from the camera
        self.image_intake = rospy.Subscriber(inputTopic, Image, self.callback)
        
        # Publisher: (to make the image cv2 compatible)
        self.image_res = rospy.Publisher(outputTopic, Image, queue_size=10)
        
        self.thresLow = t1
        self.thresHigh = t2

        #setup node
        rospy.init_node("ColorThreshold")
        self.rate = rospy.Rate(10)
        

    def callback(self, data):
        # Input reciever:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #TODO: MANIPULATE THE IMAGE AS YOU'D LIKE
        result = self.colorHSVThreshold(cv_image)

        cv2.imshow("Image window", result)
        cv2.waitKey(3)
        
        # Output publisher
        try:
            self.image_res.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def colorHSVThreshold(self, img):
        img = cv2.medianBlur(img, 5)

        frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        mask2 = cv2.inRange(frameHSV, self.thresLow, self.thresHigh)
        
        maskYellowvis = cv2.bitwise_and(img, img, mask=mask2)

        return maskYellowvis


    def stop (self):
        pass

if __name__ == "__main__":

    redBajo = np.array([175, 100, 20], np.uint8)
    redAlto = np.array([179, 255, 255], np.uint8)

    subsChannel = "video_source/raw"
    publishChannel =  "cv2RawImg"

    vision = ColorThreshold(subsChannel, publishChannel, redBajo, redAlto)
    
    rospy.spin()

    try:
        pass
    except rospy.ROSInterruptException:
        None