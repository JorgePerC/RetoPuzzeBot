#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image

class VisionClass:
    def _init_(self, inputTopic, outputTopic):

        # Bridge
        self.bridge = CvBridge()
        
        # Subscriber: (to recieve an image) from the camera
        self.image_intake = rospy.Subscriber(inputTopic, Image, self.callback)
        
        # Publisher: (to make the image cv2 compatible)
        self.image_res = rospy.Publisher(outputTopic, Image, queue_size=10)
        
        #setup node
        rospy.init_node("name")
        self.rate = rospy.Rate(10)

    def callback(self, data):
        # Input reciever:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #TODO: MANIPULATE THE IMAGE AS YOU'D LIKE

        # Output publisher
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def stop (self):
        pass

if _name_ == "_main_":

    vision = VisionClass()
    
    rospy.spin()

    try:
        pass
    except rospy.ROSInterruptException:
        None