#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import gi
gi.require_version('Gtk', '3.0')

class visionCelestial:
    def _init_(self):

        # Bridge
        self.bridge = CvBridge()
        
        # Subscriber: (to recieve an image) from the camera
        self.image_sub = rospy.Subscriber("video_source/raw",Image,self.callback)
        
        # Publisher: (to make the image cv2 compatible)
        self.image_pub = rospy.Publisher("cv2RawImg", Image, queue_size=10)
        
        #setup node
        rospy.init_node("VisionBasicc")
        self.rate = rospy.Rate(10)
        print('metodo 1')

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (640,360), 10, 255)
    
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "nv12"))
        except CvBridgeError as e:
            print(e)

        print('metodo 2')
    def stop (self):
        cv2.destroyAllWindows()

if _name_ == "_main_":

    vision = visionCelestial()
    
    rospy.spin()

    try:
        pass
    except rospy.ROSInterruptException:
        None
    cv2.destroyAllWindows()