#!/usr/bin/env python

from cv2 import circle
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image

import numpy as np

class CircleDetector:

    def __init__(self, inputTopic, outputTopic):

        # Bridge
        self.bridge = CvBridge()
        
        # Subscriber: (to recieve an image) from the camera
        self.image_intake = rospy.Subscriber(inputTopic, Image, self.callback)
        
        # Publisher: (to make the image cv2 compatible)
        self.image_res = rospy.Publisher(outputTopic, Image, queue_size=10)
        
        #setup node
        rospy.init_node("CircleDetector_Node")
        self.rate = rospy.Rate(10)

    def callback(self, data):
        # Input reciever:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #TODO: MANIPULATE THE IMAGE AS YOU'D LIKE

        result = self.detectCircles(cv_image.copy())

        # Output publisher
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def detectCircles(self, image):

        grayImg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(grayImg, 
                                cv2.HOUGH_GRADIENT_ALT, # for better accuracy
                                1, # for evalualuating the image on same dimentions
                                minDist = 5, # Distance between circle center, if smaller, circle is discarted   
                                param1 = 300, # Dunno https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
                                param2 = 0.9, # 0-1, how much of a circle?
                                minRadius = 5 , maxRadius = 0)

        if len(circles) == 0:
            return image 
        circles = np.uint16(np.around(circles))

        for i in circles[0, :]:
            # dibujar circulo
            cv2.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 3)
            # dibujar centro
            cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)

        
        _, threshold = cv2.threshold(grayImg, 110, 255,
                                    cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 400:
                approx = cv2.approxPolyDP(cnt,
                                        0.009 * cv2.arcLength(cnt, True), True)

                if (len(approx) == 7):
                    cv2.drawContours(grayImg, [approx], 0, (0, 0, 255), 5)

        return image


    def stop (self):
        pass

if __name__ == "__main__":

    vision = CircleDetector()
    
    rospy.spin()

    try:
        pass
    except rospy.ROSInterruptException:
        None