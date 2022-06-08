#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math

# Calculations:
def eccentricity_from_moments(moments):
    """Calculates the eccentricity from the moments of the contour"""

    a1 = (moments['mu20'] + moments['mu02']) / 2
    a2 = np.sqrt(4 * moments['mu11'] ** 2 + (moments['mu20'] - moments['mu02']) ** 2) / 2
    ecc = np.sqrt(1 - (a1 - a2) / (a1 + a2))
    return ecc

def compactness_from_moments(moments, contour):
    length = cv2.arcLength(contour, True)
    k = (length * length) / (moments['m00'] * 4 * np.pi)
    return k

def direction_from_moments(moments):
    # https://docs.baslerweb.com/visualapplets/files/manuals/content/examples%20imagemoments.html

    x_mean = moments["m10"] / moments["m00"]
    y_mean = moments["m01"] / moments["m00"]

    u_11 = moments['m11']/moments['m00'] - (x_mean*y_mean)
    u_02 = moments['m02']/moments['m00'] - y_mean**2
    u_20 = moments['m20']/moments['m00'] -  x_mean**2
    a = 2*u_11
    b = u_02 + u_20
    angle = 0.5*np.arctan2(a, b)
    return angle

# ================= Class node
class VisionMoments:
    def __init__(self, inputTopic, outputTopic):

        # Bridge
        self.bridge = CvBridge()
        
        # Subscriber: (to recieve an image) from the camera
        self.image_intake = rospy.Subscriber(inputTopic, Image, self.callback)
        
        # Publisher: (to make the image cv2 compatible)
        self.control_res = rospy.Publisher(outputTopic, Twist, queue_size=10)
        
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
        result, lines = self.vision2Control(cv_image)

        cv2.imshow("Image window", result)
        cv2.waitKey(3)

        if len(lines) == 0:
            stop = Twist()
            stop.linear.x = 0
            stop.linear.y = 0
            stop.linear.z = 0
            stop.angular.x = 0
            stop.angular.y = 0
            stop.angular.z = 0
            self.control_res.publish(stop)

        else:
            despos = self.lines2Pos(lines)

            v, omg = self.p_PoseController(despos)

            control = self.point2Vel(v, omg)
            # Output publisher
            self.control_res.publish(control)

    def point2Vel(self, v, omega):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        msg.linear.x = v
        msg.angular.z = omega

        return msg

    def p_PoseController(self, desPos, kt = 0.05, kb = 0.145):
        # Control constants
        # Error to desired position
        x_e =  desPos[0] - 0.056
        y_e =  desPos[1] 
        
        # ArcTan(x/y) = angle | desY - presentY, desX - presentX |
        theta_e = math.atan2(y_e, x_e)
      
        # Distance to objective
        dist2Objective = math.hypot(x_e, y_e)

        v = kt*dist2Objective
        omega = kb*theta_e #ka*alpha + kb*beta #

        # Limit vel output values
        if v > 0.05:
            v = 0.05
        if v < -0.05:
            v = -0.05
        
        # Limit rotational vel output values
        if omega > 0.1:
            omega = 0.1
        if omega < -0.1:
            omega = -0.1
        
        return v, omega 
    
    def lines2Pos (self, lines, magnitude = 1):
        avgSlope = np.mean(lines)
        #avgSlope += np.pi/2

        x = magnitude * np.cos(avgSlope)
        y = magnitude * np.sin(avgSlope)
        
        return (x, y)

        
    def vision2Control(self, image):
        image = image[200:, :, :]
        
        # Resize image
        image = cv2.resize(image, (720,480))
        # Grayscale it
        grayImg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Blur it
        grayImg = cv2.medianBlur(grayImg, 5)
        # Gray -> Threshold
        ret, thresh = cv2.threshold(grayImg, 90, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) 
                                
        # Image has already been thresholded
        # Make borders big bois
        # thresh = cv2.dilate(thresh, (9,9), iterations=10)

        # Find contours
        contours, hierarchy = cv2.findContours(thresh, method=cv2.CHAIN_APPROX_SIMPLE, mode=cv2.RETR_EXTERNAL)
        print("Totoal contours", len(contours))
        print("=========")

        # Create result variable
        result = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        # Draw contours on res
        cv2.drawContours(result, contours, contourIdx=-1, color=(100, 255, 200), thickness=1, lineType=cv2.LINE_AA)

        c = 0
        lines = []
        # Calculate moments
        for cont in contours:
            M = cv2.moments(cont)
            for i in M:
                if M[i] == 0:
                    M[i] += 1e-5
            if M['m00'] < 1000:
                continue
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # print("Contour area: '{}'".format(M['m00']))

            # print("center X : '{}'".format(cX))
            # print("center Y : '{}'".format(cY))


            ecc = eccentricity_from_moments(M)
            comp = compactness_from_moments(M, cont)
            dir = direction_from_moments(M)

            print("Fig. {} | Ecc:{:.3f} | Compactness: {:.3f} | Direction: {:.3f}".format(c, ecc, comp, dir))

            # Draw middle point
            cv2.circle(result, (cX, cY), 5, (255, 0, 0), -1)
            if (ecc > 0.94 and M['m00'] > 600):
                x, y, w, h = cv2.boundingRect(cont)
                cv2.rectangle(result, (x,y), (x+w,y+h), (0,255,0))
                lines.append(dir)            
            c+=1
        
        return result, lines


    def stop (self):
        print("Stopping")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.control_res.publish(msg)

if __name__ == "__main__":

    subsChannel = "video_source/raw"
    publishChannel =  "/cmd_vel"

    vision = VisionMoments(subsChannel, publishChannel)
    
    rospy.spin()

    try:
        pass
    except rospy.ROSInterruptException:
        None

