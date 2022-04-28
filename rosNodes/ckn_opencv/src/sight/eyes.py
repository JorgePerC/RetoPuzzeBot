#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image

def gstreamer_pipeline( 
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=0 ):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class cameraPublisher:
    def __init__(self, inputNumber = -1):
        if inputNumber == -1:
            self.cap =  cv2.VideoCapture(gstreamer_pipeline(flip_method=0, framerate=30), cv2.CAP_GSTREAMER)
        else:
            self.cap = cv2.VideoCapture(inputNumber)
        
        # Publisher: 
        self.image_pub = rospy.Publisher("robotSight", Image, queue_size=10)
        
        #setup node
        rospy.init_node("RobotCamera")
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

     
    def captureAndSend(self):
        while not rospy.is_shutdown():
            readSuccess, frame = self.cap.read()
            print(type(frame))
 
            self.image_pub.publish(frame)
            self.rate.sleep()

    def stop (self):
        # Free camera stream
        self.cap.release()
        # Closes all windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    vision = cameraPublisher(0)

    vision.captureAndSend()
    try:
        pass
    except rospy.ROSInterruptException:
        None
    
    vision.stop()