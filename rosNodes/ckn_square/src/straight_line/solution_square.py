#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math


class square:
    def __init__(self):
        #initialise wheel velocity variables
        self.wr = 0.0
        self.wl = 0.0

        #Setup ROS subscribers and publishers
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)

        self.w_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        #setup node
        rospy.init_node("Square")
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

        # Define robot params:
        self.robot = {"R": 0.0505, "L": 0.166, "xPos": 0, "yPos": 0, "theta": 0}

    # Callbacks for wheel velocities and commands
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

    # Main function
    def run(self):
        #Variable initialisations
        distance = 0.0
        angle = 0.0
        current_time = rospy.get_time()
        last_time = rospy.get_time()
        state = 0
        count = 1
        no_sides = 4

        # Create message for publishing
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        # Main Loop
        while not rospy.is_shutdown():
            # Compute time since last loop
            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time

            # Update distance and angle from the velocity measurements
            distance += 0.05 * (self.wr + self.wl) * 0.5 * dt
            angle += 0.05 * (self.wr - self.wl) / 0.18 * dt
            self.wr = 0
            self.wl = 0
            # state 0 = moving along a straight
            if state == 0:
                go2 = (1,0)
                self.controlRobot(go2, 0)
                # If at end of a side
                if distance > 0.5:
                    #Reset distance
                    distance = 0.0
                    # If we have not finished the square
                    if count < no_sides:
                        # go to corner and count how many sides are done
                        state = 1
                        count += 1
                    # otherwise we have finished a square
                    else:
                        #stop
                        state = 2
            # State 1 = Turning a corner
            elif state == 1:
                self.goToAngle(math.pi/2)
                # If finished turning through 90 degrees
                if angle > math.pi/2:
                    # Go back to moving straight
                    angle = 0.0
                    state = 0
            # State 3 = Motion completed
            elif state == 2:
                # Stop and signal
                msg.linear.x = 0
                msg.angular.z = 0
                print("Motion Completed")
                rospy.signal_shutdown("Square Completed")
            # If invalid state, stop
            else:
                msg.linear.x = 0
                msg.angular.z = 0

            # Publish message and sleep
            self.w_pub.publish(msg)

            self.rate.sleep()
            
    # Separate stop function for stopping when ROS shuts down
    def stop(self):
        print("Stopping")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.w_pub.publish(msg)
    
    def controlRobot(self, desPos, desAngle):
        
        current_time = rospy.get_time()
        last_time = rospy.get_time()

        # Create message for publishing
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        # Robot actual coordenates:
        # x = self.robot["xPos"]
        # y = self.robot["yPos"]

        # Control constants
        kt = 0.7
        ka = 0.2
        kb = -0.3
        x_e = 1
        y_e = 1
        print("in control robot")
        while not (abs(x_e) <= 0.005 and abs(y_e) <= 0.005):

            # Compute time since last loop
            current_time = rospy.get_time()
            d_t = current_time - last_time
            last_time = current_time
            
            # Error to desired position
            x_e =  desPos[0] - self.robot["xPos"]
            y_e =  desPos[1] - self.robot["yPos"]
            
            # ArcTan(x/y) = angle |   desY - presentY, desX - presentX |
            objectiveAngle = math.atan2(desPos[1] - self.robot["yPos"], desPos[0] - self.robot["xPos"])
            # Error to angle from desAngle
            thetae = self.robot["theta"] - desAngle
            # Distance to objective
            rho = math.sqrt(x_e**2 + y_e**2)
            # Theta to objectivePos
            alpha = objectiveAngle - self.robot["theta"]
            # Control angle (output)
            beta = - thetae - alpha

            v = kt*rho
            omega = kb*( self.robot["theta"] - objectiveAngle) #ka*alpha + kb*beta #
        
            if v > 1:
                v = 0.8
            
            if omega > math.pi/2:
                omega = math.pi/2
            
            if omega < -math.pi/2:
                omega = -math.pi/2
            
        
            msg.linear.x = v
            msg.angular.z = omega
            
            # write messages
            self.w_pub.publish(msg)

            #-------------------------------------------
            # Robot velocity
            V, w = self.getRobotVels()

            x_d = V* math.cos(self.robot["theta"])
            y_d = V* math.sin(self.robot["theta"])
            w_d = w

            # Update robot pos:
            self.robot["xPos"] += x_d*d_t
            self.robot["yPos"] += y_d*d_t
            # Update robot angle:
            self.robot["theta"] +=  w_d*d_t
            # COndition to remain on the tang2 boundaries
            if self.robot["theta"] > math.pi:
                self.robot["theta"] -= 2*math.pi
            elif self.robot["theta"] < -math.pi:
                self.robot["theta"] += 2*math.pi

            print("     vel :", msg.linear.x, " | ang :", msg.angular.z)
            print("x_e", x_e, "y_e", y_e, "theta_e", beta )
            
            # Wait until execution is needed 
            self.rate.sleep()
        #stop when finished
        self.stop()
        rospy.Duration.from_sec(1)
        
    def getRobotVels(self):
         # Robot velocity
        V = self.robot["R"]*(self.wr + self.wl)/2
        w = self.robot["R"]*(self.wr - self.wl)/self.robot["L"]
        return V, w

    def goToDistance(self, targetDist):

        # Define messages to:
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        # Wheel velocity in rad/s
        # self.wr -> Right wheel
        # self.wl -> Left wheel
        
        # Compute time since last loop
        current_time = rospy.get_time()
        last_time = rospy.get_time()

        # Reset robot variables
        distTraveled = 0
        self.robot["xPos"] = 0
        self.robot["yPos"] = 0        

        while not rospy.is_shutdown():

            print("Initial while")
            while distTraveled < targetDist:
                # Update time
                current_time = rospy.get_time()
                d_t = current_time - last_time
                last_time = current_time

                # Robot velocity
                V = self.robot["R"]*(self.wr + self.wl)/2
                w = self.robot["R"]*(self.wr - self.wl)/self.robot["L"]

                x_d = V* math.cos(self.robot["theta"])
                y_d = V* math.sin(self.robot["theta"])
                w_d = w

                # Update robot pos:
                self.robot["xPos"] += x_d*d_t
                self.robot["yPos"] += y_d*d_t
                # Update robot angle:
                self.robot["theta"] +=  w_d*d_t

                # Send mesg to keep velocity:
                msg.linear.x = 0.2
                self.w_pub.publish(msg)
                #    msg.angular.z = 0.0

                distTraveled = math.sqrt(self.robot["xPos"]**2 + self.robot["yPos"]**2)
                print("Distance", distTraveled)
                # Wait until next execution is needed
                self.rate.sleep()
            
            msg.linear.x = 0.0
            self.w_pub.publish(msg)

            self.stop()
            rospy.signal_shutdown("Square Completed")

    def goToAngle(self, targetAngle):

        # Define messages to:
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        # Wheel velocity in rad/s
        # self.wr -> Right wheel
        # self.wl -> Left wheel
        
        # Compute time since last loop
        current_time = rospy.get_time()
        last_time = rospy.get_time()

        # Reset robot variables
        self.robot["xPos"] = 0
        self.robot["yPos"] = 0        

        while self.robot["theta"] < targetAngle:
            # Update time
            current_time = rospy.get_time()
            d_t = current_time - last_time
            last_time = current_time

            # Robot angular velocity
            w = self.robot["R"]*(self.wr - self.wl)/self.robot["L"]

            w_d = w

            # Update robot angle:
            self.robot["theta"] +=  w_d*d_t
            e_theta = targetAngle - self.robot["theta"]

            # Send mesg to keep velocity:
            msg.linear.x = 0.0
            msg.angular.z = 0.3
            self.w_pub.publish(msg)

            # Wait until next execution is needed
            self.rate.sleep()
        
        # Stop without killing ROSnode 
        self.stop()
        rospy.Duration.from_sec(1)

    def testEncoder(self, targetDist):
         # Define messages to:
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        distTraveled = 0
        
        while not rospy.is_shutdown():
            # Compute time since last loop
            current_time = rospy.get_time()
            last_time = rospy.get_time()
            
            while distTraveled < targetDist:
                current_time = rospy.get_time()
                d_t = current_time - last_time
                last_time = current_time

                distTraveled += self.robot["R"] * self.wl*d_t
                msg.linear.x = 0.2

                # Publish message and sleep
                self.w_pub.publish(msg)
                print("\t dist: ", distTraveled, " | vel :", msg.linear.x)
                
                # Wait until next execution is needed
                self.rate.sleep()
            
            msg.linear.x = 0.0
            self.w_pub.publish(msg)

            self.stop()
            rospy.signal_shutdown("Square Completed")
            
    def printEncoders(self):
        while not rospy.is_shutdown():
            print(self.wr)
            print(self.wl)
            
    def velocityControl(self, vel, angle):
        v_r = (2*vel + angle*self.robot["L"])/(2*self.robot["R"])
        v_l = (2*vel - angle*self.robot["L"])/(2*self.robot["R"]) 

    

if __name__ == "__main__":
    sq = square()

    try:
        #sq.printEncoders() # Works! :D
        #sq.testEncoder(1)   # Also works! :D
        go2 = [0, 0.5]
        sq.controlRobot(go2, 0)

        go2 = [0.5, 0.5]
        sq.controlRobot(go2, 0)

        go2 = [0.5, 0]
        sq.controlRobot(go2, 0)

        go2 = [0, 0]
        sq.controlRobot(go2, 0)
        # #sq.goToDistance(1)
    except rospy.ROSInterruptException:
        None
