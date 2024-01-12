#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

# assume person is standing on the left and in front of the robot
THRES_X = 0.2
THRES_Y = 0.3
TOLER_X = 0.3
TOLER_Y = 0.4
X_UP = THRES_X + TOLER_X
X_LO = THRES_X - TOLER_X
Y_R = THRES_Y - TOLER_Y
Y_L = THRES_Y + TOLER_Y

X_FAC = 0.5
YAW_FAC = 1
DBL_YAW_FAC = 1.5*YAW_FAC
zero_up = 0.01
zero_down = -0.01

class Person_follow:
    def __init__(self):
        pub__topic = "/cmd_vel"
        sub__topic = "/person_tracking"

        self.pub_ = rospy.Publisher(pub__topic, Twist, queue_size=10)
        self.sub_ = rospy.Subscriber(sub__topic, Float64MultiArray, self.follow_callback)
        self.vel_msg = Twist()

    def follow_callback(self, msg):
        
        if (msg.data == [0,0,0,0]):
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        else:
            curx = msg.data[0] # old x 
            cury = msg.data[1] # old y
            newx = msg.data[2] # new x 
            newy = msg.data[3] # new y

        

            # if (cury >= Y_L):
            #     if (curx < X_LO):
            #         self.vel_msg.linear.x = -(curx - X_LO)/X_FAC
            #         self.vel_msg.angular.z = 1.0/YAW_FAC
            #         # self.pub_.publish(self.vel_msg)
            #         # while (cury > Y_L):    
            #         #     continue
            #     elif (curx > X_UP):
            #         self.vel_msg.linear.x = (curx - X_UP)/ X_FAC
            #         self.vel_msg.angular.z = (cury - Y_L)/ YAW_FAC * 0.9
            #     else:
            #         self.vel_msg.linear.x = 0.5
            #         self.vel_msg.angular.z = 0.8    
            # elif (cury <= Y_R):
            #     if (curx < X_LO):
            #         self.vel_msg.linear.x = 0.0
            #         self.vel_msg.angular.z = 0.0
            #     elif (curx > X_UP):
            #         self.vel_msg.linear.x = (curx - X_UP)/ X_FAC
            #         self.vel_msg.angular.z = (cury - Y_R)/ YAW_FAC * 1.5
            #     else:
            #         self.vel_msg.angular.z = -1.0
            # else:
            #     self.vel_msg.angular.z = 0
            #     if (curx < X_LO):
            #         self.vel_msg.linear.x = (curx - X_LO)/ X_FAC
            #     elif (curx > X_UP):
            #         self.vel_msg.linear.x = (curx - X_UP)/ X_FAC
            #     else:
            #         self.vel_msg.linear.x = 0

            diffx = 2*newx - curx
            diffy = 2*newy - cury
            self.vel_msg.linear.x = X_FAC*math.sqrt(diffx**2 + diffy**2)



            if diffx > 0:
                self.vel_msg.angular.z = YAW_FAC*math.atan(diffy/diffx)
            elif diffx < 0:
                self.vel_msg.angular.z = -DBL_YAW_FAC*math.atan(diffy/diffx)
            else:
                self.vel_msg.angular.z = abs(diffy)*YAW_FAC*(math.pi/2)



            # if (curx < X_UP and curx > X_LO and cury < Y_L and cury > Y_R) or (curx < X_UP and curx > X_LO and cury < -Y_L and cury > -Y_R):
            if (curx < X_UP and curx > X_LO and cury < Y_L and cury > Y_R):
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0

            if diffx > zero_down and diffx < zero_up and diffy > zero_down and diffy < zero_up:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
            print(self.vel_msg.linear.x, self.vel_msg.angular.z)
            self.pub_.publish(self.vel_msg)


        



if __name__ == '__main__':
    rospy.init_node('person_follow_py')
    Person_follow()
    rospy.spin()
